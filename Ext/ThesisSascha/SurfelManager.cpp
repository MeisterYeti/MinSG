/*
 * SurfelManager.cpp
 *
 *  Created on: Apr 25, 2014
 *      Author: meisteryeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#include "SurfelManager.h"
#include "Preprocessor.h"
#include "Definitions.h"

#include <MinSG/Core/Nodes/Node.h>
#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/Nodes/ListNode.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Serialization/Serialization.h>

#include <Util/Macros.h>
#include <Util/Utils.h>
#include <Util/StringUtils.h>
#include <Util/IO/FileUtils.h>
#include <Util/Concurrency/Concurrency.h>
#include <Util/Concurrency/UserThread.h>
#include <Util/Concurrency/DataStructures/SyncQueue.h>
#include <Util/Concurrency/Mutex.h>
#include <Util/Concurrency/Lock.h>
#include <Util/Timer.h>

#include <atomic>
#include <deque>
#include <unordered_map>
#include <memory>
#include <forward_list>
#include <array>
#include <limits>
#include <iostream>

#define LOG_STAT(name, value) static StringIdentifier sId ## name(#name); \
	stats->setValue(sId ## name, GenericAttribute::createNumber(value));


namespace MinSG {
namespace ThesisSascha {

using namespace Util;
using namespace Util::Concurrency;
using namespace Rendering;

typedef std::function<void()> Job_t;

const StringIdentifier NO_PARENT("I'm Batman!");

/******************************************
 * WorkerThread class
 ******************************************/

class WorkerThread : public UserThread {
public:
	WorkerThread(SurfelManager* m) : closed(false), closing(false), manager(m) { start(); };
	virtual ~WorkerThread() { close(); };
	void close();
protected:
	void run() override;
	std::atomic<bool> closed;
	std::atomic<bool> closing;
	WeakPointer<SurfelManager> manager;
};

/******************************************
 * Helper functions
 ******************************************/

inline const StringIdentifier getStringId(Node* node, const StringIdentifier& idName, const StringIdentifier& strName) {
	Node* proto = node->isInstance() ? node->getPrototype() : node; // meshIds or surfelIds are only stored in prototypes
	StringIdentifier id;
	if(!node->findAttribute(idName)) {
		WARN("Node has no attribute: " + idName.toString());
	} if(!node->findAttribute(strName)) {
		id = node->findAttribute(idName)->toString();
		proto->setAttribute(strName, new StringIDAttribute_t(id));
	} else {
		const StringIDAttribute_t * objContainer = dynamic_cast<const StringIDAttribute_t *> (node->findAttribute(strName));
		if(objContainer == nullptr) {
			WARN("Wrong attribute type for " + strName.toString());
		} else {
			id = objContainer->get();
		}
	}
	return id;
}

inline const StringIdentifier assureStringId(Node* node, const StringIdentifier& idName, const StringIdentifier& strName) {
	if(node->isInstance()) node = node->getPrototype(); // meshIds or surfelIds are only stored in prototypes
	if(!node->isAttributeSet(idName))
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(StringUtils::createRandomString(32)));
	return getStringId(node, idName, strName);
}

/**
 * Find the id of the next parent that contains surfels
 */
inline const StringIdentifier findParentId(Node* node) {
	Node* parent = node->getParent();
	while(parent != nullptr) {
		if(parent->isAttributeSet(SURFEL_ID)) // parents can only have surfels
			return getStringId(parent, SURFEL_ID, SURFEL_STRINGID);
		parent = parent->getParent();
	}
	return NO_PARENT;
}

inline FileName buildMeshFilename(const FileName& basePath, const std::string& subpath, const StringIdentifier& id) {
	FileName file(basePath.toString() + subpath);
	if(!FileUtils::isDir(file))
		FileUtils::createDir(file, true);
	file.setFile(id.toString());
	file.setEnding("mmf");
	return file;
}

/******************************************
 * CacheObject class
 ******************************************/
class CacheObject {
public:
	enum State_t {
		Free, // Object is free an can be released
		Loaded, // Object is in cache
		Loading, // Object is currently loaded from file system
		Pending // Object is requested but not loaded yet
	};
	typedef std::unique_ptr<CacheObject> Ptr_t;
	static std::deque<Ptr_t> cacheObjectPool;

	StringIdentifier id;
	uint32_t lru = 0;
	uint32_t usage = 0;
	uint32_t memory = 0;
	uint32_t childCount = 0;
	float projSize = 0;
	float minDistance = 0;
	bool used = false;
	bool isSurfel = false;
	Reference<Mesh> mesh;
	StringIdentifier parentId = NO_PARENT;

	std::atomic<State_t> state;
public:
	CacheObject(const StringIdentifier& id) :  id(id), state(Free) {}

	inline bool isFree() const { return state == Free; }
	inline bool isPending() const { return state == Pending; }
	inline bool isLoading() const { return state == Loading; }
	inline bool isLoaded() const { return state == Loaded; }

	inline void release() { releaseCacheObject(this); }

	static CacheObject* createCacheObject(const Util::StringIdentifier& id) {
		if(cacheObjectPool.empty())
			return new CacheObject(id);
		CacheObject* object = cacheObjectPool.back().release();
		cacheObjectPool.pop_back();
		object->id = id;
		object->state = CacheObject::Free;
		return object;
	}

	static void releaseCacheObject(CacheObject* object) {
		if(object == nullptr)
			return;
		//std::cout << "releasing cache object " << object->id.toString() << std::endl;
		//TODO: check for max. pool size
		cacheObjectPool.push_back(Ptr_t(object));
		object->state = CacheObject::Free;
		object->lru = 0;
		object->usage = 0;
		object->memory = 0;
		object->mesh = nullptr;
	}
};

std::deque<CacheObject::Ptr_t> CacheObject::cacheObjectPool;

/******************************************
 * MainCache class
 ******************************************/
class MainCache {
private:
	typedef std::forward_list<CacheObject*> MainCacheList_t;
	//typedef std::list<CacheObject*> MainCacheList_t;
	MainCacheList_t list;
	MainCacheList_t::iterator evictIt;
	std::unique_ptr<Mutex> mutex;
	uint32_t cacheSize = 0;
public:
	MainCache() : mutex(Concurrency::createMutex()) {
		evictIt = list.end();
	}

	void put(CacheObject* obj) {
		LOCK(*mutex);
		if(obj != nullptr) {
			list.push_front(obj);
			++cacheSize;
		}
	};
	CacheObject* evict() {
		LOCK(*mutex);
		if(list.empty()) {
			return nullptr;
		}
		if(evictIt == list.end())
			evictIt = list.before_begin();
		auto it = evictIt;
		++evictIt;
		if(evictIt != list.end()) {
			auto co = *evictIt;
			if(co->used) {
				co->used = false;
			} else if(co->childCount <= 0) {
				++evictIt;
				list.erase_after(it);
				--cacheSize;
				return co;
			}
		}
		return nullptr;
	}
	bool empty() const {
		LOCK(*mutex);
		return list.empty();
	}
	void clear() {
		LOCK(*mutex);
		list.clear();
		evictIt = list.end();
		cacheSize = 0;
	}
	uint32_t size() const {
		return cacheSize;
	}
};

/******************************************
 * Request queue class
 ******************************************/
class RequestQueue {
private:
	typedef std::deque<CacheObject*> InternalQueue_t;
	std::array<InternalQueue_t, REQUEST_QUEUE_SIZE> queues;
	std::unique_ptr<Mutex> mutex;

	float lastFramePriorityMin = 0;
	float lastFramePriorityMax = 0;
	float currentFramePriorityMin = std::numeric_limits<float>::max();
	float currentFramePriorityMax = 0;
public:
	bool sortRequests = false;

	SurfelManager::RequestPriorityFn_t priorityFn;

	RequestQueue() : mutex(Concurrency::createMutex()) {
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i)
			queues[i] = InternalQueue_t();
		priorityFn = [] (StringIdentifier id, uint32_t lru, uint32_t usage,
				uint32_t memory, uint32_t childCount, float projSize,
				float minDistance, bool isSurfel)  {
			return projSize;
		};
	}

	inline float getPriority(const CacheObject* obj) {
		if(obj == nullptr) return 0;
		return std::sqrt(obj->projSize) / std::max(obj->minDistance,0.1f);
		//return priorityFn(obj->id, obj->lru, obj->usage, obj->memory, obj->childCount, obj->projSize, obj->minDistance, obj->isSurfel);
	}

	inline uint32_t getQueueIndexFromPriority(CacheObject* obj) {
		float priority = getPriority(obj);
		currentFramePriorityMin = std::min(currentFramePriorityMin, priority);
		currentFramePriorityMax = std::max(currentFramePriorityMax, priority);
		priority = clamp<float>(priority - lastFramePriorityMin, 0, lastFramePriorityMax- lastFramePriorityMin) ;
		float range = lastFramePriorityMax-lastFramePriorityMin;
		return range > 0 ? std::floor((1.0f - priority/range)*(REQUEST_QUEUE_SIZE-1)) : 0;
	}

	void push(CacheObject* obj) {
		LOCK(*mutex);
		uint32_t p = clamp<uint32_t>(getQueueIndexFromPriority(obj), 0, REQUEST_QUEUE_SIZE-1);
		queues[p].push_back(obj);
	};

	CacheObject* popFront() {
		LOCK(*mutex);
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
			if(!queues[i].empty()) {
				auto co = queues[i].front();
				queues[i].pop_front();
				return co;
			}
		}
		return nullptr;
	};

	CacheObject* popBack() {
		LOCK(*mutex);
		for(uint32_t i=REQUEST_QUEUE_SIZE-1; i>=0; --i) {
			if(!queues[i].empty()) {
				auto co = queues[i].back();
				queues[i].pop_back();
				return co;
			}
		}
		return nullptr;
	};

	CacheObject* first() const {
		LOCK(*mutex);
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
			if(!queues[i].empty()) {
				auto co = queues[i].front();
				return co;
			}
		}
		return nullptr;
	}

	CacheObject* last() const {
		LOCK(*mutex);
		for(uint32_t i=REQUEST_QUEUE_SIZE-1; i>=0; --i) {
			if(!queues[i].empty()) {
				auto co = queues[i].back();
				return co;
			}
		}
		return nullptr;
	}

	bool empty() const {
		LOCK(*mutex);
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
			if(!queues[i].empty()) {
				return false;
			}
		}
		return true;
	}

	void clear() {
		LOCK(*mutex);
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
			queues[i].clear();
		}
		lastFramePriorityMax = 1;
		lastFramePriorityMin = 0;
		currentFramePriorityMin = std::numeric_limits<float>::max();
		currentFramePriorityMax = 0;
	}

	size_t size() const {
		LOCK(*mutex);
		size_t c=0;
		for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
			c += queues[i].size();
		}
		return c;
	}

	size_t size(uint32_t p) const {
		LOCK(*mutex);
		return queues[clamp<uint32_t>(p, 0, REQUEST_QUEUE_SIZE-1)].size();
	}

	void update() {
		LOCK(*mutex);
		if(currentFramePriorityMax > currentFramePriorityMin) {
			lastFramePriorityMax = currentFramePriorityMax;
			lastFramePriorityMin = currentFramePriorityMin;
		}
		currentFramePriorityMin = std::numeric_limits<float>::max();
		currentFramePriorityMax = 0;

		if(sortRequests) {
			for(uint32_t i=0; i<REQUEST_QUEUE_SIZE; ++i) {
				if(!queues[i].empty()) {
					std::sort(queues[i].begin(), queues[i].end(), [this](const CacheObject* oc1, const CacheObject* oc2) {
						return getPriority(oc1) < getPriority(oc2);
					});
					break;
				}
			}
		}
	}

};

/******************************************
 * Implementation class
 ******************************************/

class SurfelManager::Implementation {
public:

	Implementation(SurfelManager* m, Preprocessor* p, const FileName& basePath, uint64_t maxMemory);
	~Implementation() = default;
public:
	// "public" fields
	WeakPointer<SurfelManager> manager;
	Reference<Preprocessor> preprocessor;
	FileName basePath;
	uint64_t maxMemory;
	uint64_t requestLimit = 10485760; // 10 MB
	uint64_t frameRequestLimit = 2097152; // 2 MB
	uint32_t maxPending = 1000;
	float memoryLoadFactor = 0.8;
	uint32_t maxIter = 100;
	uint32_t maxFrameTime = 30;

	std::unique_ptr<GenericAttributeMap> stats;
public:
	void init();
	void update();
	void clear();
	void flush();

	SurfelManager::MeshLoadResult_t doFetch(const StringIdentifier& id, const StringIdentifier& parentId, bool isSurfel, Util::Reference<Rendering::Mesh>& out, float projSize, float distance, bool async, bool force);
	bool isCached(const StringIdentifier& id) const;
	SurfelManager::MeshLoadResult_t doLoadMesh(CacheObject* object, bool async);
	void doStoreMesh(const StringIdentifier& id, const FileName& filename, Mesh* mesh, bool async);

	void executeAsync(std::function<void()> function) { jobQueue.push(function); };
	void executeOnMainThread(std::function<void()> function) { mainThreadQueue.push(function); };

	void release(CacheObject* obj);
public:

	SyncQueue<Job_t> jobQueue;
	SyncQueue<Job_t> mainThreadQueue;
	std::vector<std::unique_ptr<WorkerThread>> threadPool;

	// stores all currently used cached objects
	std::unordered_map<StringIdentifier,CacheObject*> idToCacheObject;
	// stores cache objects that can not be released without further preconditions (e.g. child nodes are loaded)
	//std::unordered_map<StringIdentifier,CacheObject*> lockedCache;

	RequestQueue requests;
	MainCache cache;

	//stats
	uint32_t frameNumber = 0;
	uint32_t loadingRequests = 0;
	uint32_t cacheMisses = 0;
	uint32_t cacheHits = 0;
	std::atomic<uint64_t> currentRequestSize;
	uint64_t currentMemorySize = 0;
	double accumLoadTime = 0;
	uint64_t accumLoadNumber = 0;
};

SurfelManager::Implementation::Implementation(SurfelManager* m, Preprocessor* p, const FileName& basePath, uint64_t maxMemory) :
		manager(m), preprocessor(p), basePath(basePath), maxMemory(maxMemory), currentRequestSize(0),
		stats(new GenericAttributeMap) {
}

void SurfelManager::Implementation::init() {
	for(uint32_t i=0; i<THREAD_COUNT; ++i)
		threadPool.push_back(std::unique_ptr<WorkerThread>(new WorkerThread(manager.get())));
	// FIXME: might be faster to directly use std::malloc for a consecutive memory block
	for(uint_fast32_t i = 0; i < INITIAL_POOL_SIZE; ++i) {
		CacheObject::cacheObjectPool.push_back(CacheObject::Ptr_t(new CacheObject(NO_PARENT)));
	}
}
void SurfelManager::Implementation::update() {
	static Timer updateTimer;
	updateTimer.reset();
	++frameNumber;

	LOG_STAT(requestQueue, requests.size());
	LOG_STAT(requestQueue0, requests.size(0));
	LOG_STAT(requestQueue1, requests.size(1));
	LOG_STAT(requestQueue2, requests.size(2));
	LOG_STAT(requestQueue3, requests.size(3));
	LOG_STAT(requestQueue4, requests.size(4));
	LOG_STAT(requestQueue5, requests.size(5));
	LOG_STAT(requestQueue6, requests.size(6));
	LOG_STAT(requestQueue7, requests.size(7));
	LOG_STAT(cacheObjects, idToCacheObject.size());
	LOG_STAT(accumLoadTime, accumLoadTime);
	LOG_STAT(accumLoadNumber, accumLoadNumber);
	LOG_STAT(hits, cacheHits);
	LOG_STAT(misses, cacheMisses);
	LOG_STAT(usedMemory, currentMemorySize/(1024.0f*1024.0f));
	LOG_STAT(requestSize, currentRequestSize/(1024.0f*1024.0f));
	LOG_STAT(loadRequests, loadingRequests);
	cacheHits = 0;
	cacheMisses = 0;

	for(uint32_t i=0; i<maxIter && !mainThreadQueue.empty() && updateTimer.getMilliseconds() <= maxFrameTime; ++i) {
		// never blocks since this is the only thread that removes jobs from this queue
		mainThreadQueue.pop()();
	}

	//if(currentRequestSize < requestLimit)
	requests.update();
	uint64_t frameRequests = 0;
	while(currentRequestSize < requestLimit && frameRequests < frameRequestLimit
			&& currentMemorySize < maxMemory && loadingRequests < maxPending && !requests.empty() && updateTimer.getMilliseconds() < maxFrameTime) {
		CacheObject* object = requests.popFront();
		uint64_t tmp = currentRequestSize;
		if(doLoadMesh(object, true) == Failed)
			release(object);
		frameRequests += currentRequestSize > tmp ? currentRequestSize-tmp : 0;
	}

	if(currentMemorySize > maxMemory * memoryLoadFactor) {
		uint32_t i = 0;
		while(updateTimer.getMilliseconds() < maxFrameTime
				&& (currentMemorySize > maxMemory
						|| (currentMemorySize > maxMemory * memoryLoadFactor
								&& ++i <= std::min(cache.size(), maxIter)))) {
			CacheObject* obj = cache.evict();
			if(obj != nullptr) {
				currentMemorySize -= obj->memory;
				if(obj->parentId != NO_PARENT) {
					auto pIt = idToCacheObject.find(obj->parentId);
					if(pIt != idToCacheObject.end()) {
						pIt->second->childCount--;
					}
				}
				release(obj);
			}
		}
	}
	LOG_STAT(updateTime, updateTimer.getMilliseconds());
}

void SurfelManager::Implementation::clear() {
	flush();
	requests.clear();
	cache.clear();
	for(auto entry : idToCacheObject) {
		entry.second->release();
	}
	idToCacheObject.clear();

	frameNumber = 0;
	loadingRequests = 0;
	cacheMisses = 0;
	cacheHits = 0;
	currentRequestSize = 0;
	currentMemorySize = 0;
	accumLoadTime = 0;
	accumLoadNumber = 0;
}

void SurfelManager::Implementation::flush() {
	while(!mainThreadQueue.empty()) {
		mainThreadQueue.pop()();
	}

	while(!jobQueue.empty()) {
		Utils::sleep(1); // Wait until worker threads finish all work
		// don't want to do it here because it might block
	}
}

SurfelManager::MeshLoadResult_t SurfelManager::Implementation::doFetch(const StringIdentifier& id, const StringIdentifier& parentId, bool isSurfel, Util::Reference<Rendering::Mesh>& out, float projSize, float distance, bool async, bool force) {
	auto it = idToCacheObject.find(id);
	CacheObject* object = nullptr;
	if(it != idToCacheObject.end()) {
		object = it->second;
	} else if (currentMemorySize >= maxMemory) {
		++cacheMisses;
		return Failed;
	} else {
		if(requests.size() > maxPending) {
			++cacheMisses;
			release(requests.popBack());
			return Failed;
		}
		object = CacheObject::createCacheObject(id);
		idToCacheObject[id] = object;
	}

	// update cache object
	if(object->lru == frameNumber) {
		++object->usage;
	} else {
		object->usage = 1;
	}
	object->projSize = projSize;
	object->minDistance = distance;
	object->parentId = parentId;
	object->used = true;
	object->isSurfel = isSurfel;

	if(object->isLoaded()) {
		// Cache hit
		out = object->mesh;
		++cacheHits;
		return Success;
	} else if(object->isFree()) {
		object->state = CacheObject::Pending;
		requests.push(object);
	}
	++cacheMisses;

	// TODO: force loading
	return Pending;
}

bool SurfelManager::Implementation::isCached(const StringIdentifier& id) const {
	auto it = idToCacheObject.find(id);
	if(it != idToCacheObject.end()) {
		return it->second->isLoaded();
	}
	return false;
}

SurfelManager::MeshLoadResult_t SurfelManager::Implementation::doLoadMesh(CacheObject* object, bool async) {
	if(!object || object->isFree()) return Failed;
	if(!object->isPending()) return Pending;

	const FileName& filename = buildMeshFilename(basePath, object->isSurfel ? "surfels/" : "meshes/", object->id);
	if(!FileUtils::isFile(filename)) {
		WARN("File '" + filename.toString() + "' does not exist.");
		return Failed;
	}

	if(object->parentId != NO_PARENT) {
		auto pIt = idToCacheObject.find(object->parentId);
		if(pIt != idToCacheObject.end()) {
			pIt->second->childCount++;
			pIt->second->used = true;
		} else {
			//WARN("Parent of '" + object->id.toString() + "' is not in cache.");
			return Failed;
		}
	}

	// estimate memory usage
	uint64_t fileSize = FileUtils::fileSize(filename);
	object->memory = fileSize; // only estimates the real memory consumption
	currentRequestSize += fileSize;

	++loadingRequests;
	Util::Timer timer;
	std::function<void()> readFn = [this, object, filename, timer] () {
		Reference<Mesh> mesh = Serialization::loadMesh(filename);
		if(mesh.isNull()) {
			WARN("Could not load mesh: " + filename.toString());
			object->state = CacheObject::Pending;

			executeOnMainThread([this, object]() {
				if(object->parentId != NO_PARENT) {
					auto pIt = idToCacheObject.find(object->parentId);
					if(pIt != idToCacheObject.end()) {
						pIt->second->childCount--;
					}
				}
			});
			return;
		}

		object->mesh = mesh;
		object->state = CacheObject::Loaded;
		currentRequestSize -= object->memory;
		executeOnMainThread([this, object, mesh, timer]() {
			//object->state = CacheObject::Loaded;
			//object->mesh = mesh;
			//currentRequestSize -= object->memory;
			object->memory = mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
			currentMemorySize += object->memory;
			cache.put(object);
			--loadingRequests;
			++accumLoadNumber;
			accumLoadTime += timer.getMilliseconds();
		});
	};
	object->state = CacheObject::Loading;

	if(async) {
		executeAsync(readFn);
	} else {
		readFn();
		return Success;
	}
	return Pending;
}

void SurfelManager::Implementation::doStoreMesh(const StringIdentifier& id, const FileName& filename, Mesh* mesh, bool async) {
	auto it = idToCacheObject.find(id);
	if(it != idToCacheObject.end()) {
		// FIXME: What happens when the object is currently loading?
		it->second->state = CacheObject::Loaded;
		it->second->mesh = mesh;
		it->second->memory = mesh->getGraphicsMemoryUsage() + mesh->getMainMemoryUsage();
		currentMemorySize += it->second->memory;
		cache.put(it->second);
	}

	std::function<void()> writeFn = [mesh, filename] () {
		std::cout << "saving mesh " << filename.toString() << " ...";
		Serialization::saveMesh(mesh, filename);
		std::cout << "done" << std::endl;
		FileUtils::flush(filename);
	};

	if(async) {
		executeAsync(writeFn);
	} else {
		writeFn();
	}
}

void SurfelManager::Implementation::release(CacheObject* object) {
	if(object == nullptr) return;
	idToCacheObject.erase(object->id);
	CacheObject::releaseCacheObject(object);
}

/******************************************
 * WorkerThread Implementation
 ******************************************/

void WorkerThread::close() {
	if(closed)
		return;
	closed = true;
	closing = true;
	while(closing) {
		manager->impl->executeAsync([](){}); // add empty job to release lock on job queue
		Utils::sleep(1);
	}
	join();
}

void WorkerThread::run() {
	while(!closed) {
		manager->impl->jobQueue.pop()();
	}
	closing = false;
}

/******************************************
 * SurfelManager class
 ******************************************/

SurfelManager::SurfelManager(const Util::FileName& basePath, uint64_t maxMemory, uint64_t maxReservedMemory) : impl(new Implementation(this, new Preprocessor(this), basePath, maxMemory)) {
	impl->init();
}

SurfelManager::~SurfelManager() {
	CacheObject::cacheObjectPool.clear();
	CacheObject::cacheObjectPool.shrink_to_fit();
}


void SurfelManager::storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async) {
	const StringIdentifier id = assureStringId(node, SURFEL_ID, SURFEL_STRINGID);
	node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
	impl->doStoreMesh(id, buildMeshFilename(impl->basePath, "surfels/", id), surfelInfo.first.get(), async);
}

SurfelManager::MeshLoadResult_t SurfelManager::fetchSurfel(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async, bool force) {
	if(!node->findAttribute(SURFEL_ID)) return Failed;
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	return impl->doFetch(id, findParentId(node), true, out, projSize, distance, async, force);
}

void SurfelManager::storeMesh(GeometryNode* node, bool async) {
	const StringIdentifier id = assureStringId(node, MESH_ID, MESH_STRINGID);
	impl->doStoreMesh(id, buildMeshFilename(impl->basePath, "meshes/", id), node->getMesh(), async);
}

SurfelManager::MeshLoadResult_t SurfelManager::fetchMesh(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async, bool force) {
	if(!node->findAttribute(MESH_ID)) return Failed;
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	return impl->doFetch(id, findParentId(node), false, out, projSize, distance, async, force);
}

bool SurfelManager::isCached(Node* node) {
	if(node->isInstance()) node = node->getPrototype(); // mesheIds or surfelIds are only stored in prototypes
	if(!node->isAttributeSet(SURFEL_ID) && !node->isAttributeSet(MESH_ID)) return true; // return true because there might be surfels in the subtree
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
	if(geometry != nullptr && geometry->getMesh()->getVertexCount() > 0) return true;
	if(node->isAttributeSet(MESH_ID) && impl->isCached(getStringId(node, MESH_ID, MESH_STRINGID))) return true;
	if(node->isAttributeSet(SURFEL_ID) && impl->isCached(getStringId(node, SURFEL_ID, SURFEL_STRINGID))) return true;
	return false;
}

void SurfelManager::update() { impl->update(); }
void SurfelManager::clear() { impl->clear(); }
void SurfelManager::flush() { impl->flush(); }
void SurfelManager::executeAsync(const std::function<void()>& function) { impl->executeAsync(function); }
void SurfelManager::executeOnMainThread(const std::function<void()>& function) {impl->executeOnMainThread(function); }

Preprocessor* SurfelManager::getPreprocessor() const { return impl->preprocessor.get(); }
const Util::FileName SurfelManager::getBasePath() const { return impl->basePath; }
void SurfelManager::setBasePath(const std::string& filename) { impl->basePath = FileName(filename); }
void SurfelManager::setBasePath(const Util::FileName& filename) { impl->basePath = filename; }
void SurfelManager::setMaxMemory(uint64_t value) { impl->maxMemory = value; }
uint64_t SurfelManager::getMaxMemory() const { return impl->maxMemory;}
Util::GenericAttributeMap* SurfelManager::getStats() const { return impl->stats.get(); }
void SurfelManager::setMemoryLoadFactor(float value) { impl->memoryLoadFactor = value; }
float SurfelManager::getMemoryLoadFactor() const { return impl->memoryLoadFactor; }
void SurfelManager::setRequestLimit(uint64_t value) { impl->requestLimit = value; }
void SurfelManager::setFrameRequestLimit(uint64_t value) { impl->frameRequestLimit = value; }
void SurfelManager::setMaxPending(uint32_t value) { impl->maxPending = value; };
void SurfelManager::setSortRequests(bool value) { impl->requests.sortRequests = value; }
void SurfelManager::setMaxIter(uint32_t value) { impl->maxIter = value; };
void SurfelManager::setMaxFrameTime(uint32_t value) { impl->maxFrameTime = value; };
void SurfelManager::setRequestPriorityFn(const SurfelManager::RequestPriorityFn_t& fn) { impl->requests.priorityFn = fn; }

} /* namespace ThesisSascha */
} /* namespace MinSG */


#endif /* MINSG_EXT_THESISSASCHA */
