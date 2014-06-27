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

#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/Nodes/ListNode.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Serialization/Serialization.h>

#include <Util/Utils.h>
#include <Util/GenericAttribute.h>
#include <Util/GenericAttributeSerialization.h>
#include <Util/StringUtils.h>
#include <Util/Concurrency/Concurrency.h>
#include <Util/Concurrency/UserThread.h>
#include <Util/Concurrency/DataStructures/SyncQueue.h>
#include <Util/Concurrency/Mutex.h>
#include <Util/Concurrency/Lock.h>
#include <Util/IO/FileUtils.h>
#include <Util/Timer.h>

#include <iostream>
#include <cstdlib>
#include <atomic>
#include <iterator>

#define LOCK(mutex) auto lock = Concurrency::createLock(mutex);
#define LOG_STAT(name, value) static StringIdentifier sId ## name(#name); \
	stats->setValue(sId ## name, GenericAttribute::createNumber(value));

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;
using namespace Util::Concurrency;

/******************************************
 * helper functions
 ******************************************/

inline const StringIdentifier getStringId(Node* node, const StringIdentifier& idName, const StringIdentifier& strName) {
	StringIdentifier id;
	if(!node->isAttributeSet(strName)) {
		id = node->getAttribute(idName)->toString();
		node->setAttribute(strName, new StringIDAttribute_t(id));
	} else {
		const StringIDAttribute_t * objContainer = dynamic_cast<const StringIDAttribute_t *> (node->getAttribute(strName));
		if(objContainer == nullptr) {
			WARN("Wrong attribute type for " + strName.toString());
		} else {
			id = objContainer->get();
		}
	}
	return id;
}

std::pair<std::string, std::string> serializeID(const std::pair<const Util::GenericAttribute *, const Util::GenericAttributeMap *> & attributeAndContext) {
	auto idAttribute = dynamic_cast<const StringIDAttribute_t *>(attributeAndContext.first);
	return std::make_pair(GATypeNameStringIdentifier, GAStringIdentifierHeader + idAttribute->get().toString());
}

StringIDAttribute_t * unserializeID(const std::pair<std::string, const Util::GenericAttributeMap *> & contentAndContext) {
	const std::string & s = contentAndContext.first;
	if(!StringUtils::beginsWith(s.c_str(), GAStringIdentifierHeader.c_str()))
		return nullptr;
	return new StringIDAttribute_t(StringIdentifier(s.substr(GAStringIdentifierHeader.length())));
}

inline FileName buildSurfelFilename(const FileName& basePath, const StringIdentifier& id) {
	FileName file(basePath.toString() + "surfels/");
	if(!FileUtils::isDir(file))
		FileUtils::createDir(file, true);
	file.setFile(id.toString());
	file.setEnding("mmf");
	return file;
}

inline FileName buildMeshFilename(const FileName& basePath, const StringIdentifier& id) {
	FileName file(basePath.toString() + "meshes/");
	if(!FileUtils::isDir(file))
		FileUtils::createDir(file, true);
	file.setFile(id.toString());
	file.setEnding("mmf");
	return file;
}

/******************************************
 * CacheObject class
 ******************************************/

class SurfelManager::CacheObject {
public:
	enum Priority {
		LRU, Usage, Distance, ProjSize, LevelTB, LevelBT, Memory
	};

	typedef std::function<bool(const CacheObject & a, const CacheObject & b)> CompareFn_t;
	static std::vector<CompareFn_t> priorityFunctions;
	static std::vector<uint32_t> priorityOrder;
	//static std::unique_ptr<Concurrency::Mutex> objectMutex;
	std::unique_ptr<Concurrency::Mutex> localMutex;

	// Making everything atomic because i'm paranoid
	//std::atomic<StringIdentifier> id;
	StringIdentifier id;
	std::atomic<uint32_t> lru;
	std::atomic<uint16_t> usage;
	std::atomic<uint16_t> level;
	std::atomic<uint32_t> memory;
	std::atomic<float> projSize;
	std::atomic<float> minDistance;
	//std::atomic<Reference<Mesh>> mesh; // can Util::Reference be used with atomic?
	Reference<Mesh> mesh;
	enum State_t {
		Empty, Pending, Loading, Loaded, Abort
	};
	std::atomic<State_t> state;
	std::atomic<bool> surfel;
public:
	CacheObject() : localMutex(Concurrency::createMutex()), id(), lru(0), usage(0), level(0), memory(0), projSize(0), minDistance(0), mesh(), state(Empty), surfel(false) {}
	CacheObject(const StringIdentifier& id) : localMutex(Concurrency::createMutex()), id(id), lru(0), usage(0), level(0), projSize(0), minDistance(0), memory(0), mesh(), state(Empty), surfel(false) {}

	bool operator < (const CacheObject & other) const {
		for(uint32_t p : priorityOrder) {
			CompareFn_t fn = priorityFunctions[p];
			if(fn(*this,other)) {
				return true;
			} else if(fn(other,*this)) {
				return false;
			}
		}
		return false;
	}
	bool operator == (const CacheObject & other) const {
		return level == other.level
				&& lru == other.lru
				&& usage == other.usage
				&& state == other.state
				&& minDistance == other.minDistance
				&& projSize == other.projSize; // should not be too much of a problem for sorting
	}

	inline bool isEmpty() const { return state == Empty; }
	inline bool isPending() const { return state == Pending; }
	inline bool isLoading() const { return state == Loading; }
	inline bool isLoaded() const { return state == Loaded; }
	inline bool isAborted() const { return state == Abort; }
	inline bool isSurfel() const { return surfel; }

	inline const StringIdentifier& getId() const { return id; }
	inline void setId(const StringIdentifier& id_) { id = id_; }
	inline Mesh* getMesh() const { LOCK(*localMutex); return mesh.get(); }
	inline void setMesh(Mesh* m) { LOCK(*localMutex); mesh = m; }

	/*void swap(CacheObject* obj) {
		std::swap(this->id, obj->id);
		std::swap(this->lru, obj->lru);
		std::swap(this->usage, obj->usage);
		std::swap(this->level, obj->level);
		std::swap(this->projSize, obj->projSize);
		std::swap(this->minDistance, obj->minDistance);
		std::swap(this->state, obj->state);
		std::swap(this->mesh, obj->mesh);
		std::swap(this->memory, obj->memory);
		std::swap(this->surfel, obj->surfel);
	}*/

};

std::vector<SurfelManager::CacheObject::CompareFn_t> SurfelManager::CacheObject::priorityFunctions {
	[](const CacheObject & a, const CacheObject & b) { return a.lru < b.lru; },
	[](const CacheObject & a, const CacheObject & b) { return a.usage < b.usage; },
	[](const CacheObject & a, const CacheObject & b) { return a.minDistance > b.minDistance; },
	[](const CacheObject & a, const CacheObject & b) { return a.projSize < b.projSize; },
	[](const CacheObject & a, const CacheObject & b) { return a.level > b.level; },
	[](const CacheObject & a, const CacheObject & b) { return a.level < b.level; },
	[](const CacheObject & a, const CacheObject & b) { return a.memory < b.memory; }
};

std::vector<uint32_t> SurfelManager::CacheObject::priorityOrder {
	CacheObject::LRU, CacheObject::Distance, CacheObject::ProjSize, CacheObject::Usage
};

//std::unique_ptr<Concurrency::Mutex> SurfelManager::CacheObject::objectMutex(Concurrency::createMutex());

template<class Type>
struct ObjectCompare {

	bool operator()(const Type * a, const Type * b) const {
		if(a == nullptr || b == nullptr) {
			WARN("Null cache object");
			return true;
		}
		return *b < *a || (!(*a < *b) && b < a);
	}
};

template<typename T>
T atomic_fetch_add(std::atomic<T> *obj, T arg) {
  T expected = obj->load();
  while(!std::atomic_compare_exchange_weak(obj, &expected, expected + arg))
    ;
  return expected;
}

/******************************************
 * WorkerThread class
 ******************************************/

class SurfelManager::WorkerThread : public UserThread {
public:
	struct Job_t {
		std::function<void()> function;
	};
	struct MeshSwap_t {
		StringIdentifier id;
		Reference<Mesh> target;
		Reference<Mesh> source;
	};

	WorkerThread(SurfelManager* manager) : closed(false), manager(manager), mutex(Concurrency::createMutex()),
			accumCacheTime(0), accumCacheUpdates(0), cacheMisses(0), cacheHits(0)
			{ start(); };
	virtual ~WorkerThread() = default;
	void close();
	void executeAsync(const std::function<void()>& function);
	void flush(uint32_t timeLimit = MAX_FLUSH_TIME);
protected:
	void run() override;

	SyncQueue<Job_t> jobQueue;
	bool closed;
	WeakPointer<SurfelManager> manager;
	std::unique_ptr<Concurrency::Mutex> mutex;
public:
	SyncQueue<std::function<void()>> mainThreadQueue;

	// misuse WorkerThread for pimpl idiome
	std::atomic<double> accumCacheTime;
	std::atomic<uint64_t> accumCacheUpdates;
	std::atomic<uint32_t> cacheMisses;
	std::atomic<uint32_t> cacheHits;
};

void SurfelManager::WorkerThread::close() {
	if(closed)
		return;
	//closed = true;
	jobQueue.push({[this]() { this->closed = true; }});
}

void SurfelManager::WorkerThread::run() {
	while(!closed) {
		Job_t job;
		job = jobQueue.pop();
		job.function();
	}
}

void SurfelManager::WorkerThread::executeAsync(const std::function<void()>& function) {
	if(closed)
		return;
	Job_t job;
	job.function = function;
	jobQueue.push(job);
	if(jobQueue.size() > manager->getMaxJobs())
		flush(manager->maxJobFlushTime);
}

void SurfelManager::WorkerThread::flush(uint32_t timeLimit) {
	static Timer timer;
	if(closed)
		return;
	timer.reset();
	while(!closed && !jobQueue.empty() && (timeLimit == 0 || timer.getMilliseconds() < timeLimit)) {
		Utils::sleep(1);
	}
	std::cout << "flushed jobs. (" << timer.getMilliseconds() << "ms)" << std::endl;
}

/******************************************
 * SurfelManager class
 ******************************************/

SurfelManager::SurfelManager(const Util::FileName& basePath, uint64_t maxMemory, uint64_t maxReservedMemory) : basePath(basePath), worker(new WorkerThread(this)),
		preprocessor(new Preprocessor(this)), maxMemory(maxMemory), usedMemory(0), reservedMemory(0), maxReservedMemory(maxReservedMemory),
		frameNumber(0), maxJobNumber(MAX_JOB_NUMBER), maxJobFlushTime(30), memoryLoadFactor(0.8f), pending(0), maxPerFrameRequestMem(10*1024*1024),
		maxPending(MAX_PENDING_OBJECTS), stats(new GenericAttributeMap) {
	GenericAttributeSerialization::registerSerializer<WrapperAttribute<StringIdentifier>>(GATypeNameStringIdentifier, serializeID, unserializeID);
	// FIXME: might be faster to directly use std::malloc for a consecutive memory block
	for(uint_fast32_t i = 0; i < INITIAL_POOL_SIZE; ++i) {
		cacheObjectPool.push_back(new CacheObject());
	}
}

SurfelManager::~SurfelManager() {
	worker->close();
	worker->join();
	update();
	delete worker;
}

void SurfelManager::storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID)) {
		StringIdentifier id(StringUtils::createRandomString(32));
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(id.toString()));
		node->setAttribute(SURFEL_STRINGID, new StringIDAttribute_t(id));
	}
	node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	doStoreMesh(id, buildSurfelFilename(basePath, id), surfelInfo.first.get(), async);
}

SurfelManager::MeshLoadResult_t SurfelManager::fetchSurfel(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async, bool force) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	if(idToCacheObject.count(id) > 0)
		out = idToCacheObject[id]->isLoaded() ? idToCacheObject[id]->getMesh() : nullptr;
	if(out != nullptr)
		node->setAttribute(SURFEL_COUNT, GenericAttribute::createNumber(out->isUsingIndexData() ? out->getIndexCount() : out->getVertexCount()));
	auto result = doFetchMesh(id, true, level, projSize, distance, async);
	if(force) {
		if(idToCacheObject.count(id) <= 0)
			return Failed;
		if(doLoadMesh(idToCacheObject[id], false) != Success)
			return Failed;
		out = idToCacheObject[id]->isLoaded() ? idToCacheObject[id]->getMesh() : nullptr;
		return out == nullptr ? Failed : Success;
	}
	return result;
}

bool SurfelManager::isCached(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID) && !node->isAttributeSet(MESH_ID))
		return true; // return true because there might be surfels in the subtree
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
	if(geometry != nullptr && geometry->getMesh()->getVertexCount() > 0) {
		return true;
	}
	if(node->isAttributeSet(MESH_ID)) {
		const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
		auto it = idToCacheObject.find(id);
		if(it != idToCacheObject.end()) {
			if(it->second->isLoaded() || it->second->isLoading())
				return true;
		}
	}
	if(node->isAttributeSet(SURFEL_ID)) {
		const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
		auto it = idToCacheObject.find(id);
		if(it != idToCacheObject.end()) {
			if(it->second->isLoaded() || it->second->isLoading())
				return true;
		}
	}
	return false;
}

void SurfelManager::storeMesh(GeometryNode* node, bool async) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null
	if(!node->isAttributeSet(MESH_ID)) {
		StringIdentifier id(StringUtils::createRandomString(32));
		node->setAttribute(MESH_ID, GenericAttribute::createString(id.toString()));
	}
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	doStoreMesh(id, buildMeshFilename(basePath, id), node->getMesh(), async);
}

SurfelManager::MeshLoadResult_t SurfelManager::fetchMesh(Util::Reference<Rendering::Mesh>& out, GeometryNode* node, float projSize, float distance, bool async, bool force) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null
	if(!node->isAttributeSet(MESH_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	if(idToCacheObject.count(id) > 0)
		out = idToCacheObject[id]->isLoaded() ? idToCacheObject[id]->getMesh() : nullptr;
	auto result = doFetchMesh(id, false, level, projSize, distance, async);
	if(force) {
		if(idToCacheObject.count(id) <= 0)
			return Failed;
		if(doLoadMesh(idToCacheObject[id], false) != Success)
			return Failed;
		out = idToCacheObject[id]->isLoaded() ? idToCacheObject[id]->getMesh() : nullptr;
		return out == nullptr ? Failed : Success;
	}
	return result;
}

void SurfelManager::doStoreMesh(const Util::StringIdentifier& id, const Util::FileName& filename, Rendering::Mesh* mesh, bool async) {
	auto it = idToCacheObject.find(id);
	CacheObject* object = nullptr;
	if(it != idToCacheObject.end()) {

		object = it->second;
		uint64_t mem = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		if(object->isLoaded()) {
			usedMemory -= object->memory;
		} else if(object->isPending() || object->isLoading()) {
			object->state = CacheObject::Loaded;
			reservedMemory -= object->memory;
		} else {
			object->state = CacheObject::Loaded;
		}
		object->memory = mem;
		usedMemory += mem;
		object->setMesh(mesh);
	}

	std::function<void()> writeFn = [mesh, object, filename] () {
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

SurfelManager::MeshLoadResult_t SurfelManager::doFetchMesh(const Util::StringIdentifier& id, bool isSurfel, uint32_t level, float projSize, float distance, bool async) {
	CacheObject* object = nullptr;
	auto it = idToCacheObject.find(id);
	if(it != idToCacheObject.end()) {
		object = it->second;
		object->level = level; //TODO: problematic with instancing. use min level instead?
		object->projSize = projSize;
		object->minDistance = distance;
		if(object->lru == frameNumber) {
			++object->usage;
		} else {
			object->usage = 1;
		}
		object->lru = frameNumber;
		if(object->isLoaded() && object->getMesh()) {
			++worker->cacheHits;
			return Success;
		} else if(object->isPending() || object->isEmpty()) {
			++worker->cacheMisses;
			return Pending;
		}
		return Failed;
	}

	object = createCacheObject(id);
	object->level = level; //TODO: problematic with instancing. use min level instead?
	object->usage = 1;
	object->projSize = projSize;
	object->minDistance = distance;
	object->lru = frameNumber;
	object->setMesh(new Mesh());
	object->state = CacheObject::Pending;
	object->memory = 0;
	object->surfel = isSurfel;

	idToCacheObject[id] = object;
	pendingCacheObjects.push_back(object);
	++worker->cacheMisses;
	return Pending;
}

SurfelManager::MeshLoadResult_t SurfelManager::doLoadMesh(CacheObject* object, bool async) {
	if(!object || object->isEmpty() || object->isAborted())
		return Failed;
	if(object->isLoading())
		return Pending;

	const FileName& filename = object->isSurfel() ? buildSurfelFilename(basePath, object->getId()) : buildMeshFilename(basePath, object->getId());
	if(!FileUtils::isFile(filename)) {
		WARN("File '" + filename.toString() + "' does not exist.");
		return Failed;
	}

	// estimate memory usage
	uint64_t fileSize = FileUtils::fileSize(filename);
	reservedMemory += fileSize;
	object->memory = fileSize; // only estimates the real memory consumption

	pending++;
	Util::Timer timer;
	std::function<void()> readFn = [this, object, filename, timer] () {
		Reference<Mesh> mesh = Serialization::loadMesh(filename);
		// TODO: use this method only for surfels (faster for smaller files?)
		//std::string data = FileUtils::getFileContents(filename);
		//Reference<Mesh> mesh = Serialization::loadMesh("mmf", data);

		if(mesh.isNull()) {
			WARN("Could not load mesh: " + filename.toString());
			object->state = CacheObject::Empty;
			pending--;
			return;
		}
		object->setMesh(mesh.get());
		reservedMemory += mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
		reservedMemory -= object->memory;
		object->memory = mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
		object->state = CacheObject::Loaded;
		pending--;
		worker->accumCacheUpdates++;
		atomic_fetch_add(&worker->accumCacheTime, timer.getMilliseconds()); // += doesn't seem to be implemented in std::atomic for doubles
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

void SurfelManager::update() {
	static ObjectCompare<CacheObject> cacheCmp;
	while(!worker->mainThreadQueue.empty()) {
		worker->mainThreadQueue.pop()();
	}

	LOG_STAT(pendingCache, pendingCacheObjects.size());
	LOG_STAT(mainCache, sortedCacheObjects.size());
	LOG_STAT(accumLoadTime, worker->accumCacheTime.load());
	LOG_STAT(accumLoadNumber, worker->accumCacheUpdates.load());
	LOG_STAT(hits, worker->cacheHits.load());
	LOG_STAT(misses, worker->cacheMisses.load());
	worker->cacheHits = 0;
	worker->cacheMisses = 0;

	++frameNumber;
	uint32_t requestsPerFrame = 0;
	uint32_t handled = 0;
	static Util::Timer sortTimer;
	sortTimer.reset();
	// Sort pending objects and load by priority
	//FIXME: std::sort throws segmentation fault (probably because of weak ordering)
	std::stable_sort(pendingCacheObjects.begin(), pendingCacheObjects.end(), cacheCmp);
	LOG_STAT(pendingSortTime, sortTimer.getMilliseconds());
	sortTimer.reset();
	cacheObjectBuffer.clear();
	bool cacheDirty = false;

	//avoid #pending objects get too large
	while(pendingCacheObjects.size() > maxPending && ++handled < pendingCacheObjects.size()) {
		CacheObject* object = pendingCacheObjects.back();
		if(object->isPending() || object->isEmpty()) {
			pendingCacheObjects.pop_back();
			idToCacheObject.erase(object->getId());
			reservedMemory -= object->memory;
			releaseCacheObject(object);
		} else if(object->isLoaded()) {
			pendingCacheObjects.pop_back();
			reservedMemory -= object->memory;
			usedMemory += object->memory;
			sortedCacheObjects.push_back(object);
			cacheDirty = true;
		} else {
			break;
		}
	}

	while(!pendingCacheObjects.empty()) {
		CacheObject* object = pendingCacheObjects.front();
		pendingCacheObjects.pop_front();
		if(object->isEmpty()) {
			idToCacheObject.erase(object->getId());
			reservedMemory -= object->memory;
			releaseCacheObject(object);
		} else if(object->isLoaded()) {
			reservedMemory -= object->memory;
			usedMemory += object->memory;
			sortedCacheObjects.push_back(object);
			cacheDirty = true;
		} else if(object->isLoading()) {
			cacheObjectBuffer.push_back(object);
		} else if(object->isPending()) {
			// Load mesh unless max pending memory exceeded
			if(requestsPerFrame <= maxPerFrameRequestMem && reservedMemory < maxReservedMemory && usedMemory<=maxMemory) {
				if(doLoadMesh(object, true) == Failed)
					object->state = CacheObject::Empty;
				requestsPerFrame += object->memory;
				cacheObjectBuffer.push_back(object);
			} else {
				if(cacheObjectBuffer.empty()) {
					cacheObjectBuffer.swap(pendingCacheObjects);
					cacheObjectBuffer.push_back(object);
				} else {
					cacheObjectBuffer.push_back(object);
					if(cacheObjectBuffer.size()<pendingCacheObjects.size()) {
						pendingCacheObjects.insert(pendingCacheObjects.end(),
								std::make_move_iterator(cacheObjectBuffer.begin()),
								std::make_move_iterator(cacheObjectBuffer.end()));
						cacheObjectBuffer.clear();
						/*while(!cacheObjectBuffer.empty()) {
							pendingCacheObjects.push_front(cacheObjectBuffer.back());
							cacheObjectBuffer.pop_back();
						}*/
						cacheObjectBuffer.swap(pendingCacheObjects);
					} else {
						cacheObjectBuffer.insert(cacheObjectBuffer.end(),
								std::make_move_iterator(pendingCacheObjects.begin()),
								std::make_move_iterator(pendingCacheObjects.end()));
						pendingCacheObjects.clear();
						/*while(!pendingCacheObjects.empty()) {
							cacheObjectBuffer.push_back(pendingCacheObjects.front());
							pendingCacheObjects.pop_front();
						}*/
					}
				}
			}
		}
	}
	pendingCacheObjects.swap(cacheObjectBuffer);

	LOG_STAT(pendingIterate, sortTimer.getMilliseconds());

	sortTimer.reset();
	// if maximum memory is exceeded, sort cache and remove lru
	//if(usedMemory >= maxMemory * memoryLoadFactor && (usedMemory>maxMemory || cacheDirty)) {
	if(usedMemory >= maxMemory) {
		//FIXME: std::sort throws segmentation fault (probably because of weak ordering)
		std::stable_sort(sortedCacheObjects.begin(), sortedCacheObjects.end(), cacheCmp);
		LOG_STAT(cacheSortTime, sortTimer.getMilliseconds());
		sortTimer.reset();
		cacheObjectBuffer.clear();
		while(!sortedCacheObjects.empty()) {
			CacheObject* object = sortedCacheObjects.back();
			sortedCacheObjects.pop_back();

			if(object->isLoaded()) {
				// only remove object from cache when there is an pending object with higher priority
				//CacheObject* firstPending = pendingCacheObjects.empty() ? nullptr : pendingCacheObjects.front();
				if(usedMemory > maxMemory) {
					idToCacheObject.erase(object->getId());
					usedMemory -= object->memory;
					releaseCacheObject(object);
				} else {
					// lru object in cache has greater priority than all pending objects,
					// therefore we can assume that this is true for all objects in the cache
					if(cacheObjectBuffer.empty()) {
						cacheObjectBuffer.swap(sortedCacheObjects);
						cacheObjectBuffer.push_back(object);
					} else {
						cacheObjectBuffer.push_back(object);
						if(cacheObjectBuffer.size()<sortedCacheObjects.size()) {
							sortedCacheObjects.insert(sortedCacheObjects.end(),
									std::make_move_iterator(cacheObjectBuffer.begin()),
									std::make_move_iterator(cacheObjectBuffer.end()));
							cacheObjectBuffer.clear();
							/*while(!cacheObjectBuffer.empty()) {
								sortedCacheObjects.push_front(cacheObjectBuffer.back());
								cacheObjectBuffer.pop_back();
							}*/
							cacheObjectBuffer.swap(sortedCacheObjects);
						} else {
							cacheObjectBuffer.insert(cacheObjectBuffer.end(),
									std::make_move_iterator(sortedCacheObjects.begin()),
									std::make_move_iterator(sortedCacheObjects.end()));
							sortedCacheObjects.clear();
							/*while(!pendingCacheObjects.empty()) {
								cacheObjectBuffer.push_back(sortedCacheObjects.front());
								sortedCacheObjects.pop_front();
							}*/
						}
					}
				}
			} else {
				// move pending objects back to pending queue
				WARN("pending object " + object->getId().toString() + " in main cache.");
				pendingCacheObjects.push_back(object);
				reservedMemory += object->memory;
				usedMemory -= object->memory;
			}
		}
		sortedCacheObjects.swap(cacheObjectBuffer);
	} else {
		LOG_STAT(cacheSortTime, 0.0);
	}
	LOG_STAT(cacheIterate, sortTimer.getMilliseconds());
}

void SurfelManager::clear() {
	flush();
	//auto lock = Concurrency::createLock(*worker->memoryMutex);
	idToCacheObject.clear();
	cacheObjectBuffer.clear();
	while(!sortedCacheObjects.empty()) {
		CacheObject* object = sortedCacheObjects.back();
		sortedCacheObjects.pop_back();
		releaseCacheObject(object);
	}
	while(!pendingCacheObjects.empty()) {
		CacheObject* object = pendingCacheObjects.back();
		pendingCacheObjects.pop_back();
		releaseCacheObject(object);
	}
	reservedMemory = 0;
	usedMemory = 0;
	frameNumber = 0;
}

void SurfelManager::flush() {
	update();
	worker->flush(0);
}

void SurfelManager::executeAsync(const std::function<void()>& function) {
	worker->executeAsync(function);
}

void SurfelManager::executeOnMainThread(const std::function<void()>& function) {
	worker->mainThreadQueue.push(function);
}

SurfelManager::CacheObject* SurfelManager::createCacheObject(const Util::StringIdentifier& id) {
	if(cacheObjectPool.empty())
		return new CacheObject(id); //TODO: allocate space for multiple objects?
	CacheObject* object = cacheObjectPool.back();
	cacheObjectPool.pop_back();
	object->setId(id);
	return object;
}

SurfelManager::CacheObject* SurfelManager::createCacheObject(const CacheObject* copyOf) {
	CacheObject* object = createCacheObject(copyOf->getId());
	object->level = copyOf->level.load();
	object->lru = copyOf->lru.load();
	object->setMesh(copyOf->getMesh());
	object->minDistance = copyOf->minDistance.load();
	object->projSize = copyOf->projSize.load();
	object->state = copyOf->state.load();
	object->usage = copyOf->usage.load();
	object->memory = copyOf->memory.load();
	return object;
}

void SurfelManager::releaseCacheObject(CacheObject* object) {
	//std::cout << "releasing cache object " << object->id.toString() << std::endl;
	//TODO: check for max. pool size
	cacheObjectPool.push_back(object);
	object->state = CacheObject::Empty;
	object->level = 0;
	object->lru = 0;
	object->usage = 0;
	object->memory = 0;
	object->setMesh(nullptr);
}

SurfelManager::CacheObject* SurfelManager::getCacheObject(const Util::StringIdentifier& id) const {
	auto it = idToCacheObject.find(id);
	if(it == idToCacheObject.end())
		return nullptr;
	return it->second;
}

void SurfelManager::setPriorityOrder(const std::vector<uint32_t>& order) {
	// TODO: check bounds
	CacheObject::priorityOrder = order;
}

} /* namespace ThesisSascha */
} /* namespace MinSG */


#endif /* MINSG_EXT_THESISSASCHA */
