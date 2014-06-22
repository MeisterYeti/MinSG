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

class CacheObject {
public:
	enum Priority {
		LRU, Usage, Distance, ProjSize, LevelTB, LevelBT, Memory
	};

	typedef std::function<bool(const CacheObject & a, const CacheObject & b)> CompareFn_t;
	static std::vector<CompareFn_t> priorityFunctions;
	static std::vector<uint32_t> priorityOrder;

	StringIdentifier id;
	uint32_t lru;
	uint16_t usage;
	uint16_t level;
	uint32_t memory;
	float projSize;
	float minDistance;
	Reference<Mesh> mesh;
	enum State_t {
		Empty, Pending, Loaded, Abort
	} state;
	CacheObject() : id(), lru(0), usage(0), level(0), memory(0), projSize(0), minDistance(0), mesh(), state(Empty) {}
	CacheObject(const StringIdentifier& id) : id(id), lru(0), usage(0), level(0), projSize(0), minDistance(0), memory(0), mesh(), state(Empty) {}

	//TODO: better priority order
	bool operator < (const CacheObject & other) const {
		/*return //(state == Empty) || (state == Loaded && other.state == Pending) || ((other.state != Empty)
				//state < other.state || (!(other.state < state)
				(true
				&& (lru < other.lru || (!(other.lru < lru)
				&& (minDistance > other.minDistance || (!(other.minDistance > minDistance)
				//&& (level < other.level || (!(other.level < level)
				&& (projSize < other.projSize || (!(other.projSize < projSize)
				&& usage < other.usage))
				)))));*/
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

	inline bool isEmpty() { return state == Empty; }
	inline bool isPending() { return state == Pending; }
	inline bool isLoaded() { return state == Loaded; }
	inline bool isAborted() { return state == Abort; }

	void swap(CacheObject* obj) {
		std::swap(this->id, obj->id);
		std::swap(this->lru, obj->lru);
		std::swap(this->usage, obj->usage);
		std::swap(this->level, obj->level);
		std::swap(this->projSize, obj->projSize);
		std::swap(this->minDistance, obj->minDistance);
		std::swap(this->state, obj->state);
		std::swap(this->mesh, obj->mesh);
		std::swap(this->memory, obj->memory);
	}

};

std::vector<CacheObject::CompareFn_t> CacheObject::priorityFunctions {
	[](const CacheObject & a, const CacheObject & b) { return a.lru < b.lru; },
	[](const CacheObject & a, const CacheObject & b) { return a.usage < b.usage; },
	[](const CacheObject & a, const CacheObject & b) { return a.minDistance > b.minDistance; },
	[](const CacheObject & a, const CacheObject & b) { return a.projSize < b.projSize; },
	[](const CacheObject & a, const CacheObject & b) { return a.level > b.level; },
	[](const CacheObject & a, const CacheObject & b) { return a.level < b.level; },
	[](const CacheObject & a, const CacheObject & b) { return a.memory < b.memory; }
};

std::vector<uint32_t> CacheObject::priorityOrder {
	CacheObject::LRU, CacheObject::Distance, CacheObject::ProjSize, CacheObject::Usage
};

struct CacheObjectCompare {

	bool operator()(const CacheObject * a, const CacheObject * b) const {
		if(a == nullptr || b == nullptr) {
			WARN("Null cache object");
			return true;
		}
		return *b < *a || (!(*a < *b) && b < a);
	}
};

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

	WorkerThread(SurfelManager* manager) : closed(false), manager(manager), memoryMutex(Concurrency::createMutex()) { start(); };
	virtual ~WorkerThread() = default;
	void close();
	void executeAsync(const std::function<void()>& function);
	void flush(uint32_t timeLimit = MAX_FLUSH_TIME);
protected:
	void run() override;

	SyncQueue<Job_t> jobQueue;
	bool closed;
	WeakPointer<SurfelManager> manager;
public:
	SyncQueue<MeshSwap_t> swapQueue;
	SyncQueue<std::function<void()>> mainThreadQueue;
	std::unique_ptr<Concurrency::Mutex> memoryMutex;
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

SurfelManager::SurfelManager(const Util::FileName& basePath, uint64_t maxMemory) : basePath(basePath), worker(new WorkerThread(this)),
		preprocessor(new Preprocessor(this)), maxMemory(maxMemory), usedMemory(0), maxBufferSize(maxMemory), bufferSize(0),
		frameNumber(0), maxJobNumber(MAX_JOB_NUMBER), maxJobFlushTime(30), memoryLoadFactor(0.8f) {
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

SurfelManager::MeshLoadResult_t SurfelManager::loadSurfel(Node* node, float projSize, float distance, bool async) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	return doLoadMesh(id, buildSurfelFilename(basePath, id), level, projSize, distance, async);
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
			if(it->second->isLoaded())
				return true;
		}
	}
	if(node->isAttributeSet(SURFEL_ID)) {
		const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
		auto it = idToCacheObject.find(id);
		if(it != idToCacheObject.end()) {
			if(it->second->isLoaded())
				return true;
		}
	}
	return false;
}

Mesh* SurfelManager::getSurfel(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	Mesh* mesh = idToCacheObject.count(id) > 0 ? idToCacheObject[id]->mesh.get() : nullptr;
	if(mesh != nullptr)
		node->setAttribute(SURFEL_COUNT, GenericAttribute::createNumber(mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount()));
	return mesh;
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

SurfelManager::MeshLoadResult_t SurfelManager::loadMesh(GeometryNode* node, float projSize, float distance, bool async) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null
	if(!node->isAttributeSet(MESH_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	return doLoadMesh(id, buildMeshFilename(basePath, id), level, projSize, distance, async);
}

Mesh* SurfelManager::getMesh(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	return idToCacheObject.count(id) > 0 ? idToCacheObject[id]->mesh.get() : nullptr;
}

void SurfelManager::doStoreMesh(const Util::StringIdentifier& id, const Util::FileName& filename, Rendering::Mesh* mesh, bool async) {
	auto it = idToCacheObject.find(id);
	CacheObject* object = nullptr;
	if(it != idToCacheObject.end()) {
		auto lock = Concurrency::createLock(*worker->memoryMutex);
		object = it->second;
		object->mesh = mesh;
		usedMemory -= object->memory;
		object->memory = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		usedMemory += object->memory;
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

SurfelManager::MeshLoadResult_t SurfelManager::doLoadMesh(const Util::StringIdentifier& id, const Util::FileName& filename, uint32_t level, float projSize, float distance, bool async) {
	auto it = idToCacheObject.find(id);
	CacheObject* object = nullptr;
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
		if(object->isLoaded() && object->mesh.isNotNull()) {
			return Success;
		} else if(object->isPending()) {
			return Pending;
		}
		//WARN("object empty ");
		return Failed;
	}
	/*{
		auto lock = Concurrency::createLock(*worker->memoryMutex);
		if(usedMemory >= maxMemory) {
			return Pending;
		}
	}*/
	if(!FileUtils::isFile(filename)) {
		return Failed;
	}
	if(object == nullptr) {
		object = createCacheObject(id);
		object->level = level; //TODO: problematic with instancing. use min level instead?
		object->usage = 1;
		object->projSize = projSize;
		object->minDistance = distance;
		object->lru = frameNumber;
		object->mesh = new Mesh();
		object->state = CacheObject::Pending;
		object->memory = 0;

		idToCacheObject[id] = object;
		sortedCacheObjects.push_back(object);
	}

	std::function<void()> readFn = [this, object, filename] () {
		{
			auto lock = Concurrency::createLock(*worker->memoryMutex);
			if(object->isAborted() || usedMemory > maxMemory) {
				object->state = CacheObject::Empty;
				return;
			}
		}
		//std::cout << "loading mesh " << filename.toString() << " ..." << std::flush;
		//Reference<Mesh> mesh = Serialization::loadMesh(filename);
		// TODO: use this method only for surfels (faster for small files?)
		std::string data = FileUtils::getFileContents(filename);
		Reference<Mesh> mesh = Serialization::loadMesh("mmf", data);
		//std::cout << "done" << std::endl;
		if(mesh.isNull()) {
			WARN("Could not load mesh: " + filename.toString());
			return;
		}
		auto lock = Concurrency::createLock(*worker->memoryMutex);
		if(object->isAborted()) {
			object->state = CacheObject::Empty;
			return;
		}
		//worker->swapQueue.push({object->id, object->mesh, mesh});
		object->mesh = mesh;
		object->memory = mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
		object->state = CacheObject::Loaded;
		//updatedCacheObjects.push_back(object);
		//TODO: distinction between main memory and graphics memory
		usedMemory += object->memory;
		//std::cout << "memory usage: " << usedMemory << "/" << maxMemory << std::endl;
	};

	if(async) {
		executeAsync(readFn);
	} else {
		readFn();
	}
	return Pending;
}

void SurfelManager::update() {
	while(!worker->mainThreadQueue.empty()) {
		worker->mainThreadQueue.pop()();
	}
	/*while(!worker->swapQueue.empty()) {
		WorkerThread::MeshSwap_t meshSwap = worker->swapQueue.pop();
		auto it = idToCacheObject.find(meshSwap.id);
		if(it != idToCacheObject.end()) {
			meshSwap.source->swap(*meshSwap.target.get());
		}
	}*/
	++frameNumber;
	//TODO: update lru cache
	auto lock = Concurrency::createLock(*worker->memoryMutex);
	if(usedMemory >= maxMemory) {
		//FIXME: std::sort throws segmentation fault (probably because of weak ordering)
		std::stable_sort(sortedCacheObjects.begin(), sortedCacheObjects.end(), CacheObjectCompare());
		cacheObjectBuffer.clear();
		while(!sortedCacheObjects.empty()) {
			CacheObject* object = sortedCacheObjects.back();
			sortedCacheObjects.pop_back();
			if(object->isEmpty()) {
				idToCacheObject.erase(object->id);
				releaseCacheObject(object);
			} else if(object->isAborted()) {
				// do nothing
			} else if(usedMemory >= maxMemory * memoryLoadFactor) {
				usedMemory -= object->memory;
				//std::cout << "released " << object->id.toString() << ". memory usage: " << usedMemory << "/" << maxMemory << std::endl;
				if(object->isPending()) {
					object->mesh = nullptr;
					object->state = CacheObject::Abort;
				} else {
					idToCacheObject.erase(object->id);
					releaseCacheObject(object);
				}
			} else {
				cacheObjectBuffer.push_back(object);
			}
		}
		sortedCacheObjects.swap(cacheObjectBuffer);
	}
}

void SurfelManager::clear() {
	update();
	auto lock = Concurrency::createLock(*worker->memoryMutex);
	cacheObjectBuffer.clear();
	while(!sortedCacheObjects.empty()) {
		CacheObject* object = sortedCacheObjects.back();
		sortedCacheObjects.pop_back();
		releaseCacheObject(object);
	}
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

CacheObject* SurfelManager::createCacheObject(const Util::StringIdentifier& id) {
	if(cacheObjectPool.empty())
		return new CacheObject(id); //TODO: allocate space for multiple objects?
	CacheObject* object = cacheObjectPool.back();
	cacheObjectPool.pop_back();
	object->id = id;
	return object;
}

CacheObject* SurfelManager::createCacheObject(const CacheObject* copyOf) {
	CacheObject* object = createCacheObject(copyOf->id);
	object->level = copyOf->level;
	object->lru = copyOf->lru;
	object->mesh = copyOf->mesh;
	object->minDistance = copyOf->minDistance;
	object->projSize = copyOf->projSize;
	object->state = copyOf->state;
	object->usage = copyOf->usage;
	object->memory = copyOf->memory;
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
	object->mesh = nullptr;
}

CacheObject* SurfelManager::getCacheObject(const Util::StringIdentifier& id) const {
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
