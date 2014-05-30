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
#include <Util/IO/FileUtils.h>

#include <iostream>


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
	StringIdentifier id;
	uint32_t lru;
	uint16_t usage;
	uint16_t level;
	float projSize;
	Reference<Mesh> mesh;
	enum State_t {
		Empty, Pending, Loaded
	} state;
	CacheObject() : id(), lru(0), usage(0), level(0), projSize(0), mesh(), state(Empty) {}
	CacheObject(const StringIdentifier& id) : id(id), lru(0), usage(0), level(0), projSize(0), mesh(), state(Empty) {}

	//TODO: better priority order
	bool operator<(const CacheObject & other) const {
		return (state == Empty) || (state == Loaded && other.state == Pending) || ((other.state != Empty)
				//&& (level < other.level || (!(other.level < level)
				&& (lru < other.lru || (!(other.lru < lru)
				&& (projSize < other.projSize || (!(other.projSize < projSize)
				&& usage < other.usage))
				)));
	}
	bool operator==(const CacheObject & other) const {
		return level == other.level
				&& lru == other.lru
				&& usage == other.usage
				&& state == other.state
				&& projSize == other.projSize; // should not be too much of a problem for sorting
	}

	inline bool isEmpty() { return state == Empty; }
	inline bool isPending() { return state == Pending; }
	inline bool isLoaded() { return state == Loaded; }
};

/******************************************
 * WorkerThread class
 ******************************************/

class WorkerThread : public UserThread {
public:
	enum IOJobType_t { CLOSE=0, READ=1, WRITE=2, FUNCTION=3 };
	struct SurfelIOJob_t {
		FileName file;
		Reference<Mesh> mesh;
		std::function<void()> function;
		IOJobType_t type;
	};

	WorkerThread(SurfelManager* manager) : closed(false), manager(manager) { start(); };
	virtual ~WorkerThread() {};
	void close();
	void write(const FileName& file, Mesh* mesh);
	void read(const FileName& file, Mesh* mesh);
	void executeAsync(const std::function<void()>& function);
	void flush();
protected:
	void run() override;
	void executeJob();

	SyncQueue<SurfelIOJob_t> jobQueue;
	bool closed;
	SurfelManager* manager;
public:
	typedef std::pair<Reference<Mesh>,Reference<Mesh>> MeshSwap_t;
	SyncQueue<MeshSwap_t> swapQueue;
	SyncQueue<std::function<void()>> mainThreadQueue;
};

void WorkerThread::close() {
	if(closed)
		return;
	closed = true;
	// push empty job to stop waiting on the queue (don't know if this is required, but just in case)
	jobQueue.push({});
}

void WorkerThread::executeJob() {
	if(closed)
		return;
	SurfelIOJob_t job = jobQueue.pop();
	if(job.type == READ) {
		if(!FileUtils::isFile(job.file)) {
			WARN("Could not load file: " + job.file.toString());
			return;
		}
		std::cout << "loading mesh " << job.file.toString() << std::endl;
		Mesh* mesh = Serialization::loadMesh(job.file);
		// swap on main thread
		swapQueue.push({job.mesh, mesh});
	} else if(job.type == WRITE) {
		std::cout << "saving mesh " << job.file.toString() << std::endl;
		Serialization::saveMesh(job.mesh.get(), job.file);
		FileUtils::flush(job.file);
	} else if(job.type == FUNCTION) {
		std::cout << "execute async function " << std::endl;
		job.function();
	}
}

void WorkerThread::run() {
	while(!closed) {
		executeJob();
	}
}

void WorkerThread::write(const FileName& file, Mesh* mesh) {
	if(closed)
		return;
	jobQueue.push({file, Reference<Mesh>(mesh), [] () {}, WRITE});
	if(jobQueue.size() > MAX_JOB_NUMBER)
		flush();
}

void WorkerThread::read(const FileName& file, Mesh* mesh) {
	if(closed)
		return;
	jobQueue.push({file, Reference<Mesh>(mesh), [] () {}, READ});
	if(jobQueue.size() > MAX_JOB_NUMBER)
		flush();
}

void WorkerThread::executeAsync(const std::function<void()>& function) {
	if(closed)
		return;
	SurfelIOJob_t job;
	job.function = function;
	job.type = FUNCTION;
	jobQueue.push(job);
	if(jobQueue.size() > MAX_JOB_NUMBER)
		flush();
}

void WorkerThread::flush() {
	if(closed)
		return;
	while(!closed && jobQueue.size() >= MAX_JOB_NUMBER) {
		Utils::sleep(10);
	}
}

/******************************************
 * SurfelManager class
 ******************************************/

SurfelManager::SurfelManager(const Util::FileName& basePath, uint64_t maxMemory) : basePath(basePath), worker(new WorkerThread(this)),
		preprocessor(new Preprocessor(this)), maxMemory(maxMemory), usedMemory(0), frameNumber(0) {
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

SurfelManager::MeshLoadResult_t SurfelManager::loadSurfel(Node* node, float projSize, bool async) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	return doLoadMesh(id, buildSurfelFilename(basePath, id), level, projSize, async);
}

bool SurfelManager::isCached(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID) && !node->isAttributeSet(MESH_ID))
		return true; // return true because there might be surfels in the subtree
	if(node->isAttributeSet(SURFEL_ID)) {
		const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
		auto it = idToCacheObject.find(id);
		if(it != idToCacheObject.end()) {
			return it->second->isLoaded();
		}
	}
	if(node->isAttributeSet(MESH_ID)) {
		const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
		auto it = idToCacheObject.find(id);
		if(it != idToCacheObject.end()) {
			return it->second->isLoaded();
		}
	}
	return false;
}

Mesh* SurfelManager::getSurfel(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);
	return idToCacheObject.count(id) > 0 ? idToCacheObject[id]->mesh.get() : nullptr;
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

SurfelManager::MeshLoadResult_t SurfelManager::loadMesh(GeometryNode* node, float projSize, bool async) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null
	if(!node->isAttributeSet(MESH_ID))
		return Failed;
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	uint32_t level = node->isAttributeSet(NODE_LEVEL) ? node->findAttribute(NODE_LEVEL)->toUnsignedInt() : 0;
	return doLoadMesh(id, buildMeshFilename(basePath, id), level, projSize, async);
}

Mesh* SurfelManager::getMesh(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	return idToCacheObject.count(id) > 0 ? idToCacheObject[id]->mesh.get() : nullptr;
}

void SurfelManager::doStoreMesh(const Util::StringIdentifier& id, const Util::FileName& filename, Rendering::Mesh* mesh, bool async) {
	auto it = idToCacheObject.find(id);
	CacheObject* object;
	if(it != idToCacheObject.end()) {
		object = it->second;
		// TODO: update memory usage
		object->state = CacheObject::Empty; // Mark as empty to reload mesh
	}

	if(async) {
		worker->write(filename, mesh);
	} else {
		std::cout << "saving mesh " << filename.toString() << " ...";
		Serialization::saveMesh(mesh, filename);
		std::cout << "done" << std::endl;
		FileUtils::flush(filename);
	}

	//reload mesh
	/*auto sIt = surfels.find(id);
	if(sIt != surfels.end()) {
		if(sIt->second.isNotNull())
			usedMemory -= sIt->second->getMainMemoryUsage() + sIt->second->getGraphicsMemoryUsage();
		surfels.erase(sIt);
	}*/
}

SurfelManager::MeshLoadResult_t SurfelManager::doLoadMesh(const Util::StringIdentifier& id, const Util::FileName& filename, uint32_t level, float projSize, bool async) {
	auto it = idToCacheObject.find(id);
	CacheObject* object = nullptr;
	if(it != idToCacheObject.end()) {
		object = it->second;
		object->level = level; //TODO: problematic with instancing. use min level instead?
		object->projSize = projSize;
		if(object->lru == frameNumber) {
			++object->usage;
		} else {
			object->usage = 1;
		}
		if(object->isLoaded() && object->mesh.isNotNull()) {
			return Success;
		} else if(object->isPending()) {
			return Pending;
		}
	}
	if(object == nullptr) {
		object = createCacheObject(id);
		object->level = level; //TODO: problematic with instancing. use min level instead?
		object->usage = 1;
		object->projSize = projSize;
		sortedCacheObjects.push_front(object);
		idToCacheObject[id] = object;
	}
	if(usedMemory >= maxMemory) {
		return Pending;
	}
	// FIXME: Here be Dragons! (infinite loop when surfel file not exist)
	if(!FileUtils::isFile(filename)) {
		//preprocessor->updateSurfels(frameContext, node);
		return Failed;
	}

	Mesh* mesh;
	if(async) {
		object->mesh = new Mesh();
		object->state = CacheObject::Pending;
		worker->read(filename, object->mesh.get());
		return Pending;
	} else {
		std::cout << "loading mesh " << filename.toString() << " ..." << std::flush;
		object->mesh = Serialization::loadMesh(filename);
		std::cout << "done" << std::endl;

		object->state = CacheObject::Loaded;
		//TODO: distinction between main memory and graphics memory
		usedMemory += object->mesh->getMainMemoryUsage() + object->mesh->getGraphicsMemoryUsage();
		std::cout << "memory usage: " << usedMemory << "/" << maxMemory << std::endl;
		return Success;
	}
}

void SurfelManager::update() {
	while(!worker->mainThreadQueue.empty()) {
		worker->mainThreadQueue.pop()();
	}
	while(!worker->swapQueue.empty()) {
		WorkerThread::MeshSwap_t meshSwap = worker->swapQueue.pop();
		// TODO: remove old mesh from used memory
		meshSwap.first->swap(*meshSwap.second.get());
		usedMemory += meshSwap.first->getMainMemoryUsage() + meshSwap.first->getGraphicsMemoryUsage();
		// TODO: update cache object state
	}
	++frameNumber;
	//TODO: update lru cache
	std::sort(sortedCacheObjects.begin(), sortedCacheObjects.end());
	while(usedMemory > maxMemory) {
		CacheObject* object = sortedCacheObjects.front();
		if(object->isPending()) {
			// object is not loaded yet. We have to wait until it is loaded.
			sortedCacheObjects.push_back(object);
			continue;
		} else if(object->isLoaded() && object->mesh.isNotNull()) {
			usedMemory -= object->mesh->getMainMemoryUsage() + object->mesh->getGraphicsMemoryUsage();
		}
		std::cout << "released " << object->id.toString() << ". memory usage: " << usedMemory << "/" << maxMemory << std::endl;
		sortedCacheObjects.pop_front();
		idToCacheObject.erase(object->id);
		releaseCacheObject(object);
	}
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

void SurfelManager::releaseCacheObject(CacheObject* object) {
	//std::cout << "releasing cache object " << object->id.toString() << std::endl;
	//TODO: check for max. pool size
	cacheObjectPool.push_back(object);
	object->state = CacheObject::Empty;
	object->level = 0;
	object->lru = 0;
	object->usage = 0;
	object->mesh = nullptr;
}

} /* namespace ThesisSascha */
} /* namespace MinSG */


#endif /* MINSG_EXT_THESISSASCHA */
