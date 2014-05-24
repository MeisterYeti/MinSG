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

SurfelManager::SurfelManager(const Util::FileName& basePath, uint64_t maxMemory) : basePath(basePath), worker(new WorkerThread(this)),
		preprocessor(new Preprocessor(this)), maxMemory(maxMemory), usedMemory(0) {
	GenericAttributeSerialization::registerSerializer<WrapperAttribute<StringIdentifier>>(GATypeNameStringIdentifier, serializeID, unserializeID);
}

SurfelManager::~SurfelManager() {
	worker->close();
	worker->join();
	update();
	delete worker;
}

void SurfelManager::attachSurfel(Node* node, const SurfelInfo_t& surfelInfo) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID)) {
		StringIdentifier id(StringUtils::createRandomString(32));
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(id.toString()));
		node->setAttribute(SURFEL_STRINGID, new StringIDAttribute_t(id));
	}
	//node->setAttribute(SURFELS, new ReferenceAttribute<Mesh>(surfelInfo.first.get()));
	node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
}

void SurfelManager::storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async) {
	if(node->isInstance())
		node = node->getPrototype();
	attachSurfel(node, surfelInfo);

	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);

	FileName surfelFile(basePath.toString() + "surfels/");
	if(!FileUtils::isDir(surfelFile))
		FileUtils::createDir(surfelFile, true);
	surfelFile.setFile(id.toString());
	surfelFile.setEnding("mmf");

	if(async) {
		worker->write(surfelFile, surfelInfo.first.get());
	} else {
		std::cout << "saving mesh " << surfelFile.toString() << std::endl;
		Serialization::saveMesh(surfelInfo.first.get(), surfelFile);
		FileUtils::flush(surfelFile);
	}
	auto it = lruCacheIndex.find(id);
	if(it != lruCacheIndex.end()) {
		lruCacheIndex.erase(id);
		lruCache.erase(it->second);
	}
	auto sIt = surfels.find(id);
	if(sIt != surfels.end()) {
		if(sIt->second.isNotNull())
			usedMemory -= sIt->second->getMainMemoryUsage() + sIt->second->getGraphicsMemoryUsage();
		surfels.erase(sIt);
	}
}

SurfelManager::MeshLoadResult_t SurfelManager::loadSurfel(FrameContext& frameContext, Node* node, bool async) {
	Node* proto = node->isInstance() ? node->getPrototype() : node;
	if(!proto->isAttributeSet(SURFEL_ID))
		return Failed;

	const StringIdentifier id = getStringId(proto, SURFEL_ID, SURFEL_STRINGID);

	if(proto->isAttributeSet(SURFELS) || surfels.count(id) > 0)
		return Success;

	if(usedMemory >= maxMemory) {
		unloadLRU();
		return Pending;
	}

	FileName surfelFile(basePath.toString() + "surfels/");
	if(!FileUtils::isDir(surfelFile))
		FileUtils::createDir(surfelFile, true);
	surfelFile.setFile(id.toString());
	surfelFile.setEnding("mmf");

	// FIXME: Here be Dragons! (infinite loop when surfel file not exist)
	if(!FileUtils::isFile(surfelFile)) {
		//preprocessor->updateSurfels(frameContext, node);
		return Failed;
	}

	Mesh* mesh = new Mesh();
	if(async) {
		worker->read(surfelFile, mesh);
	} else {
		std::cout << "loading mesh " << surfelFile << std::endl;
		mesh = Serialization::loadMesh(surfelFile);
		//TODO: distinction between main memory and graphics memory
		usedMemory += mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
		std::cout << "memory usage: " << usedMemory << "/" << maxMemory << std::endl;
	}
	surfels[id] = mesh;
	updateLRU(id);
	return Pending;
}

void SurfelManager::disposeSurfel(Node* node) {
}

Mesh* SurfelManager::getSurfel(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();

	const StringIdentifier id = getStringId(node, SURFEL_ID, SURFEL_STRINGID);

	if(surfels.count(id) > 0) {
		updateLRU(id);
		return surfels[id].get();
	}
	if(node->isAttributeSet(SURFELS)) {
		Mesh* mesh = node->getAttribute(SURFELS)->toType<ReferenceAttribute<Mesh>>()->get();
		surfels[id] = mesh;
		return mesh;
	}
	return nullptr;
}

void SurfelManager::storeMesh(GeometryNode* node, bool async) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null

	if(!node->isAttributeSet(MESH_ID)) {
		StringIdentifier id(StringUtils::createRandomString(32));
		node->setAttribute(MESH_ID, GenericAttribute::createString(id.toString()));
	}
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);
	if(surfels.count(id) > 0)
		return;

	FileName file(basePath.toString() + "meshes/");
	if(!FileUtils::isDir(file))
		FileUtils::createDir(file, true);
	file.setFile(id.toString());
	file.setEnding("mmf");

	if(async) {
		worker->write(file, node->getMesh());
	} else {
		std::cout << "saving mesh " << file.toString() << std::endl;
		Serialization::saveMesh(node->getMesh(), file);
		FileUtils::flush(file);
	}
	auto it = lruCacheIndex.find(id);
	if(it != lruCacheIndex.end()) {
		lruCacheIndex.erase(id);
		lruCache.erase(it->second);
	}
	auto sIt = surfels.find(id);
	if(sIt != surfels.end()) {
		if(sIt->second.isNotNull())
			usedMemory -= sIt->second->getMainMemoryUsage() + sIt->second->getGraphicsMemoryUsage();
		surfels.erase(sIt);
	}
}

SurfelManager::MeshLoadResult_t SurfelManager::loadMesh(GeometryNode* node, bool async) {
	if(node->isInstance())
		node = dynamic_cast<GeometryNode*>(node->getPrototype()); // should not be null

	if(!node->isAttributeSet(MESH_ID)) {
		return Failed;
	}
	const StringIdentifier id = getStringId(node, MESH_ID, MESH_STRINGID);

	if(surfels.count(id) > 0)
		return Success;

	if(usedMemory >= maxMemory) {
		unloadLRU();
		return Pending;
	}

	FileName file(basePath.toString() + "meshes/");
	if(!FileUtils::isDir(file))
		FileUtils::createDir(file, true);
	file.setFile(id.toString());
	file.setEnding("mmf");

	if(!FileUtils::isFile(file)) {
		WARN("Mesh file '" + file.toString() + "' does not exist.");
		return Failed;
	}

	Mesh* mesh;
	if(async) {
		mesh = node->getMesh();
		worker->read(file, mesh);
	} else {
		std::cout << "loading mesh " << file << std::endl;
		mesh = Serialization::loadMesh(file);
		node->setMesh(mesh);
		usedMemory += mesh->getMainMemoryUsage() + mesh->getGraphicsMemoryUsage();
		std::cout << "memory usage: " << usedMemory << "/" << maxMemory << std::endl;
	}
	surfels[id] = mesh;
	updateLRU(id);
	return Pending;
}

void SurfelManager::update() {
	while(!worker->mainThreadQueue.empty()) {
		worker->mainThreadQueue.pop()();
	}
	while(!worker->swapQueue.empty()) {
		WorkerThread::MeshSwap_t meshSwap = worker->swapQueue.pop();
		meshSwap.first->swap(*meshSwap.second.get());
		usedMemory += meshSwap.first->getMainMemoryUsage() + meshSwap.first->getGraphicsMemoryUsage();
	}
}

void SurfelManager::executeAsync(const std::function<void()>& function) {
	worker->executeAsync(function);
}

void SurfelManager::executeOnMainThread(const std::function<void()>& function) {
	worker->mainThreadQueue.push(function);
}

void SurfelManager::updateLRU(Util::StringIdentifier id) {
	auto it = lruCacheIndex.find(id);
	if(it != lruCacheIndex.end()) {
		lruCache.erase(it->second);
	}
	auto end = lruCache.insert(lruCache.end(), id);
	lruCacheIndex[id] = end;
}

void SurfelManager::unloadLRU() {
	auto id = lruCache.front();
	lruCache.pop_front();
	lruCacheIndex.erase(id);

	auto sIt = surfels.find(id);
	if(sIt != surfels.end()) {
		if(sIt->second.isNotNull())
			usedMemory -= sIt->second->getMainMemoryUsage() + sIt->second->getGraphicsMemoryUsage();
		surfels.erase(sIt);
	}
	std::cout << "unload " << id.toString() << ", memory usage: " << usedMemory << "/" << maxMemory << std::endl;
}

} /* namespace ThesisSascha */
} /* namespace MinSG */


#endif /* MINSG_EXT_THESISSASCHA */
