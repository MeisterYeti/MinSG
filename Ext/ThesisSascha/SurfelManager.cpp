/*
 * SurfelManager.cpp
 *
 *  Created on: Apr 25, 2014
 *      Author: meisteryeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#include "SurfelManager.h"

#include <MinSG/Core/Nodes/Node.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Serialization/Serialization.h>

#include <Util/GenericAttribute.h>
#include <Util/GenericAttributeSerialization.h>
#include <Util/StringUtils.h>
#include <Util/Concurrency/Concurrency.h>
#include <Util/Concurrency/UserThread.h>
#include <Util/Concurrency/DataStructures/SyncQueue.h>

#include <iostream>

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;
using namespace Util::Concurrency;

class WorkerThread : public UserThread {
public:
	enum IOJobType_t { CLOSE=0, READ=1, WRITE=2 };
	struct SurfelIOJob_t {
		FileName file;
		Reference<Mesh> mesh;
		float relCovering;
		IOJobType_t type;
	};

	WorkerThread(SurfelManager* manager) : closed(false), manager(manager) { start(); };
	virtual ~WorkerThread() {};
	void close();
	void write(const FileName& file, Mesh* mesh, float relCovering);
	void read(const FileName& file, Mesh* mesh, float relCovering);
protected:
	void run() override;

	SyncQueue<SurfelIOJob_t> ioQueue;
	bool closed;
	SurfelManager* manager;
public:
	typedef std::pair<Reference<Mesh>,Reference<Mesh>> MeshSwap_t;
	SyncQueue<MeshSwap_t> swapQueue;
};

void WorkerThread::close() {
	if(closed)
		return;
	closed = true;
	// push empty job to stop waiting on the queue (don't know if this is required, but just in case)
	ioQueue.push({});
}

void WorkerThread::run() {
	while(!closed) {
		SurfelIOJob_t job = ioQueue.pop();
		if(job.type == READ) {
			std::cout << "loading surfel " << job.file.toString() << std::endl;
			// TODO: load relCovering
			Mesh* mesh = Serialization::loadMesh(job.file);
			// swap on main thread
			swapQueue.push({job.mesh, mesh});
		} else if(job.type == WRITE) {
			// TODO: save relCovering
			std::cout << "saving surfel " << job.file.toString() << std::endl;
			Serialization::saveMesh(job.mesh.get(), job.file);
		}
	}
}

void WorkerThread::write(const FileName& file, Mesh* mesh, float relCovering) {
	if(closed)
		return;
	ioQueue.push({file, Reference<Mesh>(mesh), relCovering, WRITE});
}

void WorkerThread::read(const FileName& file, Mesh* mesh, float relCovering) {
	if(closed)
		return;
	ioQueue.push({file, Reference<Mesh>(mesh), relCovering, READ});
}

static const StringIdentifier SURFEL_ID("surfelId");
static const StringIdentifier SURFEL_STRINGID("surfelStrId");
static const StringIdentifier SURFELS("surfels");
static const StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");

typedef WrapperAttribute<StringIdentifier> StringIDAttribute_t;
const std::string GATypeNameStringIdentifier("StringIdentifier");
const std::string GAStringIdentifierHeader("$[StringId]");

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

SurfelManager::SurfelManager(const Util::FileName& basePath) : basePath(basePath), worker(new WorkerThread(this)) {
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
	//TODO: create better ids?
	StringIdentifier id(StringUtils::createRandomString(32));
	if(!node->isAttributeSet(SURFEL_ID)) {
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(id.toString()));
		node->setAttribute(SURFEL_STRINGID, new StringIDAttribute_t(id));
	}
	//node->setAttribute(SURFELS, new ReferenceAttribute<Mesh>(surfelInfo.first.get()));
	node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
}

void SurfelManager::storeSurfel(Node* node, const SurfelInfo_t& surfelInfo) {
	if(node->isInstance())
		node = node->getPrototype();
	attachSurfel(node, surfelInfo);

	if(!node->isAttributeSet(SURFEL_STRINGID)) {
		StringIdentifier id(node->getAttribute(SURFEL_ID)->toString());
		node->setAttribute(SURFEL_STRINGID, new StringIDAttribute_t(id));
	}

	FileName surfelFile(basePath);
	surfelFile.setFile(node->getAttribute(SURFEL_ID)->toString());
	surfelFile.setEnding("mmf");

	worker->write(surfelFile, surfelInfo.first.get(), surfelInfo.second);
}

inline const StringIdentifier getStringId(Node* node) {
	StringIdentifier id;
	if(!node->isAttributeSet(SURFEL_STRINGID)) {
		id = node->getAttribute(SURFEL_ID)->toString();
		node->setAttribute(SURFEL_STRINGID, new StringIDAttribute_t(id));
	} else {
		const StringIDAttribute_t * objContainer = dynamic_cast<const StringIDAttribute_t *> (node->getAttribute(SURFEL_STRINGID));
		if(objContainer == nullptr) {
			WARN("Wrong attribute type for " + SURFEL_STRINGID.toString());
		} else {
			id = objContainer->get();
		}
	}
	return id;
}

bool SurfelManager::loadSurfel(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->isAttributeSet(SURFEL_ID))
		return false;

	const StringIdentifier id = getStringId(node);

	if(node->isAttributeSet(SURFELS) || surfels.count(id) > 0)
		return true;

	FileName surfelFile(basePath);
	surfelFile.setFile(node->getAttribute(SURFEL_ID)->toString());
	surfelFile.setEnding("mmf");

	Mesh* mesh = new Mesh();
	worker->read(surfelFile, mesh, node->isAttributeSet(SURFEL_REL_COVERING) ? node->getAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f);
	//node->setAttribute(SURFELS, new ReferenceAttribute<Mesh>(mesh));
	surfels[id] = mesh;
	//TODO: store&load relCovering
	//node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
	return false;
}

void SurfelManager::disposeSurfel(Node* node) {
}

Mesh* SurfelManager::getSurfel(Node* node) {
	if(node->isInstance())
		node = node->getPrototype();

	const StringIdentifier id = getStringId(node);

	if(surfels.count(id) > 0)
		return surfels[id].get();
	if(node->isAttributeSet(SURFELS)) {
		Mesh* mesh = node->getAttribute(SURFELS)->toType<ReferenceAttribute<Mesh>>()->get();
		surfels[id] = mesh;
		return mesh;
	}
	return nullptr;
}

void SurfelManager::update() {
	while(!worker->swapQueue.empty()) {
		WorkerThread::MeshSwap_t meshSwap = worker->swapQueue.pop();
		meshSwap.first->swap(*meshSwap.second.get());
	}
}

} /* namespace ThesisSascha */
} /* namespace MinSG */


#endif /* MINSG_EXT_THESISSASCHA */
