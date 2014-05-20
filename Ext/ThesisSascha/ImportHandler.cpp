#ifdef MINSG_EXT_THESISSASCHA

#include "ImportHandler.h"
#include "SurfelManager.h"

#include <MinSG/Helper/Helper.h>
#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/SceneManagement/SceneManager.h>
#include <MinSG/SceneManagement/Importer/ImporterTools.h>
#include <MinSG/SceneManagement/SceneDescription.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Serialization/Serialization.h>

#include <Util/IO/FileName.h>
#include <Util/IO/FileLocator.h>
#include <Util/StringUtils.h>
#include <Util/Macros.h>
#include <Util/Encoding.h>

#include <vector>
#include <string>
#include <iostream>

namespace MinSG {
namespace ThesisSascha {
using namespace Rendering;
using namespace SceneManagement;

//TODO: create external node when data exceeds limit?
Node * ImportHandler::handleImport(const Util::FileLocator& locator,const std::string & filename, const SceneManagement::NodeDescription * description) {
	GeometryNode* node = dynamic_cast<GeometryNode*>(loadModel(Util::FileName(filename), 0, nullptr,locator));
	manager->storeMesh(node, false);
	Util::Reference<Mesh> mesh = node->getMesh();
	node->setMesh(new Mesh);
	node->setFixedBB(mesh->getBoundingBox());
	return node;
}

Node * ImportHandler::handleImport(const SceneManagement::NodeDescription * description) {
	if(description->getValue(Consts::DATA_BLOCK) == nullptr)
		return nullptr;
	const std::string dataBlock = description->getString(Consts::DATA_BLOCK);
	if(description->getString(Consts::ATTR_DATA_ENCODING) != Consts::DATA_ENCODING_BASE64) {
		WARN("Unknown data block encoding.");
		return nullptr;
	}
	const std::vector<uint8_t> meshData = Util::decodeBase64(dataBlock);

	Util::Reference<Mesh> mesh = Rendering::Serialization::loadMesh("mmf", std::string(meshData.begin(), meshData.end()));
	if(mesh.isNull()) {
		WARN("Loading the mesh failed.");
		return nullptr;
	}
	GeometryNode* node = new GeometryNode(mesh);
	manager->storeMesh(node, false);
	node->setMesh(new Mesh);
	node->setFixedBB(mesh->getBoundingBox());
	return node;
}

void ImportHandler::pushImportHandler(SurfelManager* manager_) {
	std::cout << "push" << std::endl;
	std::unique_ptr<SceneManagement::MeshImportHandler> handler(new ImportHandler(manager_));
	SceneManagement::ImporterTools::setMeshImportHandler(std::move(handler));
}

void ImportHandler::popImportHandler() {
	std::cout << "pop" << std::endl;
	//TODO: restore last import handler
	std::unique_ptr<SceneManagement::MeshImportHandler> handler(new SceneManagement::MeshImportHandler);
	SceneManagement::ImporterTools::setMeshImportHandler(std::move(handler));
}

}
}

#endif /* MINSG_EXT_THESISSASCHA */
