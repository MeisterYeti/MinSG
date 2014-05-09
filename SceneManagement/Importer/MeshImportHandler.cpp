/*
	This file is part of the MinSG library.
	Copyright (C) 2007-2012 Benjamin Eikel <benjamin@eikel.org>
	Copyright (C) 2007-2012 Claudius Jähn <claudius@uni-paderborn.de>
	Copyright (C) 2007-2012 Ralf Petring <ralf@petring.net>
	
	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the 
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/
#include "MeshImportHandler.h"
#include "../../Helper/Helper.h"
#include "../../Core/Nodes/GeometryNode.h"
#include "../SceneDescription.h"

#include <Util/IO/FileLocator.h>
#include <Util/Macros.h>
#include <Util/Encoding.h>

#include <Rendering/Serialization/Serialization.h>

namespace MinSG {
namespace SceneManagement {

Node * MeshImportHandler::handleImport(const Util::FileLocator& locator, const std::string & url, const NodeDescription * /*description*/) {
	return loadModel(Util::FileName(url), 0, nullptr,locator);
}

Node * MeshImportHandler::handleImport(const NodeDescription * description) {
	if(description->getValue(Consts::DATA_BLOCK) == nullptr)
		return nullptr;
	const std::string dataBlock = description->getString(Consts::DATA_BLOCK);
	if(description->getString(Consts::ATTR_DATA_ENCODING) != Consts::DATA_ENCODING_BASE64) {
		WARN("Unknown data block encoding.");
		return nullptr;
	}
	const std::vector<uint8_t> meshData = Util::decodeBase64(dataBlock);

	Rendering::Mesh * mesh = Rendering::Serialization::loadMesh("mmf", std::string(meshData.begin(), meshData.end()));
	if(mesh == nullptr) {
		WARN("Loading the mesh failed.");
		return nullptr;
	}
	return new GeometryNode(mesh);
}

}
}
