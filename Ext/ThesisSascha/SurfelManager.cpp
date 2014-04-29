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

#include <Util/StringUtils.h>

#include <iostream>

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;

static const StringIdentifier SURFEL_ID("surfelId");
static const StringIdentifier SURFELS("surfels");
static const StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");

SurfelManager::SurfelManager(const Util::FileName& basePath) : basePath(basePath) {
}

SurfelManager::~SurfelManager() {
}

void SurfelManager::attachSurfel(Node* node, const SurfelInfo_t& surfelInfo) {
	if(node->isInstance())
		node = node->getPrototype();
	//TODO: create better ids?
	if(!node->isAttributeSet(SURFEL_ID))
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(StringUtils::createRandomString(10)));
	node->setAttribute(SURFELS, new ReferenceAttribute<Mesh>(surfelInfo.first.get()));
	node->setAttribute(SURFEL_REL_COVERING, GenericAttribute::createNumber(surfelInfo.second));
}

void SurfelManager::storeSurfel(Node* node, const SurfelInfo_t& surfelInfo) {
	if(node->isInstance())
		node = node->getPrototype();
	attachSurfel(node, surfelInfo);

	FileName surfelFile(basePath);
	surfelFile.setFile(node->getAttribute(SURFEL_ID)->toString());
	surfelFile.setEnding("mmf");

	std::cout << "saving surfel " << surfelFile.toString() << std::endl;
	//TODO: do asynchronous
	Serialization::saveMesh(surfelInfo.first.get(), surfelFile);
}

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_EXT_THESISSASCHA */
