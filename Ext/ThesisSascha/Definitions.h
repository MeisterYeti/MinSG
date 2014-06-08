/*
 * Definitions.h
 *
 *  Created on: 20.05.2014
 *      Author: MeisterYeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <MinSG/Core/NodeAttributeModifier.h>

#include <Util/GenericAttribute.h>
#include <Util/StringIdentifier.h>

namespace MinSG {
namespace ThesisSascha {

static const uint32_t MAX_JOB_NUMBER = 100;
static const size_t INITIAL_POOL_SIZE = 200;

typedef Util::WrapperAttribute<Util::StringIdentifier> StringIDAttribute_t;
const std::string GATypeNameStringIdentifier("StringIdentifier");
const std::string GAStringIdentifierHeader("$[StringId]");

static const Util::StringIdentifier SURFEL_ID("surfelId");
static const Util::StringIdentifier SURFEL_STRINGID(NodeAttributeModifier::create("surfelStrId", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");
static const Util::StringIdentifier SURFEL_COUNT("surfelCount");

static const Util::StringIdentifier MESH_ID("meshId");
static const Util::StringIdentifier MESH_STRINGID(NodeAttributeModifier::create("meshStrId", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier MESH_COMPLEXITY("meshComplexity");

static const Util::StringIdentifier NODE_LEVEL("nodeLevel");
static const Util::StringIdentifier CHILDREN_LOADED(NodeAttributeModifier::create("childrenLoaded", NodeAttributeModifier::PRIVATE_ATTRIBUTE));

}
}


#endif /* DEFINITIONS_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
