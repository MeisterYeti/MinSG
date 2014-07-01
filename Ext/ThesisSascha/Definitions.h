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

#define MB_TO_BYTE(value) value * 1048576UL
#define BYTE_TO_MB(value) static_cast<uint32_t>(value / 1048576UL)

#define LOCK(mutex) auto lock = Util::Concurrency::createLock(mutex);

#define LOG_STAT(name, value) static StringIdentifier sId ## name(#name); \
	stats->setValue(sId ## name, GenericAttribute::createNumber(value));

static const uint32_t THREAD_COUNT = 2;
static const uint32_t REQUEST_QUEUE_SIZE = 8;

static const size_t MAX_POOL_SIZE = 1000;

typedef Util::WrapperAttribute<Util::StringIdentifier> StringIDAttribute_t;
const std::string GATypeNameStringIdentifier("StringIdentifier");
const std::string GAStringIdentifierHeader("$[StringId]");

static const Util::StringIdentifier SURFEL_ID("surfelId");
static const Util::StringIdentifier SURFEL_STRINGID(NodeAttributeModifier::create("surfelStrId", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");
static const Util::StringIdentifier SURFEL_COUNT("surfelCount");
static const Util::StringIdentifier SURFELS("surfels");

static const Util::StringIdentifier MESH_ID("meshId");
static const Util::StringIdentifier MESH_STRINGID(NodeAttributeModifier::create("meshStrId", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier MESH_COMPLEXITY("meshComplexity");

static const Util::StringIdentifier NODE_COMPLEXITY(NodeAttributeModifier::create("nodeComplexity", NodeAttributeModifier::PRIVATE_ATTRIBUTE));

static const Util::StringIdentifier NODE_LEVEL("nodeLevel");
static const Util::StringIdentifier NODE_HANDLED(NodeAttributeModifier::create("nodeHandled", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier NODE_RENDERED(NodeAttributeModifier::create("nodeRendered", NodeAttributeModifier::PRIVATE_ATTRIBUTE));
static const Util::StringIdentifier CHILDREN_LOADED(NodeAttributeModifier::create("childrenLoaded", NodeAttributeModifier::PRIVATE_ATTRIBUTE));

template<typename T>
inline T clamp(T value, T min, T max) {
	return std::max(min, std::min(max, value));
}

}
}

#endif /* DEFINITIONS_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
