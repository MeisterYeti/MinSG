/*
 * Definitions.h
 *
 *  Created on: 20.05.2014
 *      Author: MeisterYeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <Util/GenericAttribute.h>
#include <Util/StringIdentifier.h>

namespace MinSG {
namespace ThesisSascha {

#define MAX_JOB_NUMBER 10

typedef Util::WrapperAttribute<Util::StringIdentifier> StringIDAttribute_t;
const std::string GATypeNameStringIdentifier("StringIdentifier");
const std::string GAStringIdentifierHeader("$[StringId]");

static const Util::StringIdentifier SURFEL_ID("surfelId");
static const Util::StringIdentifier SURFEL_STRINGID("surfelStrId");
static const Util::StringIdentifier SURFELS("surfels");
static const Util::StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");

static const Util::StringIdentifier MESH_ID("meshId");
static const Util::StringIdentifier MESH_STRINGID("meshStrId");

}
}


#endif /* DEFINITIONS_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
