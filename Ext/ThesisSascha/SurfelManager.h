/*
 * SurfelManager.h
 *
 *  Created on: Apr 25, 2014
 *      Author: meisteryeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#ifndef SURFELMANAGER_H_
#define SURFELMANAGER_H_

#include <Util/References.h>
#include <Util/ReferenceCounter.h>
#include <Util/IO/FileName.h>
#include <Util/TypeNameMacro.h>

#include <vector>

namespace Rendering {
class Mesh;
}

namespace MinSG {
class Node;

namespace ThesisSascha {

class SurfelManager : public Util::ReferenceCounter<SurfelManager> {
	PROVIDES_TYPE_NAME(SurfelManager)
public:
	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	SurfelManager(const Util::FileName& basePath);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo);
	void attachSurfel(Node* node, const SurfelInfo_t& surfelInfo);
private:
	Util::FileName basePath;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
