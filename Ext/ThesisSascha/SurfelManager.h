/*
 * SurfelManager.h
 *
 *  Created on: Apr 25, 2014
 *      Author: meisteryeti
 */

#ifdef MINSG_EXT_THESISSASCHA

#ifndef SURFELMANAGER_H_
#define SURFELMANAGER_H_

#include <MinSG/Core/FrameContext.h>

#include <Util/References.h>
#include <Util/ReferenceCounter.h>
#include <Util/IO/FileName.h>
#include <Util/TypeNameMacro.h>
#include <Util/StringIdentifier.h>

#include <vector>
#include <unordered_map>
#include <functional>

namespace Rendering {
class Mesh;
}

namespace MinSG {
class Node;
class GeometryNode;

namespace ThesisSascha {

class WorkerThread;
class Preprocessor;

class SurfelManager : public Util::ReferenceCounter<SurfelManager> {
	PROVIDES_TYPE_NAME(SurfelManager)
public:
	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	SurfelManager(const Util::FileName& basePath);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async = true);
	void attachSurfel(Node* node, const SurfelInfo_t& surfelInfo);

	bool loadSurfel(FrameContext& frameContext, Node* node, bool async = true);
	Rendering::Mesh* getSurfel(Node* node);
	void disposeSurfel(Node* node);

	void storeMesh(GeometryNode* node, bool async = true);
	bool loadMesh(GeometryNode* node, bool async = true);

	void update();

	Preprocessor* getPreprocessor() const { return preprocessor.get(); }
	const Util::FileName getBasePath() const { return basePath; }
	void setBasePath(const Util::FileName& filename)  { basePath = filename; }

	void executeAsync(const std::function<void()>& function);
	void executeOnMainThread(const std::function<void()>& function);
private:
	Util::FileName basePath;
	WorkerThread* worker;
	std::unordered_map<Util::StringIdentifier, Util::Reference<Rendering::Mesh>> surfels;

	Util::Reference<Preprocessor> preprocessor;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
