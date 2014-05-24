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
#include <list>
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
	enum MeshLoadResult_t {
		Success, Pending, Failed
	};

	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	SurfelManager(const Util::FileName& basePath, uint64_t maxMemory);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async = true);
	void attachSurfel(Node* node, const SurfelInfo_t& surfelInfo);

	MeshLoadResult_t loadSurfel(FrameContext& frameContext, Node* node, bool async = true);
	Rendering::Mesh* getSurfel(Node* node);
	void disposeSurfel(Node* node);

	void storeMesh(GeometryNode* node, bool async = true);
	MeshLoadResult_t loadMesh(GeometryNode* node, bool async = true);

	void update();

	Preprocessor* getPreprocessor() const { return preprocessor.get(); }
	const Util::FileName getBasePath() const { return basePath; }
	void setBasePath(const std::string& filename)  { basePath = Util::FileName(filename); }
	void setBasePath(const Util::FileName& filename)  { basePath = filename; }

	void executeAsync(const std::function<void()>& function);
	void executeOnMainThread(const std::function<void()>& function);

	void updateLRU(Util::StringIdentifier id);
	void unloadLRU();
private:
	Util::FileName basePath;
	WorkerThread* worker;
	std::unordered_map<Util::StringIdentifier, Util::Reference<Rendering::Mesh>> surfels;
	std::unordered_map<Rendering::Mesh*,Util::StringIdentifier> idByMesh;
	typedef std::list<Util::StringIdentifier> LRUCache_t;
	LRUCache_t lruCache;
	typedef std::unordered_map<Util::StringIdentifier, LRUCache_t::iterator> LRUCacheIndex_t;
	LRUCacheIndex_t lruCacheIndex;

	Util::Reference<Preprocessor> preprocessor;
	uint64_t maxMemory;
	uint64_t usedMemory;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
