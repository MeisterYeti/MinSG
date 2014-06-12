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
#include <deque>

namespace Rendering {
class Mesh;
}

namespace MinSG {
class Node;
class GeometryNode;

namespace ThesisSascha {

class WorkerThread;
class Preprocessor;
class CacheObject;

class SurfelManager : public Util::ReferenceCounter<SurfelManager> {
	PROVIDES_TYPE_NAME(SurfelManager)
public:
	enum MeshLoadResult_t {
		Success, Pending, Failed
	};

	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	SurfelManager(const Util::FileName& basePath, uint64_t maxMemory);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async = false);
	MeshLoadResult_t loadSurfel(Node* node, float projSize, float distance, bool async = false);
	Rendering::Mesh* getSurfel(Node* node);

	void storeMesh(GeometryNode* node, bool async = true);
	MeshLoadResult_t loadMesh(GeometryNode* node, float projSize, float distance, bool async = false);
	Rendering::Mesh* getMesh(Node* node);

	bool isCached(Node* node);

	void update();

	Preprocessor* getPreprocessor() const { return preprocessor.get(); }
	const Util::FileName getBasePath() const { return basePath; }
	void setBasePath(const std::string& filename)  { basePath = Util::FileName(filename); }
	void setBasePath(const Util::FileName& filename)  { basePath = filename; }

	void executeAsync(const std::function<void()>& function);
	void executeOnMainThread(const std::function<void()>& function);

	uint64_t getUsedMemory() const { return usedMemory; }
	uint64_t getMaxMemory() const { return maxMemory; }
	void setMaxMemory(uint64_t value) { maxMemory = value; }
	void setMaxJobs(uint32_t jobs)  { maxJobNumber = jobs; }
	uint32_t getMaxJobs() const { return maxJobNumber; }
	void setMemoryLoadFactor(float value)  { memoryLoadFactor = value; }
	float getMemoryLoadFactor() const { return memoryLoadFactor; }

	void setPriorityOrder(const std::vector<uint32_t>& order);
private:
	void doStoreMesh(const Util::StringIdentifier& id, const Util::FileName& filename, Rendering::Mesh* mesh, bool async);
	MeshLoadResult_t doLoadMesh(const Util::StringIdentifier& id, const Util::FileName& filename, uint32_t level, float projSize, float distance, bool async);

	CacheObject* createCacheObject(const Util::StringIdentifier& id);
	CacheObject* createCacheObject(const CacheObject* copyOf);
	void releaseCacheObject(CacheObject* object);
	CacheObject* getCacheObject(const Util::StringIdentifier& id) const;

	Util::FileName basePath;
	WorkerThread* worker;

	Util::Reference<Preprocessor> preprocessor;
	uint64_t maxMemory;
	uint64_t usedMemory;
	uint32_t frameNumber;
	uint32_t maxJobNumber;
	float memoryLoadFactor;

	typedef std::vector<CacheObject*> SortedCache_t;
	typedef std::unordered_map<Util::StringIdentifier, CacheObject*> IdToCacheMap_t;
	SortedCache_t sortedCacheObjects;
	SortedCache_t cacheObjectBuffer;
	SortedCache_t updatedCacheObjects;
	IdToCacheMap_t idToCacheObject;
	std::deque<CacheObject*> cacheObjectPool;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */