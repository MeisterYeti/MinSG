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
#include <Util/GenericAttribute.h>

#include <vector>
#include <unordered_map>
#include <list>
#include <functional>
#include <deque>
#include <atomic>

namespace Rendering {
class Mesh;
}

namespace MinSG {
class Node;
class GeometryNode;

namespace ThesisSascha {

class Preprocessor;

class SurfelManager : public Util::ReferenceCounter<SurfelManager> {
	PROVIDES_TYPE_NAME(SurfelManager)
public:
	enum MeshLoadResult_t {
		Success, Pending, Failed
	};

	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	SurfelManager(const Util::FileName& basePath, uint64_t maxMemory, uint64_t maxReservedMemory);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async = false);
	MeshLoadResult_t fetchSurfel(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async = false, bool force=false);

	void storeMesh(GeometryNode* node, bool async = true);
	MeshLoadResult_t fetchMesh(Util::Reference<Rendering::Mesh>& out, GeometryNode* node, float projSize, float distance, bool async = false, bool force=false);

	bool isCached(Node* node);

	void update();
	void clear();
	void flush();

	Preprocessor* getPreprocessor() const { return preprocessor.get(); }
	const Util::FileName getBasePath() const { return basePath; }
	void setBasePath(const std::string& filename)  { basePath = Util::FileName(filename); }
	void setBasePath(const Util::FileName& filename)  { basePath = filename; }

	void executeAsync(const std::function<void()>& function);
	void executeOnMainThread(const std::function<void()>& function);

	uint32_t getPending() const { return pending; }
	uint64_t getUsedMemory() const { return usedMemory; }
	uint64_t getReservedMemory() const { return reservedMemory; }
	uint64_t getMaxMemory() const { return maxMemory; }
	void setMaxMemory(uint64_t value) { maxMemory = value; }
	void setMaxPerFrameRequestMem(uint32_t value) { maxPerFrameRequestMem = value; }
	void setMaxReservedMemory(uint64_t value) { maxReservedMemory = value; }
	void setMaxJobs(uint32_t jobs)  { maxJobNumber = jobs; }
	uint32_t getMaxJobs() const { return maxJobNumber; }
	void setMaxPending(uint32_t value)  { maxPending = value; }
	void setMemoryLoadFactor(float value)  { memoryLoadFactor = value; }
	float getMemoryLoadFactor() const { return memoryLoadFactor; }
	void setMaxJobFlushTime(uint32_t time)  { maxJobFlushTime = time; }

	void setPriorityOrder(const std::vector<uint32_t>& order);

	Util::GenericAttributeMap * getStats() const { return stats.get(); };
private:
	class WorkerThread;
	class CacheObject;
	void doStoreMesh(const Util::StringIdentifier& id, const Util::FileName& filename, Rendering::Mesh* mesh, bool async);
	MeshLoadResult_t doFetchMesh(const Util::StringIdentifier& id, bool isSurfel, uint32_t level, float projSize, float distance, bool async);
	MeshLoadResult_t doLoadMesh(CacheObject* object, bool async);

	CacheObject* createCacheObject(const Util::StringIdentifier& id);
	CacheObject* createCacheObject(const CacheObject* copyOf);
	void releaseCacheObject(CacheObject* object);
	CacheObject* getCacheObject(const Util::StringIdentifier& id) const;

	Util::FileName basePath;
	WorkerThread* worker;

	Util::Reference<Preprocessor> preprocessor;
	std::atomic<uint64_t> maxMemory;
	std::atomic<uint64_t> usedMemory;
	std::atomic<uint64_t> reservedMemory;
	std::atomic<uint64_t> maxReservedMemory;
	std::atomic<uint32_t> pending;
	uint32_t frameNumber;
	uint32_t maxJobNumber;
	uint32_t maxJobFlushTime;
	uint32_t maxPerFrameRequestMem;
	uint32_t maxPending;
	float memoryLoadFactor;

	typedef std::deque<CacheObject*> SortedCache_t;
	typedef std::unordered_map<Util::StringIdentifier, CacheObject*> IdToCacheMap_t;
	SortedCache_t sortedCacheObjects;
	SortedCache_t cacheObjectBuffer;
	SortedCache_t pendingCacheObjects;
	IdToCacheMap_t idToCacheObject;
	std::deque<CacheObject*> cacheObjectPool;

	std::unique_ptr<Util::GenericAttributeMap> stats;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
