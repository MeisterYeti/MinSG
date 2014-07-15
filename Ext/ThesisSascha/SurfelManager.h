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
#include <Util/AttributeProvider.h>

#include <vector>
#include <memory>
#include <functional>

namespace Rendering {
class Mesh;
}

namespace MinSG {
class Node;
class GeometryNode;

namespace ThesisSascha {

class SurfelManager : public Util::ReferenceCounter<SurfelManager>, public Util::AttributeProvider {
	PROVIDES_TYPE_NAME(SurfelManager)
public:
	enum MeshLoadResult_t {
		Success, // cache hit
		Pending, // cache miss
		Failed // error (e.g. file not exists)
	};

	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	// void (StringIdentifier id, uint32_t lru, uint32_t usage, uint32_t memory, uint32_t childCount, float projSize, float minDistance, bool isSurfel)
	typedef std::function<float(const Util::StringIdentifier&, uint32_t, uint32_t, uint32_t, uint32_t, float, float, bool)> RequestPriorityFn_t;

	SurfelManager(const Util::FileName& basePath, uint64_t maxMemory, uint64_t maxReservedMemory);
	virtual ~SurfelManager();

	void storeSurfel(Node* node, const SurfelInfo_t& surfelInfo, bool async = false);
	MeshLoadResult_t fetchSurfel(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async = false, bool force=false);

	void storeMesh(GeometryNode* node, bool async = true);
	MeshLoadResult_t fetchMesh(Util::Reference<Rendering::Mesh>& out, Node* node, float projSize, float distance, bool async = false, bool force=false);

	bool isCached(Node* node);
	bool isInFront(Node* node);

	void update();
	void clear();
	void flush();

	void executeAsync(const std::function<void()>& function);
	void executeOnMainThread(const std::function<void()>& function);

	const Util::FileName getBasePath() const;
	void setBasePath(const std::string& filename);
	void setBasePath(const Util::FileName& filename);

	void setMaxMemory(uint32_t value); // MB
	uint32_t getMaxMemory() const; // MB

	void setMemoryLoadFactor(double value);
	double getMemoryLoadFactor() const;

	void setMinReleaseLimit(uint32_t value);
	void setFrameReleaseLimit(uint32_t value);
	void setFrameEvictLimit(uint32_t value);

	void setSortRequests(bool value);
	void setRequestQueueSize(uint32_t value);

	void setRequestPriorityFn(const SurfelManager::RequestPriorityFn_t& fn);

	Util::GenericAttributeMap * getStats() const;
private:
	friend class WorkerThread;
	class Implementation;
	std::unique_ptr<Implementation> impl;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* SURFELMANAGER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
