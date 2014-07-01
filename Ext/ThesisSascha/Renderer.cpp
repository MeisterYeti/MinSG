/*
	This file is part of the MinSG library extension ThesisSascha.
	Copyright (C) 2014 Sascha Brandt <myeti@mail.uni-paderborn.de>

	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifdef MINSG_EXT_THESISSASCHA

#include "Renderer.h"
#include "SurfelManager.h"
#include "Definitions.h"

#include <MinSG/Core/Nodes/Node.h>
#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/RenderParam.h>
#include <MinSG/Helper/StdNodeVisitors.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/RenderingContext/RenderingContext.h>
#include <Rendering/RenderingContext/RenderingParameters.h>

#include <Geometry/Rect.h>

#include <Util/Timer.h>

#include <forward_list>
#include <unordered_set>
#include <iostream>

namespace MinSG {
namespace ThesisSascha {

using namespace Util;
using namespace Rendering;

class _DistanceCompare {
	public:
		//! (ctor)
		_DistanceCompare(Geometry::Vec3 _referencePosition) :
			referencePosition(std::move(_referencePosition)) {
		}
	public:
		bool operator()(const Reference<Node>& a, const Reference<Node>& b) const {
			volatile const float d1 = getDistance(a.get());
			volatile const float d2 = getDistance(b.get());
			volatile const float l1 = getLevel(a.get());
			volatile const float l2 = getLevel(b.get());
			return d1 < d2 || ( !(d2 < d1) && ((l1 < l2) || ( !(l2 < l1) && (a.get() < b.get()))));
		}
	private:
		Geometry::Vec3 referencePosition;
		float getDistance(const Node * a) const {
			return a->getWorldBB().getDistanceSquared(referencePosition);
		}
		float getLevel(const Node * a) const {
			float lvl = 0;
			while(a->hasParent()) {
				a = a->getParent();
				++lvl;
			}
			return lvl;
		}
		//! Unimplemented, because the sort order must not be changed for an existing data structure.
		_DistanceCompare & operator=(const _DistanceCompare & other);
};

struct FetchResult {
	enum FetchType_t {
		Failed, RenderSurfel, RenderMesh, Skip
	} type;
	Reference<Node> node;
	Reference<Mesh> mesh;
};

class Renderer::Implementation {
public:
	Implementation(Renderer* renderer, SurfelManager* manager);
	~Implementation() = default;

	WeakPointer<Renderer> renderer;
	Reference<SurfelManager> manager;

	bool isValidFront(Node* node);

	FetchResult fetch(FrameContext& context, Node* node, const RenderParam& rp, float projSize, float distance);

	bool display(FrameContext& context, const RenderParam& rp, const FetchResult& result, float projSize, float distance);

	void renderCacheFront(FrameContext& context, const RenderParam& rp);
	NodeRendererResult displayNode(FrameContext& context, Node* node, const RenderParam& rp);
	bool enable(FrameContext & context, Node * node, const RenderParam & rp);
	void disable(FrameContext & context, Node * node, const RenderParam & rp);
public:
	std::unique_ptr<GenericAttributeMap> stats;

	float pointSizeFactor = 1.0;
	float minProjSize = 100;
	bool sortFront = false;
	bool useFrontRenderer = true;
	bool includeDistance = true;
	Timer frameTimer;
	uint32_t nodesRendered = 0;
private:
	typedef std::forward_list<Reference<Node>> CacheFront_t;
	CacheFront_t cacheFront;
	std::unordered_set<Node*> nodesInFront;
	Reference<Node> root;
};

Renderer::Implementation::Implementation(Renderer* renderer, SurfelManager* manager) : renderer(renderer), manager(manager), stats(new GenericAttributeMap) {}


bool Renderer::Implementation::isValidFront(Node* node) {
	GeometryNode* g = dynamic_cast<GeometryNode*>(node);
	return (g != nullptr && !g->getMesh()->empty())
			|| node->findAttribute(SURFEL_ID) != nullptr
			|| node->findAttribute(MESH_ID) != nullptr;
}

FetchResult Renderer::Implementation::fetch(FrameContext& context, Node* node, const RenderParam& rp, float size, float distance) {
	float qSize = std::sqrt(size);
	float far = context.getCamera()->getFarPlane();
	float invDistance = far-std::min(distance, far);

	bool cull = includeDistance ? qSize*invDistance < minProjSize*far : qSize < minProjSize ;
	if (!cull && rp.getFlag(FRUSTUM_CULLING)) {
		// TODO: autmatically cull child nodes in front?
		int t = context.getCamera()->testBoxFrustumIntersection(node->getWorldBB());
		cull = (t == Geometry::Frustum::OUTSIDE);
	}

	if(!isValidFront(node)) {
		return {FetchResult::Skip, node, nullptr};
	} else if(cull) {
		return {FetchResult::Failed, node, nullptr};
	}

	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node->isInstance() ? node->getPrototype() : node);
	if(geometry != nullptr) {
		Reference<Mesh> mesh;
		if(manager->fetchMesh(mesh, geometry, qSize, distance, true, false) == SurfelManager::Success) {
			if(mesh.isNull()) {
				WARN("Empty mesh found! " + node->findAttribute(MESH_ID)->toString());
				return {FetchResult::Failed, node, nullptr};
			}
			return {FetchResult::RenderMesh, node, mesh};
		} else if(!geometry->getMesh()->empty()) {
			return {FetchResult::RenderMesh, node, geometry->getMesh()};
		}
	}

	Reference<Mesh> mesh;
	SurfelManager::MeshLoadResult_t cacheResult = manager->fetchSurfel(mesh, node, qSize, distance, true, false);
	if(cacheResult == SurfelManager::Success ) {
		if(mesh.isNull()) {
			WARN("Empty surfel mesh found! " + node->findAttribute(SURFEL_ID)->toString());
			return {FetchResult::Failed, node, nullptr};
		}
		return {FetchResult::RenderSurfel, node, mesh};
	}

	return {FetchResult::Failed, node, nullptr};
}
bool Renderer::Implementation::display(FrameContext& context, const RenderParam& rp, const FetchResult& result, float projSize, float distance) {
	float far = context.getCamera()->getFarPlane();
	if(result.type == FetchResult::RenderMesh) {
		Renderer::drawMesh(context, result.node.get(), rp, result.mesh.get());
		return true;
	} else if(result.type == FetchResult::RenderSurfel) {
		uint32_t maxCount = result.mesh->isUsingIndexData() ? result.mesh->getIndexCount() : result.mesh->getVertexCount();
		float coverage = result.node->findAttribute(SURFEL_REL_COVERING) ? result.node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5;
		float far = context.getCamera()->getFarPlane();
		float invDistance = far-std::min(distance, far);
		//uint32_t count = clamp<uint32_t>(coverage*projSize*4, 0, maxCount);
		uint32_t count = maxCount;
		if(count > 0) {
			float pSize = std::max(1.0f, std::sqrt(coverage*projSize*pointSizeFactor/maxCount));
			Renderer::drawSurfels(context, result.node.get(), rp, result.mesh.get(), pSize, maxCount);
		}
		return true;
	}
	return false;
}

void Renderer::Implementation::renderCacheFront(FrameContext& context, const RenderParam& rp) {
	if(cacheFront.empty()) {
		cacheFront.push_front(root);
		nodesInFront.insert(root.get());
	}

	if(sortFront) {
		Geometry::Vec3 camPos = context.getCamera()->getWorldPosition();
		/*cacheFront.sort([camPos](const Reference<Node>& n1, const Reference<Node>& n2) {
			return n1->getWorldBB().getDistanceSquared(camPos) < n2->getWorldBB().getDistanceSquared(camPos);
		});*/
		cacheFront.sort(_DistanceCompare(camPos));
	}

	auto delIt = cacheFront.before_begin();
	auto it = cacheFront.begin();

	while(it != cacheFront.end()) {
		Node* node = it->get();
		Geometry::Rect projRect = context.getProjectedRect(node);
		float vpArea = context.getRenderingContext().getViewport().getArea();
		float size = std::min(projRect.getArea(), vpArea);
		float distance = node->getWorldBB().getDistance(context.getCamera()->getWorldPosition());

		FetchResult result = fetch(context, node, rp, size, distance);

		if(result.type == FetchResult::Failed ) {
			// remove node from front and backup to parent
			if(node->hasParent() && nodesInFront.count(node->getParent()) == 0) {
				// parent not in front -> add parent to front
				nodesInFront.insert(node->getParent());
				cacheFront.insert_after(it, node->getParent());
			}
			nodesInFront.erase(node);
			it = cacheFront.erase_after(delIt);
		} else {
			// fetch successful -> draw and check children
			// TODO: draw later (sorted)?
			if(display(context, rp, result, size, distance))
				++nodesRendered;

			++it;
			++delIt;

			for(Node* child : getChildNodes(node)) {
				if(child == node)
					continue;
				if(nodesInFront.count(child) <= 0) {
					nodesInFront.insert(child);
					delIt = cacheFront.insert_after(delIt, child);
				}
			}
		}
	}
}

NodeRendererResult Renderer::Implementation::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	if(useFrontRenderer) {
		renderCacheFront(context, rp);
		return NodeRendererResult::NODE_HANDLED;
	}

	Geometry::Rect projRect = context.getProjectedRect(node);
	float vpArea = context.getRenderingContext().getViewport().getArea();
	float size = std::min(projRect.getArea(), vpArea);
	float distance = node->getWorldBB().getDistance(context.getCamera()->getWorldPosition());

	FetchResult result = fetch(context, node, rp, size, distance);

	if(result.type == FetchResult::Failed ) {
		return NodeRendererResult::NODE_HANDLED;
	} else {
		// TODO: draw later (sorted)?
		if(display(context, rp, result, size, distance))
			++nodesRendered;
	}
	return NodeRendererResult::PASS_ON;
}

bool Renderer::Implementation::enable(FrameContext & context, Node * node, const RenderParam & rp) {
	frameTimer.reset();
	nodesRendered = 0;
	//TODO: clear cache front
	if(node != nullptr)
		root = node;
	if(!useFrontRenderer) {
		nodesInFront.clear();
		cacheFront.clear();
	}
	return true;
}
void Renderer::Implementation::disable(FrameContext & context, Node * node, const RenderParam & rp) {
	LOG_STAT(frontSize, nodesInFront.size());
	LOG_STAT(renderTime, frameTimer.getMilliseconds());
	LOG_STAT(nodesRendered, nodesRendered);
}

void Renderer::drawMesh(FrameContext& context, Node* node, const RenderParam& rp, Rendering::Mesh* mesh) {
	if(node->hasStates() && !(rp.getFlag(NO_STATES))) {
		for(auto & stateEntry : node->getStates()) {
			const State::stateResult_t result = stateEntry->enableState(context, node, rp);
		}
	}
	RenderingContext& rc = context.getRenderingContext();
	rc.pushMatrix();
	rc.resetMatrix();
	rc.multMatrix(node->getWorldMatrix());
	context.displayMesh(mesh);
	rc.popMatrix();
	if(node->hasStates() && !(rp.getFlag(NO_STATES))) {
		for(auto & stateEntry : node->getStates()) {
			stateEntry->disableState(context, node, rp);
		}
	}
}
void Renderer::drawSurfels(FrameContext& context, Node* node, const RenderParam& rp, Rendering::Mesh* mesh, float pSize, uint32_t count) {
	if(count == 0)
		return;
	RenderingContext& rc = context.getRenderingContext();
	rc.pushAndSetPointParameters(PointParameters(pSize, false));
	rc.pushMatrix();
	rc.resetMatrix();
	rc.multMatrix(node->getWorldMatrix());
	context.displayMesh(mesh, 0, count);
	rc.popMatrix();
	rc.popPointParameters();
}

Renderer::Renderer(SurfelManager* manager, Util::StringIdentifier channel) : NodeRendererState(channel), impl(new Renderer::Implementation(this, manager)) {}
Renderer::~Renderer() {}

NodeRendererResult Renderer::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	return impl->displayNode(context, node, rp);
}
State* Renderer::clone() const {
	return new Renderer(impl->manager.get(), this->getSourceChannel());
}
State::stateResult_t Renderer::doEnableState(FrameContext & context, Node * node, const RenderParam & rp) {
	if(!impl->enable(context, node, rp))
		return stateResult_t::STATE_SKIP_RENDERING;
	return NodeRendererState::doEnableState(context, node, rp);
}
void Renderer::doDisableState(FrameContext & context, Node * node, const RenderParam & rp) {
	NodeRendererState::doDisableState(context, node, rp);
	impl->disable(context, node, rp);
}
Util::GenericAttributeMap* Renderer::getStats() const { return impl->stats.get(); }

void Renderer::setPointSizeFactor(float value) { impl->pointSizeFactor = value; }
void Renderer::setMinProjSize(float value) { impl->minProjSize = value; }
void Renderer::sortFront(bool value) { impl->sortFront = value; }
void Renderer::useFrontRenderer(bool value) { impl->useFrontRenderer = value; }
void Renderer::includeDistance(bool value) { impl->includeDistance = value; }

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
