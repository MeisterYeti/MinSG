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

	FetchResult fetch(Node* node, float projSize, float distance);

	void renderCacheFront(FrameContext& context, const RenderParam& rp);
	//bool displayNode(FrameContext& context, Node* node, const RenderParam& rp);
	bool enable(FrameContext & context, Node * node, const RenderParam & rp);
	void disable(FrameContext & context, Node * node, const RenderParam & rp);
public:
	std::unique_ptr<GenericAttributeMap> stats;

	float pointSizeFactor = 1.0;
	float minProjSize = 100;
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

FetchResult Renderer::Implementation::fetch(Node* node, float projSize, float distance) {
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node->isInstance() ? node->getPrototype() : node);
	if(geometry != nullptr) {
		Reference<Mesh> mesh;
		if(manager->fetchMesh(mesh, geometry, projSize, distance, true, false) == SurfelManager::Success) {
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
	SurfelManager::MeshLoadResult_t result = manager->fetchSurfel(mesh, node, projSize, distance, true, false);
	if(result == SurfelManager::Success ) {
		if(mesh.isNull()) {
			WARN("Empty surfel mesh found! " + node->findAttribute(SURFEL_ID)->toString());
			return {FetchResult::Failed, node, nullptr};
		}
		return {FetchResult::RenderSurfel, node, mesh};
	}

	return {FetchResult::Failed, node, nullptr};
}

void Renderer::Implementation::renderCacheFront(FrameContext& context, const RenderParam& rp) {
	static Timer frameTimer;
	frameTimer.reset();
	if(cacheFront.empty()) {
		cacheFront.push_front(root);
		nodesInFront.insert(root.get());
	}

	auto delIt = cacheFront.before_begin();
	auto it = cacheFront.begin();

	while(it != cacheFront.end()) {
		Node* node = it->get();
		Geometry::Rect projRect = context.getProjectedRect(node);
		float size = projRect.getArea();
		float qSize = std::sqrt(size);
		float distance = node->getWorldBB().getDistanceSquared(context.getCamera()->getWorldPosition());

		//TODO: use refine criteria
		bool cull = qSize < minProjSize;
		if (!cull && rp.getFlag(FRUSTUM_CULLING)) {
			// TODO: autmatically cull child nodes in front?
			int t = context.getCamera()->testBoxFrustumIntersection(node->getWorldBB());
			cull = (t == Geometry::Frustum::OUTSIDE);
		}
		FetchResult result = {FetchResult::Failed, node, nullptr};
		if(isValidFront(node)) {
			if(!cull)
				result = fetch(node, qSize, distance);
		} else {
			result = {FetchResult::Skip, node, nullptr};
		}
		if(result.type == FetchResult::Failed) {
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
			if(result.type == FetchResult::RenderMesh) {
				Renderer::drawMesh(context, node, rp, result.mesh.get());
			} else if(result.type == FetchResult::RenderSurfel) {
				uint32_t maxCount = result.mesh->isUsingIndexData() ? result.mesh->getIndexCount() : result.mesh->getVertexCount();
				float coverage = node->findAttribute(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5;

				//uint32_t count = clamp<uint32_t>(coverage*projSize*4, 0, maxCount);
				uint32_t count = maxCount;
				float vpArea = context.getRenderingContext().getViewport().getArea();
				float pSize = std::max(1.0f, std::sqrt((coverage*std::min(size, vpArea))/maxCount)*pointSizeFactor);
				Renderer::drawSurfels(context, node, rp, result.mesh.get(), pSize, maxCount);

			}

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
	LOG_STAT(renderTime, frameTimer.getMilliseconds());
	LOG_STAT(frontSize, nodesInFront.size());
}

bool Renderer::Implementation::enable(FrameContext & context, Node * node, const RenderParam & rp) {
	root = node;
	return true;
}
void Renderer::Implementation::disable(FrameContext & context, Node * node, const RenderParam & rp) {

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
	impl->renderCacheFront(context, rp);
	return NodeRendererResult::NODE_HANDLED;
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

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
