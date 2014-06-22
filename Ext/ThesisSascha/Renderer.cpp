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

#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/Nodes/ListNode.h>
#include <MinSG/Core/RenderParam.h>
#include <MinSG/Helper/StdNodeVisitors.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/RenderingContext/RenderingContext.h>
#include <Rendering/RenderingContext/RenderingParameters.h>

#include <Geometry/Rect.h>

#include <Util/StringIdentifier.h>
#include <Util/GenericAttribute.h>
#include <Util/Utils.h>

#include <deque>
#include <string>

#define MAX_POINT_SIZE 64

namespace MinSG {
namespace ThesisSascha {
using namespace Util;
using namespace Rendering;

template<typename T, typename floatCompare, typename pointerCompare>
class _DistanceCompare {

	public:

		//! (ctor)
		_DistanceCompare(Geometry::Vec3 _referencePosition) :
			referencePosition(std::move(_referencePosition)) {
		}

		//! (ctor)
		_DistanceCompare(const _DistanceCompare & other) :
			referencePosition(other.referencePosition) {
		}

	public:
		/**
		 *
		 * @param 	a first object
		 * @param 	b second object
		 * @return 	true if the first object is closer (if order == BACK_TO_FRONT) to the reference position than the second
		 * 			if the objects have equal distance to the reference position the pointers of the objects are compared
		 *
		 * @see		float Geometry::Box::getDistanceSquared(position)
		 */
		bool operator()(const T * a, const T * b) const {
			volatile const float d1 = getDistance(a);
			volatile const float d2 = getDistance(b);
			volatile const float l1 = getLevel(b);
			volatile const float l2 = getLevel(b);
			return fltCmp(l1, l2) || ( !fltCmp(l2, l1) && (fltCmp(d1, d2) || ( !fltCmp(d2, d1) && ptrCmp(a, b))));
		}

	private:

		Geometry::Vec3 referencePosition;

		floatCompare fltCmp;
		pointerCompare ptrCmp;

		float getDistance(const Geometry::Vec3 * a) const {
			return a->distanceSquared(referencePosition);
		}

		float getDistance(const Node * a) const {
			return a->getWorldBB().getDistanceSquared(referencePosition);
		}

		float getDistance(const Geometry::Box * a) const {
			return a->getDistanceSquared(referencePosition);
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

struct SortedNodeSet: public std::set<Node *, _DistanceCompare<Node, std::less<volatile float>, std::less<const Node *> > > {
	SortedNodeSet(const Geometry::Vec3 & pos) :
		std::set<Node *, _DistanceCompare<Node, std::less<volatile float>, std::less<const Node *> > >(pos) {
	}
};

Renderer::Renderer(SurfelManager* manager, Util::StringIdentifier channel) : manager(manager), NodeRendererState(channel), async(false),
		immediate(false), waitForRender(false), timeLimit(0), currentComplexity(0), maxComplexity(0), renderTime(0), traversalTime(0) {
	countFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return static_cast<uint32_t>(coverage*projSize*4); };
	sizeFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return (coverage*projSize*4)/surfelNum; };
	refineNodeFn = [] (Node* node) { return RefineNode_t::RefineAndContinue; };
	activeNodes.reset(new SortedNodeSet(Geometry::Vec3()));
}

template<typename T>
inline T clamp(T value, T min, T max) {
	return std::max(min, std::min(max, value));
}

NodeRendererResult Renderer::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	if(immediate && timeLimit > 0 && frameTimer.getMilliseconds() > timeLimit)
		return NodeRendererResult::NODE_HANDLED;
	if(maxComplexity > 0 && currentComplexity > maxComplexity)
		return NodeRendererResult::NODE_HANDLED;
	RefineNode_t result = refineNodeFn(node);
	switch (result) {
		case RefineNode_t::RefineAndSkip:
			if(dynamic_cast<GeometryNode*>(node) != nullptr) {
				if(waitForRender)
					while(!doDisplayNode(context, node, rp)) {Utils::sleep(1); manager->update();}
				else if(immediate)
					doDisplayNode(context, node, rp);
				else
					activeNodes->insert(node);
					//activeNodes.push_back(node);
			}
			return NodeRendererResult::PASS_ON;
		case RefineNode_t::RefineAndContinue:
			if(waitForRender)
				while(!doDisplayNode(context, node, rp)) {Utils::sleep(1); manager->update();}
			else if(immediate)
				doDisplayNode(context, node, rp);
			else
				activeNodes->insert(node);
				//activeNodes.push_back(node);
			return NodeRendererResult::PASS_ON;
		case RefineNode_t::SkipChildren:
			if(waitForRender)
				while(!doDisplayNode(context, node, rp)) {Utils::sleep(1); manager->update();}
			else if(immediate)
				doDisplayNode(context, node, rp);
			else
				activeNodes->insert(node);
				//activeNodes.push_back(node);
			return NodeRendererResult::NODE_HANDLED;
		default:
			break;
	}
	return NodeRendererResult::NODE_HANDLED;
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

bool Renderer::doDisplayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	Geometry::Rect projectedRect(context.getProjectedRect(node));
	float size = projectedRect.getArea();
	float qSize = std::sqrt(size);
	float distance = node->getWorldBB().getDistanceSquared(context.getCamera()->getWorldPosition());

	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node->isInstance() ? node->getPrototype() : node);
	if(geometry != nullptr) {
		SurfelManager::MeshLoadResult_t result = manager->loadMesh(geometry, qSize, distance, async);
		if(result == SurfelManager::Success) {
			Mesh* mesh = manager->getMesh(node);
			if(mesh == nullptr) {
				WARN("Empty mesh found! " + node->findAttribute(MESH_ID)->toString());
				return false;
			}
			drawMesh(context, node, rp, mesh);
			currentComplexity += mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
			node->setAttribute(NODE_COMPLEXITY, GenericAttribute::createNumber(mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount()));
			return true;
		} else if(geometry->getMesh()->getVertexCount()>0) {
			currentComplexity += geometry->getMesh()->isUsingIndexData() ? geometry->getMesh()->getIndexCount() : geometry->getMesh()->getVertexCount();
			node->setAttribute(NODE_COMPLEXITY, GenericAttribute::createNumber(geometry->getMesh()->isUsingIndexData() ? geometry->getMesh()->getIndexCount() : geometry->getMesh()->getVertexCount()));
			return true;
		}
	}

	SurfelManager::MeshLoadResult_t result = manager->loadSurfel(node, qSize, distance, async);
	if(result == SurfelManager::Success ) {
		Mesh* mesh = manager->getSurfel(node);
		if(mesh == nullptr) {
			WARN("Empty surfel mesh found! " + node->findAttribute(SURFEL_ID)->toString());
			return false;
		}
		uint32_t maxCount = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		GenericAttribute* attr = node->findAttribute(SURFEL_REL_COVERING);
		float relCovering = attr == nullptr ? 0.5f : attr->toFloat();

		uint32_t count = clamp<uint32_t>(countFn(node, size, maxCount, relCovering), 0, maxCount);
		float pSize = sizeFn(node, size, maxCount, relCovering);
		if(pSize < 1)
			pSize = 1;

		drawSurfels(context, node, rp, mesh, pSize, count);
		currentComplexity += count;
		node->setAttribute(NODE_COMPLEXITY, GenericAttribute::createNumber(count));

		return true;
	}

	if(result == SurfelManager::Failed)
		return true;

	return false;
}

State* Renderer::clone() const {
	return new Renderer(manager.get(), this->getSourceChannel());
}

State::stateResult_t Renderer::doEnableState(FrameContext & context, Node * node, const RenderParam & rp) {
	const SortedNodeSet tempNodes = std::move(*activeNodes);
	frameTimer.reset();
	currentComplexity = 0;
	// use last frame
	for(auto & activeNode : tempNodes) {
		if(activeNode->isAttributeSet(NODE_COMPLEXITY))
			currentComplexity += activeNode->getAttribute(NODE_COMPLEXITY)->toUnsignedInt();
		if(maxComplexity > 0 && currentComplexity > maxComplexity)
			activeNode->unsetAttribute(NODE_HANDLED);
		else
			activeNode->setAttribute(NODE_HANDLED, GenericAttribute::createBool(true));
	}
	currentComplexity = 0;
	activeNodes.reset(new SortedNodeSet(context.getCamera()->getWorldPosition()));
	return NodeRendererState::doEnableState(context, node, rp);
}

void Renderer::doDisableState(FrameContext & context, Node * node, const RenderParam & rp) {
	NodeRendererState::doDisableState(context, node, rp);
	traversalTime = frameTimer.getMilliseconds();
	renderTime = traversalTime;

	if(immediate)
		return;

	//const SortedNodeSet tempNodes = std::move(*activeNodes);
	//activeNodes.reset();
	frameTimer.reset();

	for(auto & activeNode : *activeNodes) {
		if( (timeLimit > 0 && frameTimer.getMilliseconds() > timeLimit)
				|| (maxComplexity > 0 && currentComplexity > maxComplexity))
			break;
		doDisplayNode(context, activeNode, rp);
	}
	//activeNodes.clear();
	renderTime = frameTimer.getMilliseconds();
}

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
