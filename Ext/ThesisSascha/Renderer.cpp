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

Renderer::Renderer(SurfelManager* manager, Util::StringIdentifier channel) : manager(manager), NodeRendererState(channel), async(false), immediate(false), waitForRender(false), timeLimit(0) {
	countFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return static_cast<uint32_t>(coverage*projSize*4); };
	sizeFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return (coverage*projSize*4)/surfelNum; };
	refineNodeFn = [] (Node* node) { return RefineNode_t::RefineAndContinue; };
}

template<typename T>
inline T clamp(T value, T min, T max) {
	return std::max(min, std::min(max, value));
}

NodeRendererResult Renderer::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	if(immediate && timeLimit > 0 && frameTimer.getMilliseconds() > timeLimit)
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
			return true;
		} else if(geometry->getMesh()->getVertexCount()>0) {
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
	frameTimer.reset();
	debugTimer.reset();
	activeNodes.reset(new DistanceSetF2B<Node>(context.getCamera()->getWorldPosition()));
	return NodeRendererState::doEnableState(context, node, rp);
}

void Renderer::doDisableState(FrameContext & context, Node * node, const RenderParam & rp) {
	NodeRendererState::doDisableState(context, node, rp);

	if(immediate)
		return;

	const DistanceSetF2B<Node> tempNodes = std::move(*activeNodes);
	activeNodes.reset();

	frameTimer.reset();
	//TODO: limit frame time
	for(auto & activeNode : tempNodes) {
		if(timeLimit > 0 && frameTimer.getMilliseconds() > timeLimit)
			return;
		doDisplayNode(context, activeNode, rp);
	}
	//activeNodes.clear();
}

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
