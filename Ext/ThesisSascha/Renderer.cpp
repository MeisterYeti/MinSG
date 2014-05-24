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

#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/Nodes/ListNode.h>
#include <MinSG/Core/RenderParam.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/RenderingContext/RenderingContext.h>
#include <Rendering/RenderingContext/RenderingParameters.h>

#include <Geometry/Rect.h>

#include <Util/StringIdentifier.h>
#include <Util/GenericAttribute.h>

#include <string>

#define MAX_POINT_SIZE 32

namespace MinSG {
namespace ThesisSascha {
using namespace Util;
using namespace Rendering;

static const StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");

Renderer::Renderer(SurfelManager* manager, Util::StringIdentifier channel) : manager(manager), NodeRendererState(channel) {
	transitionStart = [] (Node*) { return 200; };
	transitionEnd = [] (Node*) { return 100; };
	countFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return static_cast<uint32_t>(coverage*projSize*4); };
	sizeFn = [] (Node* node, float projSize, uint32_t surfelNum, float coverage) { return (coverage*projSize*4)/surfelNum; };
}

Renderer::~Renderer() {
}

template<typename T>
inline T clamp(T value, T min, T max) {
	return std::max(min, std::min(max, value));
}

NodeRendererResult Renderer::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
	if(geometry != nullptr) {
		manager->loadMesh(geometry, false);
	}
	// calculate treshold
	// if below threshold
	//   remove or ignore surfels
	//   abort subtree rendering
	// else
	//   if not has surfels
	//     load surfels
	//	   abort subtree rendering
	//   else
	//     draw surfels

	float tStart = transitionStart(node);
	Geometry::Rect projectedRect(context.getProjectedRect(node));
	float size = projectedRect.getArea();
	float qSize = std::sqrt(size);
	if(qSize > tStart)
		return NodeRendererResult::PASS_ON;
	float tEnd = transitionEnd(node);

	SurfelManager::MeshLoadResult_t result = manager->loadSurfel(context, node, false);
	if(result == SurfelManager::Success) {
		Mesh* mesh = manager->getSurfel(node);
		uint32_t maxCount = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		GenericAttribute* attr = node->findAttribute(SURFEL_REL_COVERING);
		float relCovering = attr == nullptr ? 0.5f : attr->toFloat();

		uint32_t count = clamp<uint32_t>(countFn(node, size, maxCount, relCovering), 1, maxCount);
		float pSize = clamp<float>(sizeFn(node, size, maxCount, relCovering), 1, MAX_POINT_SIZE);
		if(qSize>tEnd && tStart>tEnd){
			pSize *= (tStart-qSize) / (tStart-tEnd);
			pSize = clamp<float>(pSize,1,MAX_POINT_SIZE);
		}
		if(qSize>tEnd && tStart>tEnd){
			count *= (tStart-qSize) / (tStart-tEnd);
			count = clamp<uint32_t>(count,1,maxCount);
		}

		RenderingContext& rc = context.getRenderingContext();
		rc.pushAndSetPointParameters(PointParameters(pSize, false));
		rc.pushMatrix();
		rc.resetMatrix();
		rc.multMatrix(node->getWorldMatrix());
		context.displayMesh(mesh, 0, count);
		rc.popMatrix();
		rc.popPointParameters();

		return qSize<tEnd ? NodeRendererResult::NODE_HANDLED : NodeRendererResult::PASS_ON;
	}

	if(result == SurfelManager::Pending)
		return NodeRendererResult::NODE_HANDLED;
	return NodeRendererResult::PASS_ON;
}

State* Renderer::clone() const {
	return new Renderer(manager.get(), this->getSourceChannel());
}

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
