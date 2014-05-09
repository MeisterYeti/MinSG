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

#include <MinSG/Core/Nodes/Node.h>
#include <MinSG/Core/RenderParam.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/RenderingContext/RenderingContext.h>
#include <Rendering/RenderingContext/RenderingParameters.h>

#include <Geometry/Rect.h>

#include <Util/StringIdentifier.h>
#include <Util/GenericAttribute.h>

#include <string>

namespace MinSG {
namespace ThesisSascha {
using namespace Util;
using namespace Rendering;

static const StringIdentifier SURFEL_REL_COVERING("surfelRelCovering");

Renderer::Renderer(SurfelManager* manager, Util::StringIdentifier channel) : manager(manager), NodeRendererState(channel) {
	// TODO Auto-generated constructor stub

}

Renderer::~Renderer() {
	// TODO Auto-generated destructor stub
}

NodeRendererResult Renderer::displayNode(FrameContext& context, Node* node, const RenderParam& rp) {

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

	float tStart = 200; //TODO: calculate by customizable function
	Geometry::Rect projectedRect(context.getProjectedRect(node));
	float size = projectedRect.getArea();
	float qSize = size*size;
	//if(qSize > tStart)
	//	return NodeRendererResult::PASS_ON;
	float tEnd = 300; //TODO: calculate by customizable function


	if(manager->loadSurfel(context, node)) {
		Mesh* mesh = manager->getSurfel(node);
		uint32_t maxCount = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		GenericAttribute* attr = node->findAttribute(SURFEL_REL_COVERING);
		float relCovering = attr == nullptr ? 0.5f : attr->toFloat();

		uint32_t count = std::min(static_cast<uint32_t>(relCovering*size*4), maxCount); //TODO: calculate
		float pSize = (relCovering*size*4)/count; //TODO: calculate

		RenderingContext& rc = context.getRenderingContext();
		rc.pushAndSetPointParameters(PointParameters(pSize, false));
		rc.pushMatrix();
		rc.resetMatrix();
		rc.multMatrix(node->getWorldMatrix());
		context.displayMesh(mesh, 0, count);
		rc.popMatrix();
		rc.popPointParameters();

		return NodeRendererResult::NODE_HANDLED;
		//return qSize<tEnd ? NodeRendererResult::NODE_HANDLED : NodeRendererResult::PASS_ON;
	}

	return NodeRendererResult::PASS_ON;
}

State* Renderer::clone() const {
	return new Renderer(manager.get(), this->getSourceChannel());
}

} /* namespace ThesisSascha */
} /* namespace MinSG */
#endif /* MINSG_EXT_THESISSASCHA */
