/*
	This file is part of the MinSG library extension ThesisSascha.
	Copyright (C) 2014 Sascha Brandt <myeti@mail.uni-paderborn.de>

	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifdef MINSG_EXT_THESISSASCHA

#include "Preprocessor.h"
#include "SurfelManager.h"

#include <Geometry/Frustum.h>
#include <Geometry/Tools.h>
#include <Geometry/Rect.h>
#include <Geometry/Matrix4x4.h>

#include <Rendering/FBO.h>
#include <Rendering/Shader/Shader.h>
#include <Rendering/Shader/ShaderObjectInfo.h>
#include <Rendering/Shader/Uniform.h>
#include <Rendering/Texture/Texture.h>
#include <Rendering/Texture/TextureUtils.h>
#include <Rendering/RenderingContext/RenderingContext.h>
#include <Rendering/RenderingContext/RenderingParameters.h>
#include <Rendering/Mesh/Mesh.h>

#include <MinSG/Core/RenderParam.h>
#include <MinSG/Core/Nodes/Node.h>
#include <MinSG/Core/Nodes/CameraNodeOrtho.h>
//#include <MinSG/Helper/StdNodeVisitors.h>
#include <MinSG/Ext/BlueSurfels/SurfelGenerator.h>

#include <Util/Graphics/PixelAccessor.h>
#include <Util/GenericAttribute.h>

#include <functional>
#include <algorithm>

#define POSITION 0
#define NORMAL 1
#define COLOR 2
#define SIZE 3

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;


void forEachNodeBottomUp(Node * root, const std::function<NodeVisitor::status (Node *)>& enterFun, const std::function<void (Node *)>& leaveFun) {
	if(root == nullptr) {
		return;
	}
	struct Visitor : public NodeVisitor {
		const std::function<NodeVisitor::status (Node *)>& m_enter;
		const std::function<void (Node *)>& m_leave;
		Visitor(const std::function<NodeVisitor::status (Node *)>& p_enter, const std::function<void (Node *)>& p_leave) : m_enter(p_enter), m_leave(p_leave) {
		}
		virtual ~Visitor() = default;

		NodeVisitor::status enter(Node * node) override {
			return m_enter(node);
		}

		NodeVisitor::status leave(Node * node) override {
			m_leave(node);
			return CONTINUE_TRAVERSAL;
		}
	} visitor(enterFun, leaveFun);
	root->traverse(visitor);
}

Geometry::Vec3 Preprocessor::directions[8] = {
	Geometry::Vec3(1,1,1),
	Geometry::Vec3(1,1,-1),
	Geometry::Vec3(1,-1,1),
	Geometry::Vec3(1,-1,-1),
	Geometry::Vec3(-1,1,1),
	Geometry::Vec3(-1,1,-1),
	Geometry::Vec3(-1,-1,1),
	Geometry::Vec3(-1,-1,-1),
};

void updateEnclosingOrthoCam(CameraNodeOrtho* camera, const Geometry::Vec3& worldDir, Node* node) {
	Geometry::Vec3 nodeCenter = node->getWorldBB().getCenter();
	camera->setWorldPosition(nodeCenter - worldDir * node->getWorldBB().getExtentMax());
	camera->rotateToWorldDir(camera->getWorldPosition() - nodeCenter);

	Geometry::Frustum frustum = Geometry::calcEnclosingOrthoFrustum(node->getBB(), camera->getWorldMatrix().inverse() * node->getWorldMatrix());
	camera->setNearFar(frustum.getNear()*0.99, frustum.getFar()*1.01);
	if( frustum.getRight()-frustum.getLeft() > frustum.getTop() - frustum.getBottom()) {
		camera->rotateLocal_deg(90, Geometry::Vec3(0,0,1));
		camera->setClippingPlanes(frustum.getBottom(), frustum.getTop(), frustum.getLeft(), frustum.getRight());
	} else {
		camera->setClippingPlanes(frustum.getLeft(), frustum.getRight(), frustum.getBottom(), frustum.getTop());
	}
}

Preprocessor::Preprocessor(SurfelManager* manager) : verticalResolution(256), surfelGenerator(new BlueSurfels::SurfelGenerator()), manager(manager) {
	// do nothing
}

Preprocessor::~Preprocessor() {
	delete surfelGenerator;
}

std::pair<Reference<Mesh>,float> Preprocessor::createSurfelsForNode(FrameContext& frameContext, Node* node) {

	RenderingContext& rc = frameContext.getRenderingContext();

	// initialize cameras for each direction
	Geometry::Vec2i resolution;
	{
		float maxWidth;
		float maxHeight;
		auto it = cameras.begin();
		for(auto dir : Preprocessor::directions) {
			CameraNodeOrtho* camera = (*it).get();
			updateEnclosingOrthoCam(camera, dir, node);
			maxWidth = std::max(maxWidth, camera->getRightClippingPlane()-camera->getLeftClippingPlane());
			maxHeight = std::max(maxHeight, camera->getTopClippingPlane()-camera->getBottomClippingPlane());
			++it;
		}
		float scaling = (verticalResolution-2) / std::max(maxWidth,maxHeight);
		int x = 0;
		for(auto camera : cameras) {
			Geometry::Rect_i viewport(x, 0, std::ceil((camera->getRightClippingPlane()-camera->getLeftClippingPlane())*scaling),
					std::ceil((camera->getTopClippingPlane()-camera->getBottomClippingPlane())*scaling));
			camera->setViewport(viewport, false);
			x = viewport.getMaxX()+1;
		}
		resolution = Geometry::Vec2( std::round(x-5)+8+2, verticalResolution);
	}

	// render textures
	std::vector<Reference<Texture> > textures;
	{
		Geometry::Matrix4x4 inverseModelMatrix = node->getWorldMatrix().inverse();
		Reference<Texture> depthT = TextureUtils::createDepthTexture(resolution.x(), resolution.y());
		Reference<FBO> fbo(new FBO());
		fbo->attachDepthTexture(rc, depthT.get());
		Geometry::Rect_i screenRect(0,0,resolution.x(), resolution.y());
		Color4f clearColor(0,0,0,0);
		RenderParam rp(RenderFlags::USE_WORLD_MATRIX);

		static const StringIdentifier INVERSE_MODEL_MATRIX("inverseModelMatrix");

		for(uint_fast8_t pass = POSITION; pass <= COLOR; ++pass) {
			Shader* shader = shaders[pass].get();
			if(shader->isUniform(INVERSE_MODEL_MATRIX))
				shader->setUniform(rc, Uniform(INVERSE_MODEL_MATRIX, inverseModelMatrix));

			Texture* colorT = TextureUtils::createHDRTexture(resolution.x(), resolution.y(), true);

			fbo->attachColorTexture(rc, colorT, 0);
			rc.pushAndSetFBO(fbo.get());
			rc.pushAndSetShader(shader);
			rc.pushAndSetScissor(ScissorParameters(screenRect));
			rc.pushViewport();
			rc.setViewport(screenRect);
			rc.clearScreenRect(screenRect, clearColor, true);
			for(auto camera : cameras) {
//				rc.setImmediateMode(false);
//				rc.applyChanges(true);
				frameContext.setCamera(camera.get());
//				renderingContext.clearScreen(clearColor);
				frameContext.displayNode(node, rp);
//				rc.setImmediateMode(true);
			}
			rc.popViewport();
			rc.popScissor();
			rc.popShader();
			rc.popFBO();
			fbo->detachColorTexture(rc, 0);

			textures.push_back(colorT);
		}
		fbo->detachDepthTexture(rc);

		textures.push_back(TextureUtils::createStdTexture(resolution.x(), resolution.y(), true));


		fbo->attachColorTexture(rc, textures[SIZE].get(), 0);
		rc.pushAndSetFBO(fbo.get());
		rc.pushAndSetShader(shaders[SIZE].get());
		rc.pushAndSetScissor(ScissorParameters(screenRect));
		rc.pushViewport();
		rc.setViewport(screenRect);
		rc.clearScreenRect(screenRect, clearColor, true);
		TextureUtils::drawTextureToScreen(rc, screenRect, textures[NORMAL].get(), Geometry::Rect(0,0,1,1));
		rc.popViewport();
		rc.popScissor();
		rc.popShader();
		rc.popFBO();
		fbo->detachColorTexture(rc, 0);
	}

	return surfelGenerator->createSurfels(
		*TextureUtils::createColorPixelAccessor(rc, textures[POSITION].get()).get(),
		*TextureUtils::createColorPixelAccessor(rc, textures[NORMAL].get()).get(),
		*TextureUtils::createColorPixelAccessor(rc, textures[COLOR].get()).get(),
		*TextureUtils::createColorPixelAccessor(rc, textures[SIZE].get()).get()
	);
}

void Preprocessor::visitNode(FrameContext& frameContext, Node* node) {
	std::pair<Reference<Mesh>,float> surfelInfo = createSurfelsForNode(frameContext, node);
	//TODO: use node complexity threshold
	manager->storeSurfel(node, surfelInfo);
}

void Preprocessor::initShaders(const FileName& helperShader, const FileName& positionShader, const FileName& normalShader, const FileName& colorShader, const FileName& sizeShader) {
	cameras.clear();
	shaders.clear();

	// Create cameras
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());

	// position shader
	Shader* shader = Shader::loadShader(positionShader,positionShader,Shader::USE_UNIFORMS);
	shader->attachShaderObject(ShaderObjectInfo::loadVertex(helperShader));
	shader->attachShaderObject(ShaderObjectInfo::loadFragment(helperShader));
	shaders.push_back(shader);

	// normal shader
	shader = Shader::loadShader(normalShader,normalShader,Shader::USE_UNIFORMS);
	shader->attachShaderObject(ShaderObjectInfo::loadVertex(helperShader));
	shader->attachShaderObject(ShaderObjectInfo::loadFragment(helperShader));
	shaders.push_back(shader);

	// color shader
	shader = Shader::loadShader(colorShader,colorShader,Shader::USE_UNIFORMS);
	shader->attachShaderObject(ShaderObjectInfo::loadVertex(helperShader));
	shader->attachShaderObject(ShaderObjectInfo::loadFragment(helperShader));
	shaders.push_back(shader);

	// size shader
	shader = Shader::loadShader(sizeShader,sizeShader,Shader::USE_UNIFORMS);
	shader->attachShaderObject(ShaderObjectInfo::loadVertex(helperShader));
	shader->attachShaderObject(ShaderObjectInfo::loadFragment(helperShader));
	shaders.push_back(shader);
}

using std::placeholders::_1;
void Preprocessor::process(FrameContext& frameContext, Node* root) {
	std::function<void(Node*)> visit = std::bind(&Preprocessor::visitNode, this, std::ref(frameContext), _1);
	forEachNodeBottomUp(root, checkProcessing, visit);
}

void Preprocessor::updateSurfels(Node* node, const std::function<bool(Node*)>& abortFn) {

}

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_EXT_THESISSASCHA */
