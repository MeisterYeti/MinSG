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
#include "Definitions.h"

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
#include <Rendering/Mesh/MeshVertexData.h>
#include <Rendering/Mesh/VertexDescription.h>
#include <Rendering/Serialization/Serialization.h>

#include <MinSG/Core/RenderParam.h>
#include <MinSG/Core/Nodes/Node.h>
#include <MinSG/Core/Nodes/GroupNode.h>
#include <MinSG/Core/Nodes/GeometryNode.h>
#include <MinSG/Core/Nodes/CameraNodeOrtho.h>
#include <MinSG/Helper/StdNodeVisitors.h>
#include <MinSG/Ext/BlueSurfels/SurfelGenerator.h>

#include <Util/Graphics/PixelAccessor.h>
#include <Util/GenericAttribute.h>
#include <Util/StringUtils.h>
#include <Util/Timer.h>

#include <string>
#include <functional>
#include <algorithm>

#define COLOR 0
#define POSITION 1
#define NORMAL 2
#define SIZE 3

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;

typedef std::function<NodeVisitor::status (Node *,uint32_t)> VisitNodeStatusFn_t;
typedef std::function<void (Node *,uint32_t)> VisitNodeFn_t;

void forEachNodeBottomUp(Node * root, const VisitNodeStatusFn_t& enterFun, const VisitNodeFn_t& leaveFun) {
	if(root == nullptr) {
		return;
	}
	struct Visitor : public NodeVisitor {
		const VisitNodeStatusFn_t& m_enter;
		const VisitNodeFn_t& m_leave;
		uint32_t level;
		Visitor(const VisitNodeStatusFn_t& p_enter, const VisitNodeFn_t& p_leave) : m_enter(p_enter), m_leave(p_leave), level(0) {
		}
		virtual ~Visitor() = default;

		NodeVisitor::status enter(Node * node) override {
			auto result = m_enter(node, level);
			auto * groupNode = dynamic_cast<GroupNode *>(node);
			if(groupNode != nullptr) {
				++level;
			}
			return result;
		}

		NodeVisitor::status leave(Node * node) override {
			auto * groupNode = dynamic_cast<GroupNode *>(node);
			if(groupNode != nullptr) {
				--level;
			}
			m_leave(node, level);
			return CONTINUE_TRAVERSAL;
		}
	} visitor(enterFun, leaveFun);
	root->traverse(visitor);
}

Geometry::Vec3 Preprocessor::directions[8] = {
	Geometry::Vec3(1,1,1).normalize(),
	Geometry::Vec3(1,1,-1).normalize(),
	Geometry::Vec3(1,-1,1).normalize(),
	Geometry::Vec3(1,-1,-1).normalize(),
	Geometry::Vec3(-1,1,1).normalize(),
	Geometry::Vec3(-1,1,-1).normalize(),
	Geometry::Vec3(-1,-1,1).normalize(),
	Geometry::Vec3(-1,-1,-1).normalize(),
};

void updateEnclosingOrthoCam(CameraNodeOrtho* camera, const Geometry::Vec3& worldDir, Node* node) {
	Geometry::Vec3 nodeCenter = node->getWorldBB().getCenter();

	camera->setWorldPosition(nodeCenter - worldDir * node->getWorldBB().getExtentMax());
	camera->rotateToWorldDir(camera->getWorldPosition() - nodeCenter);

	Geometry::Frustum frustum = Geometry::calcEnclosingOrthoFrustum(node->getBB(), camera->getWorldMatrix().inverse() * node->getWorldMatrix());
	camera->setNearFar(frustum.getNear()*0.99f, frustum.getFar()*1.01f);
	if( frustum.getRight()-frustum.getLeft() > frustum.getTop() - frustum.getBottom()) {
		camera->rotateLocal_deg(90, Geometry::Vec3(0,0,1));
		camera->setClippingPlanes(frustum.getBottom(), frustum.getTop(), frustum.getLeft(), frustum.getRight());
	} else {
		camera->setClippingPlanes(frustum.getLeft(), frustum.getRight(), frustum.getBottom(), frustum.getTop());
	}
}

NodeVisitor::status generateId(Node * node, uint32_t level) {
	if(node->isInstance())
		node = node->getPrototype();
	if(!node->findAttribute(SURFEL_ID)) {
		node->setAttribute(SURFEL_ID, GenericAttribute::createString(StringUtils::createRandomString(32)));
	}
	if(!node->findAttribute(NODE_LEVEL)) {
		node->setAttribute(NODE_LEVEL, GenericAttribute::createNumber(level));
	} else {
		dynamic_cast<_NumberAttribute<uint32_t>*>(node->getAttribute(NODE_LEVEL))->set(level);
	}
	return NodeVisitor::CONTINUE_TRAVERSAL;
}

uint32_t countComplexity(Node * root) {
	uint32_t complexity = 0;
	forEachNodeTopDown<Node>(root,[&complexity](Node* node){
		if(node->findAttribute(NODE_COMPLEXITY)) {
			if(node->isInstance())
				node = node->getPrototype();
			complexity += node->findAttribute(NODE_COMPLEXITY)->toUnsignedInt();
			return;
		}
		//TODO: external nodes?
		complexity += node->findAttribute(MESH_COMPLEXITY) ? node->findAttribute(MESH_COMPLEXITY)->toUnsignedInt() : 0;
		GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
		if(geometry != nullptr) {
			Mesh* mesh = geometry->getMesh();
			if(mesh != nullptr)
				complexity += mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		}
		if(node->isInstance())
			node = node->getPrototype();
		node->setAttribute(NODE_COMPLEXITY, GenericAttribute::createNumber(complexity));

	});
	return complexity;
}

Preprocessor::Preprocessor(SurfelManager* manager) : verticalResolution(256), surfelGenerator(new BlueSurfels::SurfelGenerator()), manager(manager), nodeCount(0), processed(0) {
	// TODO: parameterize
	surfelGenerator->setReusalRate(0.9);
}

Preprocessor::~Preprocessor() {
	delete surfelGenerator;
}

Preprocessor::SurfelTextures_t Preprocessor::renderSurfelTexturesForNode(FrameContext& frameContext, Node* node) {

	RenderingContext& rc = frameContext.getRenderingContext();
	static Util::Timer timer;
	timer.reset();

	// initialize cameras for each direction
	Geometry::Vec2i resolution;
	{
		float maxWidth = 0;
		float maxHeight = 0;
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
		std::cout << "Res: " << resolution << " ";
	}
	std::cout << "init: " << timer.getMilliseconds() << " ";
	timer.reset();

	// render textures
	SurfelTextures_t textures;
	{
		Reference<Texture> depthT = TextureUtils::createDepthTexture(resolution.x(), resolution.y());
		textures.push_back(TextureUtils::createHDRTexture(resolution.x(), resolution.y(), true));
		textures.push_back(TextureUtils::createHDRTexture(resolution.x(), resolution.y(), true));
		textures.push_back(TextureUtils::createHDRTexture(resolution.x(), resolution.y(), true));
		textures.push_back(TextureUtils::createStdTexture(resolution.x(), resolution.y(), true));

		Reference<FBO> fbo(new FBO());
		rc.pushAndSetFBO(fbo.get());
		fbo->attachDepthTexture(rc, depthT.get());
		fbo->attachColorTexture(rc, textures[COLOR].get(), 0);
		fbo->attachColorTexture(rc, textures[POSITION].get(), 1);
		fbo->attachColorTexture(rc, textures[NORMAL].get(), 2);
		fbo->setDrawBuffers(3);
		Geometry::Rect_i screenRect(0,0,resolution.x(), resolution.y());
		Color4f clearColor(0,0,0,0);
		RenderParam rp(RenderFlags::USE_WORLD_MATRIX);

		static const StringIdentifier EYESPACE_CONV_MATRIX("sg_mrt_eyeSpaceConversionMatrix");


		Geometry::Matrix4x4 inverseModelMatrix = node->getWorldMatrix().inverse();

		rc.pushAndSetShader(mrtShader.get());
		rc.pushAndSetScissor(ScissorParameters(screenRect));
		rc.pushViewport();
		rc.setViewport(screenRect);
		rc.clearScreenRect(screenRect, clearColor, true);
		for(auto camera : cameras) {
			Geometry::Matrix4x4 m = inverseModelMatrix * camera->getWorldMatrix();
			rc.setGlobalUniform(Uniform(EYESPACE_CONV_MATRIX, m));
			frameContext.setCamera(camera.get());
			frameContext.displayNode(node, rp);
		}
		rc.popViewport();
		rc.popScissor();
		rc.popShader();

		fbo->detachColorTexture(rc,1);
		fbo->detachColorTexture(rc,2);
		fbo->detachDepthTexture(rc);
		fbo->attachColorTexture(rc, textures[SIZE].get(), 0);
		fbo->setDrawBuffers(1);

		rc.pushAndSetShader(sizeShader.get());
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
		//FileName file("test.png");
		//Serialization::saveTexture(rc, textures[SIZE].get(), file);
	}
	std::cout << "render: " << timer.getMilliseconds() << std::endl;
	return textures;
}

void Preprocessor::buildAndStoreSurfels(FrameContext& frameContext, const SurfelTextures_t& textures, Node* node, bool async) {
	static Util::Timer timer;
	RenderingContext& rc = frameContext.getRenderingContext();
	Util::Reference<Util::PixelAccessor> pos = TextureUtils::createColorPixelAccessor(rc, textures[POSITION].get());
	Util::Reference<Util::PixelAccessor> normal = TextureUtils::createColorPixelAccessor(rc, textures[NORMAL].get());
	Util::Reference<Util::PixelAccessor> color = TextureUtils::createColorPixelAccessor(rc, textures[COLOR].get());
	Util::Reference<Util::PixelAccessor> size = TextureUtils::createColorPixelAccessor(rc, textures[SIZE].get());
	if(async) {
		manager->executeAsync([=] () {
			timer.reset();
			std::cout << "building surfels... " << std::endl;
			SurfelInfo_t surfels = surfelGenerator->createSurfels(*pos.get(), *normal.get(), *color.get(), *size.get());
			std::cout << "done. Time: " << timer.getMilliseconds() << std::endl;
			std::function<void()> storeSurfel = std::bind(&SurfelManager::storeSurfel, manager.get(), node, surfels, async);
			manager->executeOnMainThread(storeSurfel);
		});
	} else {
		timer.reset();
		std::cout << "building surfels... " << std::endl;
		SurfelInfo_t surfels = surfelGenerator->createSurfels(*pos.get(), *normal.get(), *color.get(), *size.get());
		std::cout << "done. Time: " << timer.getMilliseconds() << std::endl;
		manager->storeSurfel(node, surfels, async);
	}
}

void Preprocessor::visitNode(FrameContext& frameContext, Node* node, uint32_t level, bool async) {
	Node* proto = node->isInstance() ? node->getPrototype() : node;
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
	if(geometry != nullptr && geometry->getMesh()->getVertexCount()>0) {
		// externalize meshes
		manager->storeMesh(geometry, async);
	}
	if(node->findAttribute(SURFELS)) {
		ReferenceAttribute<Mesh>* attr = node->findAttribute(SURFELS)->toType<ReferenceAttribute<Mesh>>();
		if(attr != nullptr) {
			float coverage = node->findAttribute(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
			manager->storeSurfel(node,{attr->get(), coverage},async);
			proto->unsetAttribute(SURFELS);
			std::cout << "Attached Surfels found." << std::endl;
		}
	} else if(node->findAttribute(SURFEL_ID) && countComplexity(node) > surfelGenerator->getMaxAbsSurfels()) {
		SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
		buildAndStoreSurfels(frameContext, textures, node, async);
	}
	std::cout << "Progress: " << (static_cast<float>(++processed)/nodeCount) << std::endl;
}

void Preprocessor::initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader) {
	cameras.clear();

	// Create cameras
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());
	cameras.push_back(new CameraNodeOrtho());

	this->mrtShader = mrtShader;
	this->sizeShader = sizeShader;
}

using std::placeholders::_1;
using std::placeholders::_2;
void Preprocessor::process(FrameContext& frameContext, Node* root, bool async) {
	{
		auto nodes = collectNodes(root);
		processed = 0;
		nodeCount = nodes.size();
	}
	VisitNodeFn_t visit = std::bind(&Preprocessor::visitNode, this, std::ref(frameContext), _1, _2, async);
	forEachNodeBottomUp(root, [this] (Node* node, uint32_t level) {
		if(countComplexity(node) > surfelGenerator->getMaxAbsSurfels())
			return NodeVisitor::CONTINUE_TRAVERSAL;
		return generateId(node, level);
	}, visit);
	forEachNodeTopDown(root, [] (Node* node) {
		if(node->isInstance())
			node = node->getPrototype();
		node->unsetAttribute(NODE_COMPLEXITY);
	});
}

void Preprocessor::updateSurfels(FrameContext& frameContext, Node* node, float coverage, bool async) {
	//TODO: return updated nodes?
	if(!node->findAttribute(SURFEL_ID)) {
		WARN("Could not update surfels for node. Missing surfel id.");
		return;
	}

	SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
	buildAndStoreSurfels(frameContext, textures, node, async);
	//TODO: recalculate coverage
	coverage = node->findAttribute(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
	/*if(dynamic_cast<GeometryNode*>(node) != nullptr) {
		SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
		buildAndStoreSurfels(frameContext, textures, node, async);
		coverage = node->isAttributeSet(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
	} else {
		std::deque<Node*> children = getChildNodes(node);
		std::deque<SurfelInfo_t> surfels;
		for(auto child : children) {
			while(manager->loadSurfel(child,9999,0,false) == SurfelManager::Pending) {
				manager->update();
			}
			if(manager->loadSurfel(child,9999,0,false) == SurfelManager::Success) {
				float relCov = child->isAttributeSet(SURFEL_REL_COVERING) ? child->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
				surfels.push_back({manager->getSurfel(child), relCov});
			}
		}
		Mesh* mesh = combineSurfelMeshes(surfels, node->isAttributeSet(SURFEL_COUNT) ? node->findAttribute(SURFEL_COUNT)->toUnsignedInt() :10000);
		//TODO: calculate coverage
		coverage = node->isAttributeSet(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
		if(mesh != nullptr) {
			manager->storeSurfel(node, {mesh, coverage}, async);
		} else {
			SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
			buildAndStoreSurfels(frameContext, textures, node, async);
			coverage = node->isAttributeSet(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
		}
	}*/

	if(node->hasParent() && !abortUpdate(node, coverage)) {
		updateSurfels(frameContext, node->getParent(), coverage, async);
	}
}

Rendering::Mesh* Preprocessor::combineSurfelMeshes(const std::deque<SurfelInfo_t>& meshes, uint32_t targetSize) {
	if(meshes.empty())
		return nullptr;

	const SurfelInfo_t& first = meshes.front();

	// Assuming all meshes have the same vertex description
	const VertexDescription & vd = first.first->getVertexDescription();

	// compute weights
	std::vector<float> weights;
	uint32_t surfelCount = 0;
	for(auto sInfo : meshes) {
		uint32_t c = sInfo.first->isUsingIndexData() ? sInfo.first->getIndexCount() : sInfo.first->getVertexCount();
		surfelCount += c;
		weights.push_back(c);
	}
	for(uint32_t i = 0; i<weights.size(); ++i) {
		weights[i] /= surfelCount;
	}

	uint32_t vertexCount = std::min(surfelCount, targetSize);

	uint32_t weightedSum = 0;
	for(uint32_t i = 0; i<weights.size(); ++i) {
		weights[i] /= surfelCount;
	}

	// create mesh
	Mesh* mesh = new Mesh;
	MeshVertexData & vertices = mesh->openVertexData();
	vertices.allocate(vertexCount, vd);

	uint32_t vertexPointer = 0;
	std::vector<uint32_t> pointers(meshes.size(),0);
	uint32_t currentCount = 0;
	uint32_t vdSize = vd.getVertexSize();
	while(currentCount < vertexCount) {
		uint32_t i = 0;
		bool fail = true;
		for(auto sInfo : meshes) {
			MeshVertexData & currentVertices = sInfo.first->openVertexData();
			uint32_t vCount = currentVertices.getVertexCount();
			uint32_t max = std::min(vCount, static_cast<uint32_t>(std::ceil(vertexCount*weights[i])));
			if(pointers[i] >= max) {
				fail = false;
				continue;
			}

			std::copy(currentVertices.data() + pointers[i], currentVertices.data() + pointers[i] + vdSize, vertices[vertexPointer]);
			++pointers[i];
			++currentCount;
			vertexPointer += vdSize;
		}
		if(currentCount < vertexCount && fail) {
			WARN("combineSurfelMeshes failed!"); //TODO: better error description
			delete mesh;
			return nullptr;
		}
	}
	// TODO: calculate combined coverage
	return mesh;
}

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_EXT_THESISSASCHA */
