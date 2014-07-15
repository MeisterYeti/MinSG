/*
	This file is part of the MinSG library extension ThesisSascha.
	Copyright (C) 2014 Sascha Brandt <myeti@mail.uni-paderborn.de>

	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifdef MINSG_EXT_THESISSASCHA

#include "Definitions.h"
#include "Preprocessor.h"
#include "SurfelManager.h"
#include "Renderer.h"

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
#include <MinSG/Core/States/NodeRendererState.h>

#include <Util/Graphics/PixelAccessor.h>
#include <Util/GenericAttribute.h>
#include <Util/StringUtils.h>
#include <Util/Timer.h>
#include <Util/Utils.h>
#include <Util/Concurrency/Concurrency.h>
#include <Util/Concurrency/Mutex.h>
#include <Util/Concurrency/Lock.h>

#include <string>
#include <functional>
#include <algorithm>
#include <stdexcept>
#include <atomic>
#include <deque>
#include <array>

#define COLOR 0
#define POSITION 1
#define NORMAL 2
#define SIZE 3

#define MAX_DEPTH 2

namespace MinSG {
namespace ThesisSascha {

using namespace Rendering;
using namespace Util;

/******************************************
 * helper functions
 ******************************************/

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
	} else {
		// do not regenerate surfels
		node->setAttribute(NODE_HANDLED, GenericAttribute::createBool(true));
	}
	return NodeVisitor::CONTINUE_TRAVERSAL;
}

/******************************************
 * Preprocessor Implementation class
 ******************************************/
class Preprocessor::Implementation {
public:
	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	typedef std::vector<Util::Reference<Rendering::Texture> > SurfelTextures_t;

	Implementation(SurfelManager* manager);
	~Implementation() = default;
	
	uint32_t countComplexity(Node * root);
	void initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader);
	void process(FrameContext& frameContext, Node* root, bool async, bool dryRun);
	void updateSurfels(FrameContext& frameContext, Node* node, float coverage, bool async);

	Rendering::Mesh* generateMeshFromSurfels(FrameContext& frameContext, Node* node);

//private:
	class InternalRenderer;

	void updateProgressInternal(uint32_t processed, uint32_t nodeCount);
	SurfelTextures_t renderSurfelTexturesForNode(FrameContext& frameContext, Node* node);
	void buildAndStoreSurfels(FrameContext& frameContext, const SurfelTextures_t& textures, Node* node, bool async);
	void visitNode(FrameContext& frameContext, Node* node, uint32_t level, bool async);
	Rendering::Mesh* combineSurfelMeshes(const std::deque<SurfelInfo_t>& meshes, uint32_t targetSize);

	Util::Reference<Rendering::Shader> mrtShader;
	Util::Reference<Rendering::Shader> sizeShader;
	std::vector<Util::Reference<MinSG::CameraNodeOrtho> > cameras;

	static Geometry::Vec3 directions[];

	float verticalResolution;

	std::unique_ptr<BlueSurfels::SurfelGenerator> surfelGenerator;
	Util::Reference<SurfelManager> manager;
	Util::Reference<InternalRenderer> internalRenderer;

	std::function<bool(Node*,float)> abortUpdate;

	std::function<void(uint32_t,uint32_t)> updateProgress;

	uint32_t processed;
	uint32_t nodeCount;
	uint32_t maxComplexity;
	std::atomic<uint32_t> inProgress;

	std::unique_ptr<Util::GenericAttributeMap> stats;
	std::unique_ptr<Concurrency::Mutex> mutex;
};


/******************************************
 * internal renderer class
 ******************************************/
class Preprocessor::Implementation::InternalRenderer : public NodeRendererState {
public:
	InternalRenderer(Preprocessor::Implementation* p, Util::StringIdentifier channel = FrameContext::DEFAULT_CHANNEL) : NodeRendererState(channel), processor(p), root(nullptr) {}
	virtual ~InternalRenderer() = default;

	virtual NodeRendererResult displayNode(FrameContext & context, Node * node, const RenderParam & rp);

	virtual State * clone() const { return new InternalRenderer(processor.get()); };
protected:
	stateResult_t doEnableState(FrameContext & context, Node * node, const RenderParam & rp) override;
	void doDisableState(FrameContext & context, Node * node, const RenderParam & rp) override;
private:
	WeakPointer<Preprocessor::Implementation> processor;
	Node* root;
};
/******************************************
 * Preprocessor Implementation
 ******************************************/

Geometry::Vec3 Preprocessor::Implementation::directions[8] = {
	Geometry::Vec3(1,1,1).normalize(),
	Geometry::Vec3(1,1,-1).normalize(),
	Geometry::Vec3(1,-1,1).normalize(),
	Geometry::Vec3(1,-1,-1).normalize(),
	Geometry::Vec3(-1,1,1).normalize(),
	Geometry::Vec3(-1,1,-1).normalize(),
	Geometry::Vec3(-1,-1,1).normalize(),
	Geometry::Vec3(-1,-1,-1).normalize(),
};


Preprocessor::Implementation::Implementation(SurfelManager* manager) :
		verticalResolution(256), surfelGenerator(new BlueSurfels::SurfelGenerator()), manager(manager), processed(0), nodeCount(0),
		maxComplexity(10000), inProgress(0), stats(new GenericAttributeMap()), mutex(Concurrency::createMutex()) {
	// TODO: parameterize
	surfelGenerator->setReusalRate(0.9);
	internalRenderer = new InternalRenderer(this);
	internalRenderer->setTempState(true);
	updateProgress = [](uint32_t processed, uint32_t nodeCount) {
		std::cout << "Progress: " << (static_cast<float>(processed)/nodeCount) << std::endl;
	};
}

uint32_t Preprocessor::Implementation::countComplexity(Node * root) {
	uint32_t complexity = 0;
	traverseTopDown<Node>(root,[this, &complexity](Node* node){
		if(node->isInstance())
			node = node->getPrototype();

		if(node->findAttribute(NODE_COMPLEXITY)) {
			complexity += node->findAttribute(NODE_COMPLEXITY)->toUnsignedInt();
			return NodeVisitor::BREAK_TRAVERSAL;
		}
		//TODO: external nodes?
		complexity += node->findAttribute(MESH_COMPLEXITY) ? node->findAttribute(MESH_COMPLEXITY)->toUnsignedInt() : 0;
		GeometryNode* geometry = dynamic_cast<GeometryNode*>(node);
		if(node->findAttribute(MESH_ID)) {
			// force mesh loading
			Reference<Mesh> mesh;
			if( manager->fetchMesh(mesh,geometry,9999,0,false, true) == SurfelManager::Success) {
				if(mesh.isNotNull()) {
					complexity += mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
				}
			}
		} else if(geometry != nullptr) {
			Mesh* mesh = geometry->getMesh();
			if(mesh != nullptr)
				complexity += mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();				
		}
		return NodeVisitor::CONTINUE_TRAVERSAL;
	});
	root->setAttribute(NODE_COMPLEXITY, GenericAttribute::createNumber(complexity));
	return complexity;
}


void Preprocessor::Implementation::initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader) {
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
void Preprocessor::Implementation::process(FrameContext& frameContext, Node* root, bool async, bool dryRun) {

	stats->clear();
	processed = 0;
	nodeCount = 0;
	uint32_t maxLevel = 0;

	// Count and initialize nodes
	forEachNodeBottomUp(root, [&] (Node* node, uint32_t level) {
		node->setAttribute(NODE_LEVEL, GenericAttribute::createNumber(level));
		++nodeCount;
		maxLevel = std::max(level, maxLevel);
		return NodeVisitor::CONTINUE_TRAVERSAL;
	}, [&] (Node* node, uint32_t level) {
		countComplexity(node);
	});


	static Timer timer;
	timer.reset();

	typedef std::deque<Node*> LevelQueue_t;
	std::vector<LevelQueue_t> levels(maxLevel+1);

	uint32_t todo = 0;
	
	//VisitNodeFn_t visit = std::bind(&Preprocessor::Implementation::visitNode, this, std::ref(frameContext), _1, _2, async);
	forEachNodeTopDown(root, [&](Node* node){
		uint32_t complexity = node->getAttribute(NODE_COMPLEXITY)->toUnsignedInt();
		if(complexity >= maxComplexity) {
			++todo;
			uint32_t level = node->getAttribute(NODE_LEVEL)->toUnsignedInt();
			levels[level].push_back(node);
		}
	});
	
	uint32_t maxJobs = 100;
	uint32_t jobs = 0;
	inProgress = 0;
	
	updateProgressInternal(processed, todo);
	
	// traverse tree level by level (bottom up) to increase parallelizability
	for(int32_t i=maxLevel; i>=0; --i) {
		while(!levels[i].empty()) {
			++inProgress;
			if(!dryRun)
				visitNode(frameContext, levels[i].front(), i, async);
			levels[i].pop_front();
			manager->update();
			if(inProgress > maxJobs) {
				std::cout << "flushing jobs..." << std::endl;
				manager->flush();
			}
			updateProgressInternal(processed, todo);
		}
		manager->flush();
	}

	manager->clear();
	LOG_STAT(TreeNodes, nodeCount);
	LOG_STAT(Depth, maxLevel);
	LOG_STAT(Processed, processed);
	LOG_STAT(Skipped, nodeCount-processed);
	LOG_STAT(ProcessTime, timer.getMilliseconds());
	LOG_STAT(Complexity, countComplexity(root));
}

void Preprocessor::Implementation::updateSurfels(FrameContext& frameContext, Node* node, float coverage, bool async) {
	static Timer timer;
	timer.reset();
	stats->clear();

	std::deque<Node*> todo;
	todo.push_back(node);
	while(node->hasParent()) {
		node = node->getParent();
		todo.push_back(node);
	}
	processed = 0;
	uint32_t tmp = 0;
	nodeCount = todo.size();
	updateProgressInternal(processed, nodeCount);

	while(!todo.empty()) {
		Node* current = todo.front();
		todo.pop_front();
		//++processed;
		if(current->findAttribute(SURFEL_ID)) {
			coverage = current->findAttribute(SURFEL_REL_COVERING) ? current->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
			SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, current);
			buildAndStoreSurfels(frameContext, textures, current, async);

			if(abortUpdate(current, coverage)) {
				tmp = nodeCount-processed;
				todo.clear();
			}
			manager->update();
		} else {
			++tmp;
		}
		updateProgressInternal(processed+tmp, nodeCount);
	}
	//manager->clear();

	LOG_STAT(TreeNodes, nodeCount);
	LOG_STAT(Processed, processed);
	LOG_STAT(Skipped, nodeCount-processed);
	LOG_STAT(ProcessTime, timer.getMilliseconds());
}

void Preprocessor::Implementation::updateProgressInternal(uint32_t processed, uint32_t nodeCount) {
	LOCK(*mutex);
	updateProgress(processed, nodeCount);
}

Preprocessor::Implementation::SurfelTextures_t Preprocessor::Implementation::renderSurfelTexturesForNode(FrameContext& frameContext, Node* node) {

	RenderingContext& rc = frameContext.getRenderingContext();
	static Util::Timer timer;
	timer.reset();

	// initialize cameras for each direction
	Geometry::Vec2i resolution;
	{
		float maxWidth = 0;
		float maxHeight = 0;
		auto it = cameras.begin();
		try {
		for(auto dir : Preprocessor::Implementation::directions) {
			CameraNodeOrtho* camera = (*it).get();
			updateEnclosingOrthoCam(camera, dir, node);
			maxWidth = std::max(maxWidth, camera->getRightClippingPlane()-camera->getLeftClippingPlane());
			maxHeight = std::max(maxHeight, camera->getTopClippingPlane()-camera->getBottomClippingPlane());
			++it;
		}
		} catch(std::invalid_argument& e) {
			// frustum with zero volume -> remove surfel and return
			WARN(e.what());
			node->isInstance() ? node->getPrototype()->unsetAttribute(SURFEL_ID) : node->unsetAttribute(SURFEL_ID);
			return SurfelTextures_t();
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

		internalRenderer->enableState(frameContext, node, rp);

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
		//FileName file(node->findAttribute(SURFEL_ID)->toString() + ".png");
		//Serialization::saveTexture(rc, textures[SIZE].get(), file);
		internalRenderer->disableState(frameContext, node, rp);
	}
	//rc.flush();
	std::cout << "render: " << timer.getMilliseconds() << std::endl;
	return textures;
}

void Preprocessor::Implementation::buildAndStoreSurfels(FrameContext& frameContext, const SurfelTextures_t& textures, Node* node, bool async) {
	if(textures.size() < 4)
		return;
	RenderingContext& rc = frameContext.getRenderingContext();
	Util::Reference<Util::PixelAccessor> pos = TextureUtils::createColorPixelAccessor(rc, textures[POSITION].get());
	Util::Reference<Util::PixelAccessor> normal = TextureUtils::createColorPixelAccessor(rc, textures[NORMAL].get());
	Util::Reference<Util::PixelAccessor> color = TextureUtils::createColorPixelAccessor(rc, textures[COLOR].get());
	Util::Reference<Util::PixelAccessor> size = TextureUtils::createColorPixelAccessor(rc, textures[SIZE].get());
	std::function<void()> generateSurfels = [=] () {
		static Util::Timer timer;
		timer.reset();
		std::cout << "building surfels... " << std::endl;
		SurfelInfo_t surfels = surfelGenerator->createSurfels(*pos.get(), *normal.get(), *color.get(), *size.get());
		std::cout << "done. Time: " << timer.getMilliseconds() << std::endl;

		++processed;
		--inProgress;
		std::function<void()> storeSurfel = std::bind(&SurfelManager::storeSurfel, manager.get(), node, surfels, false);
		manager->executeOnMainThread(storeSurfel);
	};

	if(async) {
		manager->executeAsync(generateSurfels);
	} else {
		generateSurfels();
	}
}

void Preprocessor::Implementation::visitNode(FrameContext& frameContext, Node* node, uint32_t level, bool async) {

	/*if(node->findAttribute(NODE_HANDLED)) {
		++processed;
		return;
	}*/
	Node* proto = node->isInstance() ? node->getPrototype() : node;
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(proto);

	/*if(geometry != nullptr && geometry->getMesh()->getVertexCount()>0) {
		// externalize meshes
		manager->storeMesh(geometry, false);
		geometry->setFixedBB(geometry->getBB());
		Mesh* mesh = geometry->getMesh();
		proto->setAttribute(MESH_COMPLEXITY, GenericAttribute::createNumber(mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount()));
		geometry->setMesh(new Mesh);
		dynamic_cast<GeometryNode*>(node)->setMesh(geometry->getMesh());
	}*/

	/*uint32_t complexity = countComplexity(node);
	if(complexity < maxComplexity) {
		updateProgressInternal(++processed, nodeCount);
		++skipped;
		//node->setAttribute(NODE_HANDLED, GenericAttribute::createBool(true));
		return;
	}*/

	generateId(node, level);

	if(node->findAttribute(SURFELS)) {
		ReferenceAttribute<Mesh>* attr = node->findAttribute(SURFELS)->toType<ReferenceAttribute<Mesh>>();
		if(attr != nullptr) {
			float coverage = node->findAttribute(SURFEL_REL_COVERING) ? node->findAttribute(SURFEL_REL_COVERING)->toFloat() : 0.5f;
			manager->storeSurfel(node,{attr->get(), coverage},false);
			proto->unsetAttribute(SURFELS);
			std::cout << "Attached Surfels found." << std::endl;
		}
		//updateProgressInternal(++processed, nodeCount);
		++processed;
	} else if(node->findAttribute(SURFEL_ID) && countComplexity(node) > maxComplexity) {
		SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
		buildAndStoreSurfels(frameContext, textures, node, async);
	}
	//proto->setAttribute(NODE_HANDLED, GenericAttribute::createBool(true));
}

Rendering::Mesh* Preprocessor::Implementation::combineSurfelMeshes(const std::deque<SurfelInfo_t>& meshes, uint32_t targetSize) {
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

Rendering::Mesh* Preprocessor::Implementation::generateMeshFromSurfels(FrameContext& frameContext, Node* node) {
	SurfelTextures_t textures = renderSurfelTexturesForNode(frameContext, node);
	if(textures.size() < 4)
		return nullptr;
	static Util::Timer timer;
	RenderingContext& rc = frameContext.getRenderingContext();
	Util::Reference<Util::PixelAccessor> pos = TextureUtils::createColorPixelAccessor(rc, textures[POSITION].get());
	Util::Reference<Util::PixelAccessor> normal = TextureUtils::createColorPixelAccessor(rc, textures[NORMAL].get());
	Util::Reference<Util::PixelAccessor> color = TextureUtils::createColorPixelAccessor(rc, textures[COLOR].get());
	Util::Reference<Util::PixelAccessor> size = TextureUtils::createColorPixelAccessor(rc, textures[SIZE].get());

	timer.reset();
	std::cout << "building surfels... " << std::endl;
	SurfelInfo_t surfels = surfelGenerator->createSurfels(*pos.get(), *normal.get(), *color.get(), *size.get());
	std::cerr << std::flush;
	std::cout << "done. Time: " << timer.getMilliseconds() << std::endl;
	return nullptr;
}

/******************************************
 * internal renderer implementation
 ******************************************/

NodeRendererResult Preprocessor::Implementation::InternalRenderer::displayNode(FrameContext & context, Node * node, const RenderParam & rp) {
	processor->manager->update();
	GeometryNode* geometry = dynamic_cast<GeometryNode*>(node->isInstance() ? node->getPrototype() : node);
	if(geometry) {
		if(geometry->getMesh() && !geometry->getMesh()->empty()) {
			return NodeRendererResult::PASS_ON;
		} else if(node->findAttribute(MESH_ID)) {
			SurfelManager::MeshLoadResult_t result;
			// force mesh loading
			Reference<Mesh> mesh;
			if( processor->manager->fetchMesh(mesh,geometry,9999,0,false, true) == SurfelManager::Success) {
				if(mesh.isNull()) {
					return NodeRendererResult::PASS_ON;
				}
				Renderer::drawMesh(context, node, rp, mesh.get() );
				return NodeRendererResult::PASS_ON;
			}
		}
	}

	// don't render root node
	if(node == root || node->findAttribute(SURFEL_ID) == nullptr) {
		return NodeRendererResult::PASS_ON;
	}

	uint32_t depth = 0;
	Node* tmp = node;
	while(tmp->hasParent() && tmp != root) {
		++depth;
		tmp = tmp->getParent();
	}

	NodeRendererResult renderResult = NodeRendererResult::PASS_ON;
	if(depth >= MAX_DEPTH) {
		renderResult = NodeRendererResult::NODE_HANDLED;
	}

	SurfelManager::MeshLoadResult_t result;
	// force mesh loading
	Reference<Mesh> mesh;
	if( processor->manager->fetchSurfel(mesh,node,9999,0,false, true) == SurfelManager::Success) {
		if(mesh.isNull()) {
			return renderResult;
		}
		uint32_t count = mesh->isUsingIndexData() ? mesh->getIndexCount() : mesh->getVertexCount();
		Renderer::drawSurfels(context, node, rp, mesh.get() , 4, count);
		return renderResult;
	}
	return renderResult;
}

State::stateResult_t Preprocessor::Implementation::InternalRenderer::doEnableState(FrameContext & context, Node * node, const RenderParam & rp) {
	root = node;
	return NodeRendererState::doEnableState(context,node,rp);
}

void Preprocessor::Implementation::InternalRenderer::doDisableState(FrameContext & context, Node * node, const RenderParam & rp) {
	root = nullptr;
	NodeRendererState::doDisableState(context,node,rp);
}

/******************************************
 * Preprocessor
 ******************************************/

Preprocessor::Preprocessor(SurfelManager* manager) : impl(new Implementation(manager)) {}
Preprocessor::~Preprocessor() {};

void Preprocessor::initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader) { impl->initShaders(mrtShader, sizeShader); }

void Preprocessor::process(FrameContext& frameContext, Node* root, bool async, bool dryRun) { impl->process(frameContext, root, async, dryRun); }

void Preprocessor::updateSurfels(FrameContext& frameContext, Node* node, float coverage, bool async) { impl->updateSurfels(frameContext, node, coverage, async); }

void Preprocessor::setAbortUpdateFn(const std::function<bool(Node*,float)> & function) { impl->abortUpdate = function; }
void Preprocessor::setUpdateProgressFn(const std::function<void(uint32_t,uint32_t)> & function) { impl->updateProgress = function; }
uint32_t Preprocessor::getMaxAbsSurfels()const	{	return impl->surfelGenerator->getMaxAbsSurfels();	}
float Preprocessor::getReusalRate()const		{	return impl->surfelGenerator->getReusalRate();	}
void Preprocessor::setMaxAbsSurfels(uint32_t i)	{	impl->surfelGenerator->setMaxAbsSurfels(i);	}
void Preprocessor::setReusalRate(float f)		{	impl->surfelGenerator->setReusalRate(f);	}
void Preprocessor::setMaxComplexity(uint32_t value) { impl->maxComplexity = value; }
void Preprocessor::setVerticalResolution(float f)		{	impl->verticalResolution = f;	}
Rendering::Mesh* Preprocessor::generateMeshFromSurfels(FrameContext& frameContext, Node* node) { return impl->generateMeshFromSurfels(frameContext, node); }

Util::GenericAttributeMap * Preprocessor::getStats() const { return impl->stats.get(); }

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_EXT_THESISSASCHA */
