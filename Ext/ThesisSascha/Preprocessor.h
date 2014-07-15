/*
	This file is part of the MinSG library extension ThesisSascha.
	Copyright (C) 2014 Sascha Brandt <myeti@mail.uni-paderborn.de>

	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifdef MINSG_EXT_THESISSASCHA

#ifndef MINSG_THESISSASCHA_PREPROCESSOR_H_
#define MINSG_THESISSASCHA_PREPROCESSOR_H_

#include <MinSG/Core/FrameContext.h>
#include <MinSG/Core/NodeVisitor.h>

#include <Util/References.h>
#include <Util/ReferenceCounter.h>
#include <Util/IO/FileName.h>
#include <Util/TypeNameMacro.h>
#include <Util/AttributeProvider.h>

#include <Geometry/Vec2.h>
#include <Geometry/Vec3.h>
#include <Geometry/Matrix4x4.h>

#include <functional>
#include <vector>

namespace Rendering {
class Shader;
class Texture;
class Mesh;
}

namespace MinSG {
class Node;
class CameraNodeOrtho;

namespace BlueSurfels {
class SurfelGenerator;
}

namespace ThesisSascha {
class SurfelManager;

class Preprocessor : public Util::AttributeProvider, public Util::ReferenceCounter<Preprocessor> {
	PROVIDES_TYPE_NAME(Preprocessor)
public:
	typedef std::pair<Util::Reference<Rendering::Mesh>,float> SurfelInfo_t;
	typedef std::vector<Util::Reference<Rendering::Texture> > SurfelTextures_t;

	Preprocessor(SurfelManager* manager);
	virtual ~Preprocessor();

	void initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader);

	void process(FrameContext& frameContext, Node* root, bool async = true);

	void updateSurfels(FrameContext& frameContext, Node* node, float coverage=1.0f, bool async = true);

	void setAbortUpdateFn(const std::function<bool(Node*,float)> & function) { this->abortUpdate = function; }

	void setUpdateProgressFn(const std::function<void(uint32_t,uint32_t)> & function) { this->updateProgress = function; }

	uint32_t getMaxAbsSurfels()const;
	float getReusalRate()const;
	void setMaxAbsSurfels(uint32_t i);
	void setReusalRate(float f);
	void setMaxComplexity(uint32_t value) { maxComplexity = value; }

	Rendering::Mesh* generateMeshFromSurfels(FrameContext& frameContext, Node* node);
	Rendering::Mesh* generateMeshFromSurfels(FrameContext& frameContext, Rendering::Mesh* surfelMesh, bool inplace);
private:
	class InternalRenderer;

	SurfelTextures_t renderSurfelTexturesForNode(FrameContext& frameContext, Node* node);
	void buildAndStoreSurfels(FrameContext& frameContext, const SurfelTextures_t& textures, Node* node, bool async);
	void visitNode(FrameContext& frameContext, Node* node, uint32_t level, bool async);
	Rendering::Mesh* combineSurfelMeshes(const std::deque<SurfelInfo_t>& meshes, uint32_t targetSize);

	Util::Reference<Rendering::Shader> mrtShader;
	Util::Reference<Rendering::Shader> sizeShader;
	std::vector<Util::Reference<MinSG::CameraNodeOrtho> > cameras;

	static Geometry::Vec3 directions[];

	float verticalResolution;

	BlueSurfels::SurfelGenerator* surfelGenerator;
	Util::Reference<SurfelManager> manager;
	Util::Reference<InternalRenderer> internalRenderer;

	std::function<bool(Node*,float)> abortUpdate;

	std::function<void(uint32_t,uint32_t)> updateProgress;

	uint32_t processed;
	uint32_t nodeCount;
	uint32_t maxComplexity;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_PREPROCESSOR_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
