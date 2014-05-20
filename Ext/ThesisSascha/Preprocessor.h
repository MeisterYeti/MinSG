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

	void initShaders(const Util::FileName& helperShader, const Util::FileName& positionShader, const Util::FileName& normalShader, const Util::FileName& colorShader, const Util::FileName& sizeShader);

	void process(FrameContext& frameContext, Node* root, bool async = true);

	void updateSurfels(FrameContext& frameContext, Node* node, float coverage=1.0f, bool async = true);

	void setPrepareNodeFn(const std::function<NodeVisitor::status(Node*)>& function) { this->prepareNode = function; }
	void setAbortUpdateFn(const std::function<bool(Node*,float)> & function) { this->abortUpdate = function; }
private:
	SurfelTextures_t renderSurfelTexturesForNode(FrameContext& frameContext, Node* node);
	void buildAndStoreSurfels(FrameContext& frameContext, const SurfelTextures_t& textures, Node* node, bool async);
	void visitNode(FrameContext& frameContext, Node* node, bool async);

	std::vector<Util::Reference<Rendering::Shader> > shaders;
	std::vector<Util::Reference<MinSG::CameraNodeOrtho> > cameras;

	static Geometry::Vec3 directions[];

	float verticalResolution;

	BlueSurfels::SurfelGenerator* surfelGenerator;
	Util::WeakPointer<SurfelManager> manager;

	std::function<NodeVisitor::status(Node*)> prepareNode;
	std::function<bool(Node*,float)> abortUpdate;

	uint32_t processed;
	uint32_t nodeCount;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_PREPROCESSOR_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
