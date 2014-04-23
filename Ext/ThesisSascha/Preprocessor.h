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

#include <Util/References.h>
#include <Util/IO/FileName.h>

#include <Geometry/Vec2.h>
#include <Geometry/Vec3.h>
#include <Geometry/Matrix4x4.h>

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

class Preprocessor {
public:
	Preprocessor();
	virtual ~Preprocessor();

	void initialize(const Util::FileName& helperShader, const Util::FileName& positionShader, const Util::FileName& normalShader, const Util::FileName& colorShader, const Util::FileName& sizeShader);
	void process(FrameContext& frameContext, Node* root);
private:
	std::pair<Util::Reference<Rendering::Mesh>,float> createSurfelsForNode(FrameContext& frameContext, Node* node);
	void visitNode(FrameContext& frameContext, Node* node);

	std::vector<Util::Reference<Rendering::Shader> > shaders;
	std::vector<Util::Reference<MinSG::CameraNodeOrtho> > cameras;

	static Geometry::Vec3 directions[];

	float verticalResolution;

	BlueSurfels::SurfelGenerator* surfelGenerator;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_PREPROCESSOR_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
