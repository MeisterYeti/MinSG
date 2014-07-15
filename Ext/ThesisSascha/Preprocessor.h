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
#include <memory>

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

	Preprocessor(SurfelManager* manager);
	virtual ~Preprocessor();

	void initShaders(Rendering::Shader* mrtShader, Rendering::Shader* sizeShader);

	void process(FrameContext& frameContext, Node* root, bool async = true, bool dryRun = false);

	void updateSurfels(FrameContext& frameContext, Node* node, float coverage=1.0f, bool async = true);

	void setAbortUpdateFn(const std::function<bool(Node*,float)> & function);

	void setUpdateProgressFn(const std::function<void(uint32_t,uint32_t)> & function);
	
	uint32_t getMaxAbsSurfels()const;
	float getReusalRate()const;
	void setMaxAbsSurfels(uint32_t i);
	void setReusalRate(float f);
	void setMaxComplexity(uint32_t value);

	void setVerticalResolution(float f);
	
	Rendering::Mesh* generateMeshFromSurfels(FrameContext& frameContext, Node* node);

	Util::GenericAttributeMap * getStats() const;
private:
	class Implementation;
	std::unique_ptr<Implementation> impl;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_PREPROCESSOR_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
