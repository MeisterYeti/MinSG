/*
	This file is part of the MinSG library extension ThesisSascha.
	Copyright (C) 2014 Sascha Brandt <myeti@mail.uni-paderborn.de>

	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifdef MINSG_EXT_THESISSASCHA
#ifndef MINSG_THESISSASCHA_RENDERER_H_
#define MINSG_THESISSASCHA_RENDERER_H_

#include <MinSG/Core/States/NodeRendererState.h>
#include <MinSG/Core/FrameContext.h>

#include <Util/References.h>
#include <Util/ReferenceCounter.h>
#include <Util/TypeNameMacro.h>
#include <Util/GenericAttribute.h>

#include <functional>
#include <vector>
#include <memory>

namespace MinSG {
class Node;
namespace ThesisSascha {
class SurfelManager;

struct SortedNodeSet;

class Renderer : public NodeRendererState {
	PROVIDES_TYPE_NAME(Renderer)
public:
	Renderer(SurfelManager* manager, Util::StringIdentifier channel = FrameContext::DEFAULT_CHANNEL);
	virtual ~Renderer();

	virtual NodeRendererResult displayNode(FrameContext & context, Node * node, const RenderParam & rp);

	virtual State * clone() const;

	Util::GenericAttributeMap * getStats() const;

	//void setCountFn(const std::function<uint32_t(Node*,float,uint32_t,float)>& function) { countFn = function; }
	//void setSizeFn(const std::function<float(Node*,float,uint32_t,float)>& function) { sizeFn = function; }
	//void setRefineFn(const RefineNodeFn_t& function) { refineNodeFn = function; }
	//void setAsync(bool async) { this->async = async; }
	//bool isAsync() { return this->async; }
	//void setImmediate(bool immediate) { this->immediate = immediate; }
	//bool isImmediate() { return this->immediate; }
	void setPointSizeFactor(float value);
	void setMinProjSize(float value);

	static void drawMesh(FrameContext& context, Node* node, const RenderParam& rp, Rendering::Mesh* mesh);
	static void drawSurfels(FrameContext& context, Node* node, const RenderParam& rp, Rendering::Mesh* mesh, float pSize, uint32_t count);
protected:
	stateResult_t doEnableState(FrameContext & context, Node *, const RenderParam & rp) override;
	void doDisableState(FrameContext & context, Node * node, const RenderParam & rp) override;
	bool doDisplayNode(FrameContext & context, Node * node, const RenderParam & rp, bool force=false);
private:
	class Implementation;
	std::unique_ptr<Implementation> impl;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_RENDERER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
