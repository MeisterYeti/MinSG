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

#include <functional>

namespace MinSG {
class Node;
namespace ThesisSascha {
class SurfelManager;

class Renderer : public NodeRendererState {
	PROVIDES_TYPE_NAME(Renderer)
public:
	Renderer(SurfelManager* manager, Util::StringIdentifier channel = FrameContext::DEFAULT_CHANNEL);
	virtual ~Renderer();

	virtual NodeRendererResult displayNode(FrameContext & context, Node * node, const RenderParam & rp);

	virtual State * clone() const;

	void setTransitionStartFn(const std::function<float(Node*)>& function) { transitionStart = function; }
	void setTransitionEndFn(const std::function<float(Node*)>& function) { transitionEnd = function; }
	void setCountFn(const std::function<uint32_t(Node*,float,uint32_t,float)>& function) { countFn = function; }
	void setSizeFn(const std::function<float(Node*,float,uint32_t,float)>& function) { sizeFn = function; }
private:
	Util::Reference<SurfelManager> manager;

	std::function<float(Node*)> transitionStart;
	std::function<float(Node*)> transitionEnd;
	std::function<uint32_t(Node*,float,uint32_t,float)> countFn;
	std::function<float(Node*,float,uint32_t,float)> sizeFn;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_RENDERER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
