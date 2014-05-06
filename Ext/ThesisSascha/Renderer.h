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

namespace MinSG {
namespace ThesisSascha {
class SurfelManager;

class Renderer : public NodeRendererState {
	PROVIDES_TYPE_NAME(Renderer)
public:
	Renderer(SurfelManager* manager, Util::StringIdentifier channel = FrameContext::DEFAULT_CHANNEL);
	virtual ~Renderer();

	virtual NodeRendererResult displayNode(FrameContext & context, Node * node, const RenderParam & rp);

	virtual State * clone() const;
private:

	Util::Reference<SurfelManager> manager;
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_RENDERER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
