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

#include <Util/ReferenceCounter.h>
#include <Util/TypeNameMacro.h>

namespace MinSG {
namespace ThesisSascha {

class Renderer : public Util::ReferenceCounter<Renderer> {
	PROVIDES_TYPE_NAME(Renderer)
public:
	Renderer();
	virtual ~Renderer();
};

} /* namespace ThesisSascha */
} /* namespace MinSG */

#endif /* MINSG_THESISSASCHA_RENDERER_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
