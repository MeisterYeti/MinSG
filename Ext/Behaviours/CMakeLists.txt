#
# This file is part of the MinSG library.
# Copyright (C) 2013 Benjamin Eikel <benjamin@eikel.org>
#
# This library is subject to the terms of the Mozilla Public License, v. 2.0.
# You should have received a copy of the MPL along with this library; see the 
# file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
#
minsg_add_sources(
	AbstractBehaviourDecorator.cpp
	BehaviourGroup.cpp
	KeyFrameAnimationBehaviour.cpp
	SimplePhysics.cpp
	SoundEmittingBehaviour.cpp
	SoundReceivingBehaviour.cpp
	SRTBehaviour.cpp
	TimedBehaviourDecorator.cpp
)

minsg_add_extension(MINSG_EXT_SOUND "Defines if the MinSG extension Sound is built.")
minsg_add_dependencies(MINSG_EXT_SOUND MINSG_HAVE_LIB_SOUND)
