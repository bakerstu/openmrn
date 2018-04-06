/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file DmxSceneConfig.hxx
 * CDI configuration for the Scene based DMX control.
 *
 * @author Balazs Racz
 * @date 28 Feb 2018
 */

#ifndef _DMX_DMXSCENECONFIG_HXX_
#define _DMX_DMXSCENECONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace dmx
{

constexpr unsigned SCENE_CONFIG_NAME_LEN = 32;
constexpr unsigned SCENE_CONFIG_CHANNEL_COUNT = 8;
constexpr unsigned SCENE_CONFIG_SCENE_COUNT = 16;

// A SceneChannelConfig is 5 bytes of memory
CDI_GROUP(SceneChannelConfig);
CDI_GROUP_ENTRY(delay, openlcb::Uint16ConfigEntry, Name("Time delay (ms)"),
    Default(0), Min(0),
    Description("When non-zero, this channel change will happen the given time "
                "(in milliseconds) later than the scene was called up. A value "
                "of 1000 is one second delay."));
CDI_GROUP_ENTRY(channel, openlcb::Uint16ConfigEntry, Name("DMX channel"),
    Default(0), Min(0), Max(512),
    Description("Sets the DMX channel of the light/parameter to change. Set to "
                "zero to not change anything."));
CDI_GROUP_ENTRY(value, openlcb::Uint8ConfigEntry, Name("Value"), Default(0),
    Min(0), Max(255), Description("What value to set the DMX parameter to."));
CDI_GROUP_END();

using AllChannelConfig =
    openlcb::RepeatedGroup<SceneChannelConfig, SCENE_CONFIG_CHANNEL_COUNT>;

// A sceneconfig is 32+8+8*5 = 80 bytes long.
CDI_GROUP(SceneConfig);
CDI_GROUP_ENTRY(name, openlcb::StringConfigEntry<SCENE_CONFIG_NAME_LEN>,
    Name("Name"),
    Description("User description for the scene. This does not impact the "
                "device's operation."));
CDI_GROUP_ENTRY(event, openlcb::EventConfigEntry,
    Name("Event to call up scene"),
    Description("When this event is consumed, the scene will be activated. You "
                "can have one event activate multiple scenes if you need to "
                "set more channels than the parameters below."));
CDI_GROUP_ENTRY(channels, AllChannelConfig, RepName("Entry"),
    Name("Scene parameters"),
    Description("Sets individual DMX channels to specific values when the "
                "scene gets activated."));
CDI_GROUP_END();

using AllSceneConfig = openlcb::RepeatedGroup<SceneConfig, SCENE_CONFIG_SCENE_COUNT>;

} // namespace dmx

#endif // _DMX_DMXSCENECONFIG_HXX_
