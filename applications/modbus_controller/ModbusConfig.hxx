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

#ifndef _MODBUS_MODBUSSCENECONFIG_HXX_
#define _MODBUS_MODBUSSCENECONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace modbus
{

constexpr unsigned SCENE_CONFIG_NAME_LEN = 32;
constexpr unsigned SCENE_CONFIG_CHANNEL_COUNT = 4;
constexpr unsigned SCENE_CONFIG_SCENE_COUNT = 16;

CDI_GROUP(ScenePacketConfig, FixedSize(32));
CDI_GROUP_ENTRY(packet, openlcb::StringConfigEntry<24>, Name("MODBUS request"),
    Description(
        "Modbus ASCII packet to send. Must begin with :. If ends with ! then "
        "the trailing ! will be replaced with a checksum byte. CRLF is added "
        "automatically. Example: :F7031389000A! or :F7031389000A60. If empty, "
        "a register write will be created from the components below."));
CDI_GROUP_ENTRY(channel, openlcb::Uint8ConfigEntry, Name("Slave address"),
    Default(0), Min(0), Max(255),
    Description("Sets the modbus client (slave) address."));
CDI_GROUP_ENTRY(function, openlcb::Uint8ConfigEntry, Name("Function Code"),
    Default(6), Min(1), Max(255),
    Description("Sets the function code to invoke. Default of 6 is 'write "
                "holding register'."));
CDI_GROUP_ENTRY(address, openlcb::Uint16ConfigEntry,
    Name("Register number"), Default(5001), Min(0), Max(65536),
    Description("Sets the holding register to write."));
CDI_GROUP_ENTRY(value, openlcb::Uint16ConfigEntry, Name("Register value"),
    Default(0), Min(0), Max(65536),
    Description("Sets the value to write to the register."));
CDI_GROUP_END();

using AllPacketConfig =
    openlcb::RepeatedGroup<ScenePacketConfig, SCENE_CONFIG_CHANNEL_COUNT>;

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
CDI_GROUP_ENTRY(packets, AllPacketConfig, RepName("Packet"),
    Name("MODBus Packets to send"),
    Description(
        "Sends individual MODBUS packets when the scene gets activated."));
CDI_GROUP_END();

using AllSceneConfig = openlcb::RepeatedGroup<SceneConfig, SCENE_CONFIG_SCENE_COUNT>;

} // namespace modbus

#endif // _MODBUS_MODBUSSCENECONFIG_HXX_
