/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file Esp32SocInfo.hxx
 *
 * Utility class which provides details of the running ESP32 SoC.
 *
 * @author Mike Dunston
 * @date 4 May 2021
 */
#ifndef _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_

#include <stdint.h>

#if defined(ESP_PLATFORM)

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp32/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp32s2/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s3/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#include <esp32c3/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#include <esp32h2/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#include <esp32c2/rom/rtc.h>
#endif

namespace openmrn_arduino
{

/// Utility class which logs information about the currently running SoC.
class Esp32SocInfo
{
public:
    /// Logs information about the currently running SoC.
    ///
    /// @return Reason for the reset of the SoC.
    static uint8_t print_soc_info();
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32SocInfo;

#endif // ESP_PLATFORM

#endif // _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_
