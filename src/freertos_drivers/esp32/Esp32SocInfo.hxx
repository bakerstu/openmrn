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
 * TWAI driver implementation for OpenMRN. This leverages the ESP-IDF TWAI HAL
 * API rather than the TWAI driver to allow for a more integrated solution than
 * the TWAI driver which requires polling for RX. This implementation supports
 * both ::select and the non-blocking ::ioctl/::fnctl approach.
 *
 * @author Mike Dunston
 * @date 4 May 2021
 */
#ifndef _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_

#include "sdkconfig.h"

#include <esp_idf_version.h>
#include <esp_ota_ops.h>
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp32s2/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s3/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#include <esp32c3/rom/rtc.h>
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,3,0)
#include <esp32/rom/rtc.h>
#else
#include <rom/rtc.h>
#endif
#endif

#include "utils/logging.h"

namespace openmrn_arduino
{

#if defined(CONFIG_IDF_TARGET_ESP32S2)
/// ESP32-S2 SoC reset reasons.
static const char * const ESP32_SOC_RESET_REASONS[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // SW_RESET                 3
    "unknown",                  // no key                   4
    "deep sleep reset",         // DEEPSLEEP_RESET          5
    "unknown",                  // no key                   6
    "watchdog reset (group0)",  // TG0WDT_SYS_RESET         7
    "watchdog reset (group1)",  // TG1WDT_SYS_RESET         8
    "RTC system reset",         // RTCWDT_SYS_RESET         9
    "Intrusion test reset",     // INTRUSION_RESET          10
    "WDT Timer group0 reset",   // TG0WDT_CPU_RESET         11
    "software reset (CPU)",     // RTC_SW_CPU_RESET         12
    "RTC WDT reset",            // RTCWDT_CPU_RESET         13
    "unknown",                  // no key                   14
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   15
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         16
    "WDT Timer group1 reset",   // TG1WDT_CPU_RESET         17
    "WDT Reset",                // SUPER_WDT_RESET          18
    "RTC Reset (Glitch)",       // GLITCH_RTC_RESET         19
};
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
/// ESP32-C3 SoC reset reasons.
static const char * const ESP32_SOC_RESET_REASONS[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // SW_RESET                 3
    "unknown",                  // no key                   4
    "deep sleep reset",         // DEEPSLEEP_RESET          5
    "reset (SLC)",              // SDIO_RESET               6
    "watchdog reset (group0)",  // TG0WDT_SYS_RESET         7
    "watchdog reset (group1)",  // TG1WDT_SYS_RESET         8
    "RTC system reset",         // RTCWDT_SYS_RESET         9
    "Intrusion test reset",     // INTRUSION_RESET          10
    "WDT Timer group0 reset",   // TG0WDT_CPU_RESET         11
    "software reset (CPU)",     // RTC_SW_CPU_RESET         12
    "RTC WDT reset",            // RTCWDT_CPU_RESET         13
    "unknown",                  // no key                   14
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   15
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         16
    "WDT Timer group1 reset",   // TG1WDT_CPU_RESET         17
    "WDT Reset",                // SUPER_WDT_RESET          18
};
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#else
/// ESP32 SoC reset reasons.
static const char * const ESP32_SOC_RESET_REASONS[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // SW_RESET                 3
    "watchdog reset (legacy)",  // OWDT_RESET               4
    "deep sleep reset",         // DEEPSLEEP_RESET          5
    "reset (SLC)",              // SDIO_RESET               6
    "watchdog reset (group0)",  // TG0WDT_SYS_RESET         7
    "watchdog reset (group1)",  // TG1WDT_SYS_RESET         8
    "RTC system reset",         // RTCWDT_SYS_RESET         9
    "Intrusion test reset",     // INTRUSION_RESET          10
    "WDT Timer group reset",    // TGWDT_CPU_RESET          11
    "software reset (CPU)",     // SW_CPU_RESET             12
    "RTC WDT reset",            // RTCWDT_CPU_RESET         13
    "software reset (CPU)",     // EXT_CPU_RESET            14
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   15
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         16
};
#endif

class Esp32SocInfo
{
public:
    static uint8_t print_soc_info()
    {
        // capture the reason for the CPU reset. For dual core SoCs this will
        // only check the PRO CPU and not the APP CPU since they will usually
        // restart one after the other.
        uint8_t reset_reason = rtc_get_reset_reason(PRO_CPU_NUM);
        uint8_t orig_reset_reason = reset_reason;
        // Ensure the reset reason it within bounds.
        if (reset_reason > ARRAYSIZE(ESP32_SOC_RESET_REASONS))
        {
            reset_reason = 0;
        }
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        LOG(INFO, "[SoC] reset reason:%d - %s", reset_reason,
            ESP32_SOC_RESET_REASONS[reset_reason]);
        LOG(INFO,
            "[SoC] model:%s,rev:%d,cores:%d,flash:%s,WiFi:%s,BLE:%s,BT:%s",
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,3,0)
            chip_info.model == CHIP_ESP32 ? "ESP32" :
            chip_info.model == CHIP_ESP32S2 ? "ESP32-S2" :
            chip_info.model == CHIP_ESP32C3 ? "ESP32-C3" : "unknown",
#else
            "ESP32",
#endif // IDF v4.3+
            chip_info.revision, chip_info.cores,
            chip_info.features & CHIP_FEATURE_EMB_FLASH ? "Yes" : "No",
            chip_info.features & CHIP_FEATURE_WIFI_BGN ? "Yes" : "No",
            chip_info.features & CHIP_FEATURE_BLE ? "Yes" : "No",
            chip_info.features & CHIP_FEATURE_BT ? "Yes" : "No");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,3,0)
        LOG(INFO, "[SoC] Heap: %.2fkB / %.2fkB",
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f,
            heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024.0f);
#if CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM
        LOG(INFO, "[SoC] PSRAM: %.2fkB / %.2fkB",
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.0f,
            heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024.0f);
#endif // CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM
#else // NOT IDF v4.3+
        LOG(INFO, "[SoC] Free Heap: %.2fkB",
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f);
#if CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM
        LOG(INFO, "[SoC] Free PSRAM: %.2fkB",
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.0f);
#endif // CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM

#endif // IDF v4.3+
        LOG(INFO, "[SoC] App running from: %s",
            esp_ota_get_running_partition()->label);
        if (reset_reason != orig_reset_reason)
        {
            LOG(WARNING, "Reset reason mismatch: %d vs %d", reset_reason,
                orig_reset_reason);
        }
        return reset_reason;
    }
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32SocInfo;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32SOCINFO_HXX_