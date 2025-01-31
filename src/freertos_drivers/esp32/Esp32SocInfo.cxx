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
 * \file Esp32SocInfo.cxx
 *
 * Utility class for printing information about the ESP32 device currently in
 * use.
 *
 * @author Mike Dunston
 * @date 4 May 2021
 */

#if defined(ESP_PLATFORM)

#include "freertos_drivers/esp32/Esp32SocInfo.hxx"
#include "utils/logging.h"

#include <esp_ota_ops.h>
#include <esp_chip_info.h>

namespace openmrn_arduino
{

#if defined(CONFIG_IDF_TARGET_ESP32)
/// ESP32 SoC reset reasons.
static constexpr const char * const RESET_REASONS[] =
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
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
/// ESP32-S2 SoC reset reasons.
static constexpr const char * const RESET_REASONS[] =
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
static constexpr const char * const RESET_REASONS[] =
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
    "eFuse Reset",              // EFUSE_RESET              20
    "USB UART Reset",           // USB_UART_CHIP_RESET      21
    "USB JTAG Reset",           // USB_JTAG_CHIP_RESET      22
    "Power Glitch Reset",       // POWER_GLITCH_RESET       23
};
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
/// ESP32-S3 SoC reset reasons.
static constexpr const char * const RESET_REASONS[] =
{
    "unknown",                  // NO_MEAN                  0
    "power on reset",           // POWERON_RESET            1
    "unknown",                  // no key                   2
    "software reset",           // RTC_SW_SYS_RESET         3
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
    "eFuse Reset",              // EFUSE_RESET              20
    "USB UART Reset",           // USB_UART_CHIP_RESET      21
    "USB JTAG Reset",           // USB_JTAG_CHIP_RESET      22
    "Power Glitch Reset",       // POWER_GLITCH_RESET       23
};
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
/// ESP32-H2 SoC reset reasons.
static constexpr const char * const RESET_REASONS[] =
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
    "RTC Reset (Glitch)",       // GLITCH_RTC_RESET         19
    "eFuse Reset",              // EFUSE_RESET              20
    "USB UART Reset",           // USB_UART_CHIP_RESET      21
    "USB JTAG Reset",           // USB_JTAG_CHIP_RESET      22
    "Power Glitch Reset",       // POWER_GLITCH_RESET       23
    "JTAG Reset",               // JTAG_RESET               24
};
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
/// ESP32C2 SoC reset reasons.
static constexpr const char * const RESET_REASONS[] =
{
    "unknown",                  // NO_MEAN                  0x00
    "power on reset",           // POWERON_RESET            0x01
    "unknown",                  // no key                   0x02
    "software reset",           // SW_RESET                 0x03
    "unknown",                  // no key                   0x04
    "deep sleep reset",         // DEEPSLEEP_RESET          0x05
    "unknown",                  // no key                   0x06
    "MWDT (digital) reset",     // MWDT0                    0x07
    "unknown",                  // no key                   0x08
    "RTC WDT reset",            // RTC_WDT                  0x09
    "unknown",                  // no key                   0x0A
    "MWDT reset (CPU)",         // MWDT0                    0x0B
    "software reset (CPU)",     // SW_CPU_RESET             0x0C
    "RTC WDT reset (CPU)",      // RTC_WDT                  0x0D
    "unknown",                  // no key                   0x0E
    "Brownout reset",           // RTCWDT_BROWN_OUT_RESET   0x0F
    "RTC Reset (Normal)",       // RTCWDT_RTC_RESET         0x10
    "unknown",                  // no key                   0x11
    "WDT Super reset",          // SUPER_WDT                0x12
    "unknown",                  // no key                   0x13
    "eFuse CRC error",          // EFUSE_CRC_ERROR          0x14
    "unknown",                  // no key                   0x15
    "unknown",                  // no key                   0x16
    "unknown",                  // no key                   0x17
    "JTAG Reset"                // JTAG_RESET               0x18
};
#endif // IDF Target

/// Mapping of known ESP chip id values.
static constexpr const char * const CHIP_NAMES[] =
{
    // Note: this must be kept in sync with esp_chip_model_t.
    "Unknown",          // 0 Unknown (placeholder)
    "ESP32",            // 1 CHIP_ESP32
    "ESP32-S2",         // 2 CHIP_ESP32S2
    "Unknown",          // 3 Unknown (placeholder)
    "Unknown",          // 4 Unknown (placeholder)
    "ESP32-C3",         // 5 CHIP_ESP32C3
    "ESP32-H2",         // 6 CHIP_ESP32H2
    "Unknown",          // 7 Unknown (placeholder)
    "Unknown",          // 8 Unknown (placeholder)
    "ESP32-S3",         // 9 CHIP_ESP32S3
    "Unknown",          // 10 Unknown (placeholder)
    "Unknown",          // 11 Unknown (placeholder)
    "ESP32-C2",         // 12 CHIP_ESP32C2
};

uint8_t Esp32SocInfo::print_soc_info()
{
    // capture the reason for the CPU reset. For dual core SoCs this will
    // only check the PRO CPU and not the APP CPU since they will usually
    // restart one after the other.
    uint8_t reset_reason = rtc_get_reset_reason(PRO_CPU_NUM);
    uint8_t orig_reset_reason = reset_reason;
    // Ensure the reset reason is within bounds.
    if (reset_reason >= ARRAYSIZE(RESET_REASONS))
    {
        reset_reason = 0;
    }
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    size_t chip_model = chip_info.model;
    // Ensure chip model is within bounds.
    if (chip_model >= ARRAYSIZE(CHIP_NAMES))
    {
        chip_model = 0;
    }

    LOG(INFO, "[SoC] reset reason:%d - %s", reset_reason,
        RESET_REASONS[reset_reason]);
    LOG(INFO,
        "[SoC] model:%s,rev:%d,cores:%d,flash:%s,WiFi:%s,BLE:%s,BT:%s",
        CHIP_NAMES[chip_model], chip_info.revision,
        chip_info.cores,
        chip_info.features & CHIP_FEATURE_EMB_FLASH ? "Yes" : "No",
        chip_info.features & CHIP_FEATURE_WIFI_BGN ? "Yes" : "No",
        chip_info.features & CHIP_FEATURE_BLE ? "Yes" : "No",
        chip_info.features & CHIP_FEATURE_BT ? "Yes" : "No");

    LOG(INFO, "[SoC] Heap: %.2fkB / %.2fkB",
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.0f,
        heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024.0f);
#if CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM || CONFIG_SPIRAM
    LOG(INFO, "[SoC] PSRAM: %.2fkB / %.2fkB",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.0f,
        heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024.0f);
#endif // CONFIG_SPIRAM_SUPPORT || BOARD_HAS_PSRAM || CONFIG_SPIRAM

    LOG(INFO, "[SoC] App running from partition: %s",
        esp_ota_get_running_partition()->label);
    if (reset_reason != orig_reset_reason)
    {
        LOG(WARNING, "Reset reason mismatch: %d vs %d", reset_reason,
            orig_reset_reason);
    }
    return reset_reason;
}

} // namespace openmrn_arduino

#endif // ESP_PLATFORM
