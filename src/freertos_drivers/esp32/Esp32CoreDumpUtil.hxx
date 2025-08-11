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
 * \file Esp32CoreDumpUtil.hxx
 *
 * Utility class for handling ESP32 core dump files stored in flash.
 *
 * @author Mike Dunston
 * @date 8 August 2021
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32COREDUMPUTIL_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32COREDUMPUTIL_HXX_

#include "sdkconfig.h"
#include "os/Gpio.hxx"
#include "utils/FileUtils.hxx"
#include "utils/logging.h"
#include "utils/StringPrintf.hxx"

#include <esp_core_dump.h>
#include <esp_log.h>

namespace openmrn_arduino
{

// Currently the core dump functionality here supports only the xtensa
// architecture and requires multiple core dump flags in sdkconfig which are
// not enabled by default.
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && \
  CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF && \
  CONFIG_IDF_TARGET_ARCH_XTENSA
#define CORE_DUMP_USABLE_XTENSA 1
#endif

/// Utility class containing methods related to esp32 core dumps.
///
/// This class requires the following sdkconfig flags to be enabled and the
/// addition of a 64kb "coredump" partition in flash.
///
/// Required sdkconfig flags:
/// * CONFIG_ESP32_ENABLE_COREDUMP_TO_FLASH
/// * CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
/// If not all three flags are enabled all methods will be treated as no-op.
///
/// At this time the RISC-V MCUs are not suported and may become supported in
/// the future.
///
/// NOTE: ESP-IDF v4.4 or later is required for the display() method.
class Esp32CoreDumpUtil
{
public:
  /// Utility method to check if a core dump is present.
  /// @return true if a core dump is available, false otherwise.
  static bool is_present()
  {
#ifdef CORE_DUMP_USABLE_XTENSA
    esp_log_level_set("esp_core_dump_flash", ESP_LOG_NONE);
    esp_err_t res = esp_core_dump_image_check();
    esp_log_level_set("esp_core_dump_flash", ESP_LOG_WARN);
    return res == ESP_OK;
#else // !CORE_DUMP_USABLE_XTENSA
    return false;
#endif
  }

  /// Utility method that displays a core dump (if present).
  ///
  /// @param output_path Optional output path to store the code dump on the
  /// filesystem.
  static void display(const char *output_path = nullptr)
  {
#ifdef CORE_DUMP_USABLE_XTENSA
    if (is_present())
    {
      esp_core_dump_summary_t details;
      if (esp_core_dump_get_summary(&details) == ESP_OK)
      {
        // Convert the core dump to a text file
        string core_dump_summary =
            StringPrintf("Task:%s (%" PRIu32 ") crashed at PC %08" PRIx32 "\n",
                         details.exc_task, details.exc_tcb, details.exc_pc);
        core_dump_summary += StringPrintf("Registers:\n");
        for (size_t idx = 0; idx < 16; idx += 4)
        {
          core_dump_summary +=
              StringPrintf(
                  "A%02zu: 0x%08" PRIx32 " A%02zu: 0x%08" PRIx32
                  " A%02zu: 0x%08" PRIx32 " A%02zu: 0x%08" PRIx32 "\n",
                  idx, details.ex_info.exc_a[idx],
                  idx + 1, details.ex_info.exc_a[idx + 1],
                  idx + 2, details.ex_info.exc_a[idx + 2],
                  idx + 3, details.ex_info.exc_a[idx + 3]);
        }
        core_dump_summary +=
            StringPrintf("EXCCAUSE: %08" PRIx32 " EXCVADDR: %08" PRIx32 "\n",
                         details.ex_info.exc_cause, details.ex_info.exc_vaddr);
        if (details.ex_info.epcx_reg_bits)
        {
          core_dump_summary += "EPCX:";
          for (size_t idx = 0; idx < 8; idx++)
          {
            if (details.ex_info.epcx_reg_bits & BIT(idx))
            {
              core_dump_summary +=
                  StringPrintf("%zu:%08" PRIx32 " ", idx,
                    details.ex_info.epcx[idx]);
            }
          }
          core_dump_summary += "\n";
        }
        core_dump_summary += "Backtrace:";
        for (size_t idx = 0; idx < details.exc_bt_info.depth; idx++)
        {
          core_dump_summary +=
              StringPrintf(" 0x%08" PRIx32, details.exc_bt_info.bt[idx]);
          if (details.exc_bt_info.corrupted)
          {
            core_dump_summary += "(corrupted)";
          }
        }
        core_dump_summary += "\n";
        LOG_ERROR("Core dump:\n%s", core_dump_summary.c_str());
        if (output_path)
        {
          write_string_to_file(output_path, core_dump_summary);
        }
      }
    }
#endif // CORE_DUMP_USABLE_XTENSA
  }

  /// Utility method to cleanup a core dump.
  static void cleanup()
  {
#ifdef CORE_DUMP_USABLE_XTENSA
    if (is_present())
    {
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_core_dump_image_erase());
    }
#endif // CORE_DUMP_USABLE_XTENSA
  }
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32CoreDumpUtil;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32COREDUMPUTIL_HXX_