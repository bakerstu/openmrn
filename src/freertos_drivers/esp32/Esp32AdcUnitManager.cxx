/** \copyright
 * Copyright (c) 2023, Mike Dunston
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
 * \file Esp32AdcUnitManager.cxx
 *
 * Implementation of ADC conversion unit manager.
 *
 * @author Mike Dunston
 * @date 21 July 2023
 */

#if defined(ESP_PLATFORM)

#include "freertos_drivers/esp32/Esp32AdcOneShot.hxx"

Esp32ADCUnitManager Esp32AdcUnit[SOC_ADC_PERIPH_NUM] =
{
    ADC_UNIT_1,
#if SOC_ADC_PERIPH_NUM >= 2
    ADC_UNIT_2
#endif // SOC_ADC_PERIPH_NUM == 2
};

// Constructor.
Esp32ADCUnitManager::Esp32ADCUnitManager(const adc_unit_t unit)
    : unit_(unit), handle_(nullptr)
{
}

// Initializes the underlying hardware unit if not already initialized.
void Esp32ADCUnitManager::hw_init()
{
    // due to using #if/#elif/#endif it is not possible to include this in
    // the ADC_PIN wrapper code.
    const adc_oneshot_unit_init_cfg_t unit_config =
    {
        .unit_id = unit_,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,1,0)
        // use default value from driver, no constant available thus using
        // 0 with type cast.
        .clk_src = (adc_oneshot_clk_src_t)0,
#endif // IDF v5.1+
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    // Check if we have been initialized previously.
    if (handle())
    {
        return;
    }

    LOG(VERBOSE, "[Esp32ADCUnit] Initializing ADC Unit: %d", unit_);
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &handle_));
}

// Returns the handle to use for this ADC unit.
adc_oneshot_unit_handle_t Esp32ADCUnitManager::handle()
{
    return handle_;
}

#endif // defined(ESP_PLATFORM)