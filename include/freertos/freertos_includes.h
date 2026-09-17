/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file freertos_includes.h
 * This file simplifies the include path for FreeRTOS header files between
 * platforms.
 *
 * @author Balazs Racz
 * @date 2 March 2019
 */

#ifdef ESP_PLATFORM

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "sdkconfig.h"

#define NSEC_TO_TICK(ns)                                                       \
    (((((long long)(ns)) / 1000 * configTICK_RATE_HZ) + 999999) / 1000000)

// IDF v5.0 has introduced a configuration option (disabled by default) which
// enables the usage of legacy FreeRTOS data types, if that configuration option
// is *NOT* selected *AND* IDF v5.0+ is in use we need to add compatibility
// defines in order to compile OpenMRN successfully.
#if !defined(CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY)

// used in os/os.c and os/os.h
#define portTickType                  TickType_t
#define xTaskHandle                   TaskHandle_t
#define xQueueHandle                  QueueHandle_t
#define xSemaphoreHandle              SemaphoreHandle_t

// used in freertos_drivers/common/CpuLoad.hxx and os/os.c
#define pcTaskGetTaskName             pcTaskGetName

#endif // IDF v5.0+ and !CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY

#else

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define NSEC_TO_TICK(ns) ((ns) >> NSEC_TO_TICK_SHIFT)

#endif
