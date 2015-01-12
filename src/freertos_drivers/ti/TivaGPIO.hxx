/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TivaGPIO.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on Tiva MCUs.
 *
 * @author Balazs Racz
 * @date 2 Dec 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

struct DummyPin {
    static void hw_init() {}
    static void hw_set_to_safe() {}
    static void set(bool value) {}
    static void toggle() {}
};

#define DECL_PIN(NAME, PORT, NUM)                                              \
    static const auto NAME##_PERIPH = SYSCTL_PERIPH_GPIO##PORT;                \
    static const auto NAME##_BASE = GPIO_PORT##PORT##_BASE;                    \
    static const auto NAME##_PIN = GPIO_PIN_##NUM

#define DECL_HWPIN(NAME, PORT, NUM, CONFIG)                                    \
    DECL_PIN(NAME, PORT, NUM);                                                 \
    static const auto NAME##_CONFIG = GPIO_P##PORT##NUM##_##CONFIG

template<class Defs, bool SAFE_VALUE>
struct GpioOutputPin : public Defs {
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    static void hw_init() {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeGPIOOutput(GPIO_BASE, GPIO_PIN);
        set(SAFE_VALUE);
    }
    static void hw_set_to_safe() {
        hw_init();
    }
    static void set(bool value) {
        //if (GPIO_INVERTED) value = !value;
        uint8_t *ptr = reinterpret_cast<uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        *ptr = value ? 0xff : 0;
    }
    static bool get() {
        const uint8_t *ptr = reinterpret_cast<const uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        return *ptr;
    }
    static void toggle() {
        set(!get());
    }
};

template<class Defs>
struct GpioOutputSafeLow : public GpioOutputPin<Defs, false> {};

template<class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true> {};

template<class Defs>
struct LedPin : public GpioOutputPin<Defs, false> {
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    static void hw_init() {
        GpioOutputPin<Defs, false>::hw_init();
        MAP_GPIOPadConfigSet(GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);
    }
};

#define GPIO_PIN(NAME, BaseClass, PORT, NUM)                                   \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_PIN(GPIO, PORT, NUM);                                             \
        static const bool GPIO_INVERTED = false;                               \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

template<class Defs, uint32_t GPIO_PULL>
struct GpioInputPin : public Defs {
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    static void hw_init() {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeGPIOInput(GPIO_BASE, GPIO_PIN);
        MAP_GPIOPadConfigSet(GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PULL);
    }
    static void hw_set_to_safe() {
        hw_init();
    }
    static bool get() {
        const uint8_t *ptr = reinterpret_cast<const uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        return *ptr;
    }
};

template<class Defs>
struct GpioInputPU : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD_WPU> {};

template<class Defs>
struct GpioInputPD : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD_WPD> {};

template<class Defs>
struct GpioInputNP : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD> {};

template<class Defs>
struct GpioHwPin : public Defs {
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    using Defs::GPIO_CONFIG;
    static void hw_init() {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinConfigure(GPIO_CONFIG);
        /// TODO(balazs.racz) this is wrong, we have to define somehow which typ eto call.
        MAP_GPIOPinTypeUART(GPIO_BASE, GPIO_PIN);
        /// TODO(balazs.racz): we need to somehow specify what to do to be safe. Options are drive low, drive high, input std, input wpu, input wpd.
    }
    static void hw_set_to_safe() {
        hw_init();
    }
};

#define GPIO_HWPIN(NAME, BaseClass, PORT, NUM, CONFIG)                         \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_HWPIN(GPIO, PORT, NUM, CONFIG);                                   \
        static const bool GPIO_INVERTED = false;                               \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

#endif //_FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
