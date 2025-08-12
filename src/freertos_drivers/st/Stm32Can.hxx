/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * @file Stm32Can.hxx
 * This file implements a can device driver layer specific to STM32F0xx devices.
 *
 * @author Stuart W. Baker
 * @date 3 May 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_

#include <cstdint>

#include "freertos_drivers/common/Can.hxx"
#ifdef ARDUINO
#include <Arduino.h>
using CanBase = openmrn_arduino::Can;
#else
using CanBase = ::Can;
#endif

#include "stm32f_hal_conf.hxx"

/// Max possible number of CAN ifs across the whole STM32 -- the constructor
/// will calculate the actual max for the specific chip compiling for
#define MAXCANIFS 3 

/** Specialization of CAN driver for LPC17xx and LPC40xx CAN.
 */
class Stm32Can : public CanBase
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     */
    Stm32Can(const char *name, uint8_t index = 0);

    /** Destructor.
     */
    ~Stm32Can()
    {
    }

    /** Handle an interrupt. */
    void rx_interrupt_handler();
    /** Handle an interrupt. */
    void tx_interrupt_handler();
    /** Handle an interrupt. */
    void sce_interrupt_handler();

    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Can *instances[MAXCANIFS];

private:
#ifndef ARDUINO    
    /// Request an ioctl transaction.
    /// @param file file reference for this device
    /// @param key ioctl key
    /// @param data key data
    /// @return >= 0 upon success, -errno upon failure
    int ioctl(File *file, unsigned long int key, unsigned long data) override;
#endif
    
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */
    void tx_msg() override; /**< function to try and transmit a message */

    /** one interrupt vector is shared between two CAN controllers, so we need
     *  to keep track of the number of controllers in use.
     */
    static unsigned int intCount;

    uint8_t state_; ///< present bus state
    
    CAN_TypeDef *can_; ///< CAN hardware registers for this instance (defined in the SDK).
    IRQn_Type canIrqn_; ///< CAN IRQn for this instance.
    IRQn_Type canSecondIrqn_; ///< CAN Second IRQn (if used) for this instance. 
    IRQn_Type canThirdIrqn_; ///< CAN Third IRQn (if used) for this instance. 
    
    /** Default constructor. */
    Stm32Can();
    
    DISALLOW_COPY_AND_ASSIGN(Stm32Can);
};

#ifdef ARDUINO

extern void arduino_can_pinmap(PinName tx_pin, PinName rx_pin);

#endif

#endif /* _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_ */
