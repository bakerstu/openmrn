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
 * @file Lpc17xx40xxCan.hxx
 * This file implements a can device driver layer specific to LPC17xx and
 * LPC40xx devices.
 *
 * @author Stuart W. Baker
 * @date 18 April 2015
 */

#ifndef _FREERTOS_DRIVERS_NXP_LPC17XX40XXCAN_HXX_
#define _FREERTOS_DRIVERS_NXP_LPC17XX40XXCAN_HXX_

#include <cstdint>

#include "Can.hxx"

#include "cmsis.h"
#if defined (CHIP_LPC175X_6X)
#include "cmsis_175x_6x.h"
#elif defined (CHIP_LPC177X_9X)
#include "cmsis_177x_8x.h"
#elif defined (CHIP_LPC407X_8X)
#include "cmsis_407x_8x.h"
#else
#error "LPC CHIP undefined"
#endif
#include "core_cm3.h"
#include "can_17xx_40xx.h"

/** Specialization of CAN driver for LPC17xx and LPC40xx CAN.
 */
class LpcCan : public Can
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     */
    LpcCan(const char *name, LPC_CAN_T *base);

    /** Destructor.
     */
    ~LpcCan()
    {
    }

    /** Translate an interrupt handler into C++ object context.
     */
    static void interrupt_handler()
    {
        for (unsigned i = 0; i < 2; ++i)
        {
            if (instances[i])
            {
                uint32_t status = Chip_CAN_GetIntStatus(instances[i]->base);
                if (status != 0)
                {
                    instances[i]->interrupt_handler(status);
                }
            }
        }
    }

private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */
    void tx_msg() override; /**< function to try and transmit a message */

    /** handle an interrupt.
     * @param status interrupt source status
     */
    void interrupt_handler(uint32_t status);

    LPC_CAN_T *base; /**< base address of this device */

    /** one interrupt vector is shared between two CAN controllers, so we need
     *  to keep track of the number of controllers in use.
     */
    static unsigned int intCount;

    /** Instance pointers help us get context from the interrupt handler(s) */
    static LpcCan *instances[2];

    /** Default constructor.
     */
    LpcCan();

    DISALLOW_COPY_AND_ASSIGN(LpcCan);
};

#endif /* _FREERTOS_DRIVERS_NXP_LPC17XX40XXCAN_HXX_ */
