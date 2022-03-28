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
 * \file Stm32I2C.hxx
 *
 * This file implements an I2C device driver layer on top of the STM32 Cube
 * middleware.
 *
 * @author Balazs Racz
 * @date 28 Oct 2018
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32I2C_HXX_
#define _FREERTOS_DRIVERS_ST_STM32I2C_HXX_

#include "I2C.hxx"

#include "stm32f_hal_conf.hxx"

/** Specialization of I2C driver for STM32 devices.
 */
class Stm32I2C : public I2C
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param port hardware instance of this device, e.g. I2C1
     * @param ev_interrupt event interrupt vector number
     * @param er_interrupt error interrupt vector number
     */
    Stm32I2C(const char *name, I2C_TypeDef *port, uint32_t ev_interrupt,
        uint32_t er_interrupt);

    /** Destructor.
     */
    ~Stm32I2C()
    {
    }

    /// Call this function from the specific i2c interrupt routine in HwInit.
    void event_interrupt_handler()
    {
        HAL_I2C_EV_IRQHandler(&i2cHandle_);
    }

    /// Call this function from the specific i2c interrupt routine in HwInit.
    void error_interrupt_handler()
    {
        HAL_I2C_ER_IRQHandler(&i2cHandle_);
    }

    /// Internal. This function is called from the complete ISR callback.
    inline void complete_from_isr();
    /// Internal. This function is called from the error ISR callback.
    inline void error_from_isr();

private:
    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

    /** Method to transmit/receive the data.
     * @param msg message to transact.
     * @param stop produce a stop condition at the end of the transfer
     * @return bytes transfered upon success, -errno upon failure
     */
    int transfer(struct i2c_msg *msg, bool stop) override;

    /// Stm32 HAL device structure.
    I2C_HandleTypeDef i2cHandle_;
    /// Pending transfer error field.
    int error_;
    /// Semaphore to wakeup task level from ISR
    OSSem sem;

    /** Default constructor.
     */
    Stm32I2C();

    DISALLOW_COPY_AND_ASSIGN(Stm32I2C);
};

#endif // _FREERTOS_DRIVERS_ST_STM32I2C_HXX_
