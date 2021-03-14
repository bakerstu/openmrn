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
 * \file Stm32SPI.hxx
 *
 * This file implements a SPI device driver layer on top of the STM32 Cube
 * middleware.
 *
 * @author Balazs Racz
 * @date 29 April 2018
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32SPI_HXX_
#define _FREERTOS_DRIVERS_ST_STM32SPI_HXX_

#include "SPI.hxx"

#include "stm32f_hal_conf.hxx"

/** Specialization of SPI driver for STM32 devices.
 */
class Stm32SPI : public SPI
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param port hardware instance of this device, e.g. SPI1
     * @param interrupt interrupt number of this device (unused)
     * @param cs_assert function pointer to a method that asserts chip select
     * @param cs_deassert function pointer to a method that deasserts chip
     *                    select
     * @param bus_lock the user must provide a shared mutex if the device
     *                 instance represents more than one chip select on the
     *                 same bus interface.
     */
    Stm32SPI(const char *name, SPI_TypeDef *port, uint32_t interrupt,
            ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
            OSMutex *bus_lock = nullptr);

    /** Destructor.
     */
    ~Stm32SPI()
    {
    }

private:
    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

    /** Method to transmit/receive the data.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    int transfer(struct spi_ioc_transfer *msg) override;

    /** Method to transmit/receive the data.  This will be always polled mode.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    int transfer_polled(struct spi_ioc_transfer *msg) override
    {
        return transfer(msg);
    }

    /** Update the configuration of the bus.
     * @return >= 0 upon success, -errno upon failure
     */
    int update_configuration() override;

    /// Stm32 HAL device structure.
    SPI_HandleTypeDef spiHandle_;

    /** Default constructor.
     */
    Stm32SPI();

    DISALLOW_COPY_AND_ASSIGN(Stm32SPI);
};




#endif // _FREERTOS_DRIVERS_ST_STM32SPI_HXX_
