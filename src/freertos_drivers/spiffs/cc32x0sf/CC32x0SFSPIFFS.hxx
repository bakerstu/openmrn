/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file CC32xxSPIFFS.hxx
 * This file implements a SPIFFS FLASH driver specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 1 January 2018
 */

#ifndef _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32XXSPIFFS_HXX_
#define _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32XXSPIFFS_HXX_

#ifndef gcc
#define gcc
#endif

#include <cstdint>

#include "spiffs.h"

/** Specialization of Serial SPI driver for CC32xx devices.
 */
class CC32xxSPIFFS : public SPI
{
public:
    /** Constructor.
     */
    CC32xxSPIFFS(const char *name, unsigned long base, uint32_t interrupt,
            ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
            OSMutex *bus_lock = nullptr);

    /** Destructor.
     */
    ~CC32xxSPIFFS()
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

    /** Update the configuration of the bus.
     * @return >= 0 upon success, -errno upon failure
     */
    int update_configuration() override;

    unsigned long base; /**< base address of this device */
    unsigned long clock; /**< clock rate supplied to the module */
    unsigned long interrupt; /**< interrupt of this device */
    struct spi_ioc_transfer *msg_; /**< message for current transaction */
    bool stop_; /**< current transaction ends in a stop if true */
    int count_; /**< current count index within transaction */

    /** Semaphore to wakeup task level from ISR */
    OSSem sem;

    DISALLOW_COPY_AND_ASSIGN(CC32xxSPIFFS);
};

#endif _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32XXSPIFFS_HXX_
