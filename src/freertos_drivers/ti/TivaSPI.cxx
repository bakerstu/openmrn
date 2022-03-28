/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file TivaSPI.cxx
 * This file implements a SPI device driver for Tiva (129).
 *
 * @author Balazs Racz
 * @date 18 Aug 2021
 */

#include "TivaSPI.hxx"

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

/// One semaphore per peripheral.
OSSem sem[4];

TivaSPI::TivaSPI(const char *name, unsigned long base, uint32_t interrupt,
    ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
    OSMutex *bus_lock,
    size_t dma_threshold,
    uint32_t dma_channel_index_tx,
    uint32_t dma_channel_index_rx)
    :  SPI(name, cs_assert, cs_deassert, bus_lock)
    , base_(base)
    , interrupt_(interrupt)
    , dmaThreshold_(dma_threshold)
    , dmaChannelIndexTx_(dma_channel_index_tx)
    , dmaChannelIndexRx_(dma_channel_index_rx)
{
    uint32_t p = 0;
    switch(base_) {
    case SSI0_BASE:
        p = SYSCTL_PERIPH_SSI0;
        sem_ = sem + 0;
        break;
    case SSI1_BASE:
        p = SYSCTL_PERIPH_SSI1;
        sem_ = sem + 1;
        break;
    case SSI2_BASE:
        p = SYSCTL_PERIPH_SSI2;
        sem_ = sem + 2;
        break;
    case SSI3_BASE:
        p = SYSCTL_PERIPH_SSI3;
        sem_ = sem + 3;
        break;
    default:
        DIE("Unknown SPI peripheral.");
    }
    MAP_SysCtlPeripheralEnable(p);
    MAP_SysCtlPeripheralReset(p);

    update_configuration();
}

TivaSPI::~TivaSPI()
{
}

void TivaSPI::enable()
{
}

void TivaSPI::disable()
{
}

/** Update the configuration of the bus.
 * @return >= 0 upon success, -errno upon failure
 */
int TivaSPI::update_configuration()
{
    unsigned long new_mode;

    switch (mode_)
    {
        default:
        case SPI_MODE_0:
            new_mode = SSI_FRF_MOTO_MODE_0;
            break;
        case SPI_MODE_1:
            new_mode = SSI_FRF_MOTO_MODE_1;
            break;
        case SPI_MODE_2:
            new_mode = SSI_FRF_MOTO_MODE_2;
            break;
        case SPI_MODE_3:
            new_mode = SSI_FRF_MOTO_MODE_3;
            break;
    }

    HASSERT(bitsPerWord <= 16);

    uint32_t clock = cm3_cpu_clock_hz;
    /* The SPI peripheral only supoprts certain frequencies and driverlib does
     * not properly round down. We can do some math to make sure that driverlib
     * makes the correct selection.
     */
    if (speedHz > (clock / 2))
    {
        speedHz = clock / 2;
    }
    else if ((clock % speedHz) != 0)
    {
        speedHz = clock / ((clock / speedHz) + 1);
    }

    MAP_SSIDisable(base_);
    MAP_SSIConfigSetExpClk(base_, clock, new_mode, SSI_MODE_MASTER,
                           speedHz, bitsPerWord);
    MAP_IntPrioritySet(interrupt_, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(interrupt_);
    MAP_SSIEnable(base_);

    // these values are used to quickly program instance local configuration
    // settings
    spiCr0_ = HWREG(base_ + SSI_O_CR0);
    spiCr1_ = HWREG(base_ + SSI_O_CR1);
    spiPrescaler_ = HWREG(base_ + SSI_O_CPSR);

    return 0;
}

/** Configure a DMA transaction.
 * @param msg message to transact.
 */
__attribute__((optimize("-O3")))
void TivaSPI::config_dma(struct spi_ioc_transfer *msg)
{
    DIE("DMA transfer is not yet implemented.");
}

__attribute__((optimize("-O3")))
void TivaSPI::interrupt_handler()
{
    DIE("SPI interrupt is not yet implemented.");
}

