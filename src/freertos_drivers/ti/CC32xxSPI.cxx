/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxSPI.cxx
 * This file implements a SPI device driver layer specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 30 May 2016
 */

#include "CC32xxSPI.hxx"

#include <algorithm>
#include <unistd.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_udma.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/spi.h"
#include "driverlib/interrupt.h"
#include "driverlib/prcm.h"

/** Instance pointers help us get context from the interrupt handler(s) */
static CC32xxSPI *instances_[1] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 * @param cs_assert function pointer to a method that asserts chip select
 * @param cs_deassert function pointer to a method that deasserts chip
 *                    select
 * @param bus_lock the user must provide a shared mutex if the device
 *                 instance represents more than one chip select on the
 *                 same bus interface.
 */
CC32xxSPI::CC32xxSPI(const char *name, unsigned long base, uint32_t interrupt,
                     ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
                     OSMutex *bus_lock, size_t dma_threshold,
                     uint32_t dma_channel_index_tx,
                     uint32_t dma_channel_index_rx)
    : SPI(name, cs_assert, cs_deassert, bus_lock)
    , base_(base)
    , interrupt_(interrupt)
    , dmaThreshold_(dma_threshold)
    , dmaChannelIndexTx_(dma_channel_index_tx)
    , dmaChannelIndexRx_(dma_channel_index_rx)
    , sem_()
{
    switch (base_)
    {
        default:
            HASSERT(0);
        case GSPI_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
            MAP_PRCMPeripheralReset(PRCM_GSPI);
            clock_ = MAP_PRCMPeripheralClockGet(PRCM_GSPI);
            instances_[0] = this;
            break;
    }

    update_configuration();
}

/** Update the configuration of the bus.
 * @return >= 0 upon success, -errno upon failure
 */
int CC32xxSPI::update_configuration()
{
    unsigned long new_mode;
    unsigned long bits_per_word;

    switch (mode_)
    {
        default:
        case SPI_MODE_0:
            new_mode = SPI_SUB_MODE_0;
            break;
        case SPI_MODE_1:
            new_mode = SPI_SUB_MODE_1;
            break;
        case SPI_MODE_2:
            new_mode = SPI_SUB_MODE_2;
            break;
        case SPI_MODE_3:
            new_mode = SPI_SUB_MODE_3;
            break;
    }

    switch (bitsPerWord)
    {
        default:
        case 0:
        case 8:
            bits_per_word = SPI_WL_8;
            break;
        case 16:
            bits_per_word = SPI_WL_16;
            break;
        case 32:
            bits_per_word = SPI_WL_32;
            break;
    }

    MAP_SPIDisable(base_);
    MAP_SPIReset(base_);
    MAP_SPIConfigSetExpClk(base_, clock_, speedHz, SPI_MODE_MASTER, new_mode,
                           (SPI_3PIN_MODE | SPI_TURBO_OFF | bits_per_word));
    MAP_SPIFIFOEnable(base_, SPI_RX_FIFO | SPI_TX_FIFO);
    MAP_SPIFIFOLevelSet(base_, 1, 1);
    MAP_IntPrioritySet(interrupt_, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(interrupt_);
    MAP_SPIEnable(base_);

    return 0;
}

/** Configure a DMA transaction.
 * @param msg message to transact.
 */
__attribute__((optimize("-O3")))
void CC32xxSPI::config_dma(struct spi_ioc_transfer *msg)
{
    static uint32_t scratch_buffer __attribute__((aligned(4))) = 0;

    /* use DMA */
    void *buf;
    uint32_t channel_control_options;

    /** @todo support longer SPI transactions */
    HASSERT(msg->len <= MAX_DMA_TRANSFER_AMOUNT);

    if (msg->tx_buf)
    {
        channel_control_options = dmaTxConfig_;
        buf = (void*)msg->tx_buf;
    }
    else
    {
        channel_control_options = dmaNullConfig_;
        buf = &scratch_buffer;
    }

    /* Setup the TX transfer characteristics & buffers */
    uDMAChannelControlSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                          channel_control_options);
#if 0
    uDMAChannelAttributeDisable(dmaChannelIndexTx_,
                                    UDMA_ATTR_ALTSELECT);
#else
    HWREG(UDMA_BASE + UDMA_O_ALTCLR) = 1 << (dmaChannelIndexTx_ & 0x1f);
#endif
    uDMAChannelTransferSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, buf,
                               (void*)(base_ + MCSPI_O_TX0), msg->len);

    if (msg->rx_buf)
    {
        channel_control_options = dmaRxConfig_;
        buf = (void*)msg->rx_buf;
    }
    else
    {
        channel_control_options = dmaNullConfig_;
        buf = &scratch_buffer;
    }

    /* Setup the RX transfer characteristics & buffers */
    uDMAChannelControlSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                              channel_control_options);
#if 0
    uDMAChannelAttributeDisable(dmaChannelIndexRx_,
                                    UDMA_ATTR_ALTSELECT);
#else
    HWREG(UDMA_BASE + UDMA_O_ALTCLR) = 1 << (dmaChannelIndexRx_ & 0x1f);
#endif
    uDMAChannelTransferSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void*)(base_ + MCSPI_O_RX0), buf, msg->len);

    /* Globally disables interrupts. */
    asm("cpsid i\n");

    uDMAChannelAssign(dmaChannelIndexRx_);
    uDMAChannelAssign(dmaChannelIndexTx_);

    SPIFIFOLevelSet(base_, 1, 1);
    SPIDmaEnable(base_, SPI_RX_DMA | SPI_TX_DMA);
    SPIIntClear(base_, SPI_INT_DMARX);
    SPIIntEnable(base_, SPI_INT_DMARX);

    /* Enable channels & start DMA transfers */
    uDMAChannelEnable(dmaChannelIndexTx_);
    uDMAChannelEnable(dmaChannelIndexRx_);

    /* enable interrupts */
    asm("cpsie i\n");

    sem_.wait();
}

/** Common interrupt handler for all SPI devices.
 */
__attribute__((optimize("-O3")))
void CC32xxSPI::interrupt_handler()
{
    if (SPIIntStatus(base_, true) & SPI_INT_RX_FULL)
    {
        SPIIntDisable(base_, SPI_INT_RX_FULL);
    }
    else
    {
        if (uDMAChannelIsEnabled(dmaChannelIndexRx_))
        {
            /* DMA has not completed if the channel is still enabled */
            return;
        }

        SPIDmaDisable(base_, SPI_RX_DMA | SPI_TX_DMA);
        SPIIntDisable(base_, SPI_INT_DMARX);
    }

    int woken = 0;
    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** SPI0 interrupt handler.
 */
__attribute__((optimize("-O3")))
void spi0_interrupt_handler(void)
{
    if (instances_[0])
    {
        instances_[0]->interrupt_handler();
    }
}
} // extern C
