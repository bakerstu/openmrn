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

#include <algorithm>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_mcspi.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/spi.h"
#include "driverlib/interrupt.h"
#include "driverlib/prcm.h"

#include <ti/drivers/dma/UDMACC32XX.h>

#include "CC32xxSPI.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static CC32xxSPI *instances[1] = {NULL};

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
    , base(base)
    , interrupt(interrupt)
    , dmaThreshold_(dma_threshold)
    , dmaChannelIndexTx_(dma_channel_index_tx)
    , dmaChannelIndexRx_(dma_channel_index_rx)
    , dmaTransferSize_(0)
    , msg_(NULL)
    , stop_(false)
    , count_(0)
    , sem_()
{
    switch (base)
    {
        default:
            HASSERT(0);
        case GSPI_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
            MAP_PRCMPeripheralReset(PRCM_GSPI);
            clock = MAP_PRCMPeripheralClockGet(PRCM_GSPI);
            instances[0] = this;
            break;
    }

    update_configuration();
}

/** Method to transmit/receive the data.
 * @param msg message(s) to transact.
 * @return bytes transfered upon success, -errno upon failure
 */
__attribute__((optimize("-O3")))
int CC32xxSPI::transfer(struct spi_ioc_transfer *msg)
{
    uint32_t scratch_buffer __attribute__((aligned(4))) = 0;
    uint32_t len = msg->len;
    unsigned long tx_buf = msg->tx_buf;
    unsigned long rx_buf = msg->rx_buf;

    if (msg->len < dmaThreshold_ || dmaThreshold_ == 0)
    {
#if 0
        //SPIEnable(base);
        SPITransfer(base, (unsigned char*)msg->tx_buf,
                    (unsigned char*)msg->rx_buf, msg->len, 0);
        //SPIDisable(base);
#else
        uint32_t rx_len = len;
        long result;
        while (rx_len || len)
        {
            while (len)
            {
                unsigned long data = tx_buf ? *((unsigned char*)tx_buf) : 0xFF;
                result = SPIDataPutNonBlocking(base, data);
                if (result == 0)
                {
                    break;
                }
                if (tx_buf)
                {
                    ++tx_buf;
                }
                --len;
            }

            while (rx_len)
            {
                unsigned long data;
                result = SPIDataGetNonBlocking(base, &data);
                if (result == 0)
                {
                    break;
                }
                if (rx_buf)
                {
                    *((unsigned char*)rx_buf) = data;
                    ++rx_buf;
                }
                --rx_len;
            }
        }
#endif
    }
    else
    {
        uint32_t len = msg->len;
        unsigned long tx_buf = msg->tx_buf;
        unsigned long rx_buf = msg->rx_buf;
        while (len)
        {
            /* use DMA */
            void *buf;
            uint32_t channel_control_options;

            /*
             * The DMA has a max transfer amount of 1024.  If the transaction is
             * greater; we must transfer it in chunks.
             */
            dmaTransferSize_ = len > MAX_DMA_TRANSFER_AMOUNT ?
                                                  MAX_DMA_TRANSFER_AMOUNT : len;

            if (tx_buf)
            {
                channel_control_options = dmaTxConfig_;
                buf = (void*)tx_buf;
            }
            else
            {
                channel_control_options = dmaNullConfig_;
                buf = &scratch_buffer;
            }

            /* Setup the TX transfer characteristics & buffers */
            MAP_uDMAChannelControlSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                                      channel_control_options);
            MAP_uDMAChannelAttributeDisable(dmaChannelIndexTx_,
                                            UDMA_ATTR_ALTSELECT);
            MAP_uDMAChannelTransferSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC, buf,
                                       (void*)(base + MCSPI_O_TX0),
                                       dmaTransferSize_);

            if (rx_buf)
            {
                channel_control_options = dmaRxConfig_;
                buf = (void*)rx_buf;
            }
            else
            {
                channel_control_options = dmaNullConfig_;
                buf = &scratch_buffer;
            }

            /* Setup the RX transfer characteristics & buffers */
            MAP_uDMAChannelControlSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                                      channel_control_options);
            MAP_uDMAChannelAttributeDisable(dmaChannelIndexRx_,
                                            UDMA_ATTR_ALTSELECT);
            MAP_uDMAChannelTransferSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC,
                                       (void*)(base + MCSPI_O_RX0),
                                       buf, dmaTransferSize_);

            /* A lock is needed because we are accessing shared uDMA memory */
            portENTER_CRITICAL();

            MAP_uDMAChannelAssign(dmaChannelIndexTx_);
            MAP_uDMAChannelAssign(dmaChannelIndexRx_);

            /* Enable DMA to generate interrupt on SPI peripheral */
            MAP_SPIDmaEnable(base, SPI_RX_DMA | SPI_TX_DMA);
            MAP_SPIIntClear(base, SPI_INT_DMARX);
            MAP_SPIIntEnable(base, SPI_INT_DMARX);
            MAP_SPIWordCountSet(base, dmaTransferSize_);

            /* Enable channels & start DMA transfers */
            MAP_uDMAChannelEnable(dmaChannelIndexTx_);
            MAP_uDMAChannelEnable(dmaChannelIndexRx_);

            portEXIT_CRITICAL();

            MAP_SPIEnable(base);
            sem_.wait();

            len -= dmaTransferSize_;
            if (tx_buf)
            {
                tx_buf += dmaTransferSize_;
            }
            if (rx_buf)
            {
                rx_buf += dmaTransferSize_;
            }
        }
    }

    return msg->len;
}

/** Update the configuration of the bus.
 * @return >= 0 upon success, -errno upon failure
 */
int CC32xxSPI::update_configuration()
{
    unsigned long new_mode;
    unsigned long bits_per_word;

    switch (mode)
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

    MAP_SPIDisable(base);
    MAP_SPIReset(base);
    MAP_SPIConfigSetExpClk(base, clock, speedHz, SPI_MODE_MASTER, new_mode,
                           (SPI_3PIN_MODE | SPI_TURBO_OFF | bits_per_word));
    MAP_SPIFIFOEnable(base, SPI_RX_FIFO | SPI_TX_FIFO);
    MAP_SPIFIFOLevelSet(base, 1, 1);
    MAP_IntPrioritySet(interrupt, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(interrupt);
    MAP_SPIEnable(base);

    return 0;
}


/// Debugging helper.
volatile uint32_t g_error;
/// Debugging helper.
volatile uint32_t g_status;

/** Common interrupt handler for all SPI devices.
 */
__attribute__((optimize("-O3")))
void CC32xxSPI::interrupt_handler()
{
    int woken = 0;
#if 0
    volatile unsigned long raw_status = SPIIntStatus(base, false);
    volatile unsigned long mask_status = SPIIntStatus(base, true);

    (void)raw_status;
    (void)mask_status;
#endif
    if (MAP_uDMAChannelIsEnabled(dmaChannelIndexRx_))
    {
           /* DMA has not completed if the channel is still enabled */
           return;
    }

    /* RX DMA channel has completed */
    MAP_SPIDmaDisable(base, SPI_RX_DMA | SPI_TX_DMA);
    MAP_SPIIntDisable(base, SPI_INT_DMARX);
    MAP_SPIIntClear(base, SPI_INT_DMARX);
    MAP_SPIDisable(base);

    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** SPI0 interrupt handler.
 */
__attribute__((optimize("-O3")))
void spi0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}
} // extern C
