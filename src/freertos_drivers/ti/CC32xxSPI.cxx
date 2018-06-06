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
#include "driverlib/udma.h"

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
    , dmaThreshold_(dma_threshold < DMA_THRESHOLD_BYTES ? dma_threshold :
                                                          DMA_THRESHOLD_BYTES)
    , dmaChannelIndexTx_(dma_channel_index_tx)
    , dmaChannelIndexRx_(dma_channel_index_rx)
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

/** Method to transmit/receive the data.
 * @param msg message(s) to transact.
 * @return bytes transfered upon success, -errno upon failure
 */
__attribute__((optimize("-O3")))
int CC32xxSPI::transfer(struct spi_ioc_transfer *msg)
{
    /** @todo It is assumed that 8 bit per word is used, need to extend */
    if (msg->len < dmaThreshold_)
    {
        uint8_t *tx_buf = msg->tx_buf ? (uint8_t*)msg->tx_buf : dummyBuf;
        uint8_t *rx_buf = msg->rx_buf ? (uint8_t*)msg->rx_buf : dummyBuf;
        unsigned long data;

#if 0
        uint32_t len = msg->len < FIFO_SIZE_BYTES ? msg->len : FIFO_SIZE_BYTES;
        SPIFIFOLevelSet(base, 1, len);
        SPIIntClear(base, SPI_INT_RX_FULL);

        for (unsigned i = 0; i < len; ++i, ++tx_buf)
        {
            data = tx_buf ? *((unsigned char*)tx_buf) : 0xFF;
            SPIDataPutNonBlocking(base, data);
        }

        if (msg->len > 4)
        {
            SPIIntEnable(base, SPI_INT_RX_FULL);
            sem_.wait();

            while (len--)
            {
                SPIDataGetNonBlocking(base, &data);
                if (rx_buf)
                {
                    *((unsigned char*)rx_buf) = data;
                    ++rx_buf;
                }
            }
        }
        else
        {
            long result;
            /* empty RX FIFO */
            while (len)
            {
                result = SPIDataGetNonBlocking(base, &data);
                if (result != 0)
                {
                    --len;
                    if (rx_buf)
                    {
                        *((unsigned char*)rx_buf) = data;
                        ++rx_buf;
                    }
                }
            }
        }
#else
        uint32_t tx_len = msg->len;
        uint32_t rx_len = msg->len;

        SPIDataPut(base, *tx_buf++);
        --tx_len;

        while (tx_len || rx_len)
        {
            /* fill TX FIFO */
            if (tx_len)
            {
                if (SPIDataPutNonBlocking(base, *tx_buf) != 0)
                {
                    --tx_len;
                    ++tx_buf;
                }
            }

            /* empty RX FIFO */
            if (rx_len)
            {
                if (SPIDataGetNonBlocking(base, &data) != 0)
                {
                    --rx_len;
                    *rx_buf++ = data;
                }
            }
        }
#endif
    }
    else
    {
        /* use DMA */
        config_dma(msg);
    }

    return msg->len;
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
                               (void*)(base + MCSPI_O_TX0), msg->len);

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
                               (void*)(base + MCSPI_O_RX0), buf, msg->len);

    /* Globally disables interrupts. */
    asm("cpsid i\n");

    uDMAChannelAssign(dmaChannelIndexRx_);
    uDMAChannelAssign(dmaChannelIndexTx_);

    SPIFIFOLevelSet(base, 1, 1);
    SPIDmaEnable(base, SPI_RX_DMA | SPI_TX_DMA);
    SPIIntClear(base, SPI_INT_DMARX);
    SPIIntEnable(base, SPI_INT_DMARX);

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
    if (SPIIntStatus(base, true) & SPI_INT_RX_FULL)
    {
        SPIIntDisable(base, SPI_INT_RX_FULL);
    }
    else
    {
        if (uDMAChannelIsEnabled(dmaChannelIndexRx_))
        {
            /* DMA has not completed if the channel is still enabled */
            return;
        }

        SPIDmaDisable(base, SPI_RX_DMA | SPI_TX_DMA);
        SPIIntDisable(base, SPI_INT_DMARX);
    }

    int woken = 0;
    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}


extern volatile unsigned current_interrupt;

class SetInterrupt {
public:
    SetInterrupt(unsigned new_value) {
        old_value = current_interrupt;
        current_interrupt = new_value;
    }

    ~SetInterrupt() {
        current_interrupt = old_value;
    }

private:
    unsigned old_value;
};

extern "C" {
/** SPI0 interrupt handler.
 */
__attribute__((optimize("-O3")))
void spi0_interrupt_handler(void)
{
    SetInterrupt si(11);
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}
} // extern C
