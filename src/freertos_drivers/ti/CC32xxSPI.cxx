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

/** One semaphore required per instance pointer */
OSSem sem[1];

constexpr uint32_t CC32xxSPI::dmaRxConfig_[];
constexpr uint32_t CC32xxSPI::dmaTxConfig_[];
constexpr uint32_t CC32xxSPI::dmaNullConfig_[];

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
    , dmaHandle_(nullptr)
    , base_(base)
    , interrupt_(interrupt)
    , dmaThreshold_(dma_threshold)
    , dmaChannelIndexTx_(dma_channel_index_tx)
    , dmaChannelIndexRx_(dma_channel_index_rx)
{
    switch (base_)
    {
        default:
            HASSERT(0);
        case GSPI_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
            MAP_PRCMPeripheralReset(PRCM_GSPI);
            clock_ = MAP_PRCMPeripheralClockGet(PRCM_GSPI);
            UDMACC32XX_init();
            instances_[0] = this;
            sem_ = &sem[0];
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
    unsigned long fifo_level;

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
            bitsPerWord = 8;
            // fall through
        case 8:
            bits_per_word = SPI_WL_8;
            fifo_level = sizeof(uint8_t);
            break;
        case 16:
            bits_per_word = SPI_WL_16;
            fifo_level = sizeof(uint16_t);
            break;
        case 32:
            bits_per_word = SPI_WL_32;
            fifo_level = sizeof(uint32_t);
            break;
    }

    /* The SPI peripheral only supoprts certain frequencies and driverlib does
     * not properly round down. We can do some math to make sure that driverlib
     * makes the correct selection.
     */
    if (speedHz > (clock_ / 2))
    {
        speedHz = clock_ / 2;
    }
    else if ((clock_ % speedHz) != 0)
    {
        speedHz = clock_ / ((clock_ / speedHz) + 1);
    }

    MAP_SPIDisable(base_);
    MAP_SPIReset(base_);
    MAP_SPIConfigSetExpClk(base_, clock_, speedHz, SPI_MODE_MASTER, new_mode,
                           (SPI_3PIN_MODE | SPI_TURBO_ON | bits_per_word));
    MAP_SPIFIFOEnable(base_, SPI_RX_FIFO | SPI_TX_FIFO);
    MAP_SPIFIFOLevelSet(base_, fifo_level, fifo_level);
    MAP_IntPrioritySet(interrupt_, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(interrupt_);
    MAP_SPIEnable(base_);

    // these values are used to quickly program instance local configuration
    // settings
    spiChctrl_ = HWREG(base_ + MCSPI_O_CH0CTRL);
    spiChconf_ = HWREG(base_ + MCSPI_O_CH0CONF);
    spiXferlevel_ = HWREG(base_ + MCSPI_O_XFERLEVEL);

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
    unsigned config_index;
    unsigned items;

    /* set instance specific configuration */
    set_configuration();

    /** @todo We could potentially perform the first daata write immediately
     *        and only use DMA for n-1 words of TX. This would reduce the
     *        latency of the transaction slightly.
     */

    switch (bitsPerWord)
    {
        default:
        case 8:
            config_index = 0;
            items = msg->len / sizeof(uint8_t);
            break;
        case 16:
            config_index = 1;
            items = msg->len / sizeof(uint16_t);
            break;
        case 32:
            config_index = 2;
            items = msg->len / sizeof(uint32_t);
            break;
    }

    /** @todo support longer SPI transactions */
    HASSERT(items <= MAX_DMA_TRANSFER_AMOUNT);

    if (msg->tx_buf)
    {
        channel_control_options = dmaTxConfig_[config_index];
        buf = (void*)msg->tx_buf;
    }
    else
    {
        channel_control_options = dmaNullConfig_[config_index];
        buf = &scratch_buffer;
    }

    /* Setup the TX transfer characteristics & buffers */
    uDMAChannelControlSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                          channel_control_options);
#if 0
    // This disabled section is provided as a driverlib reference to the
    // faster enabled (#else) version inline.
    uDMAChannelAttributeDisable(dmaChannelIndexTx_, UDMA_ATTR_ALTSELECT);
#else
    HWREG(UDMA_BASE + UDMA_O_ALTCLR) = 1 << (dmaChannelIndexTx_ & 0x1f);
#endif
    uDMAChannelTransferSet(dmaChannelIndexTx_ | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC, buf, (void*)(base_ + MCSPI_O_TX0),
                           items);

    if (msg->rx_buf)
    {
        channel_control_options = dmaRxConfig_[config_index];
        buf = (void*)msg->rx_buf;
    }
    else
    {
        channel_control_options = dmaNullConfig_[config_index];
        buf = &scratch_buffer;
    }

    /* Setup the RX transfer characteristics & buffers */
    uDMAChannelControlSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                              channel_control_options);
#if 0
    // This disabled section is provided as a driverlib reference to the
    // faster enabled (#else) version inline.
    uDMAChannelAttributeDisable(dmaChannelIndexRx_,  UDMA_ATTR_ALTSELECT);
#else
    HWREG(UDMA_BASE + UDMA_O_ALTCLR) = 1 << (dmaChannelIndexRx_ & 0x1f);
#endif
    uDMAChannelTransferSet(dmaChannelIndexRx_ | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC, (void*)(base_ + MCSPI_O_RX0), buf,
                           items);

    {
        AtomicHolder h(this);

        uDMAChannelAssign(dmaChannelIndexRx_);
        uDMAChannelAssign(dmaChannelIndexTx_);

        SPIDmaEnable(base_, SPI_RX_DMA | SPI_TX_DMA);
        SPIIntClear(base_, SPI_INT_DMARX);
        SPIIntEnable(base_, SPI_INT_DMARX);

        /* Enable channels & start DMA transfers */
        uDMAChannelEnable(dmaChannelIndexTx_);
        uDMAChannelEnable(dmaChannelIndexRx_);
    }

    sem_->wait();
}

/** Common interrupt handler for all SPI devices.
 */
__attribute__((optimize("-O3")))
void CC32xxSPI::interrupt_handler()
{
    // Note: There can be more than one CC32xxSPI instance sharing a single
    //       bus. However, the fact that in this case the base_ address and
    //       sem_ reference will also be shared, we can exploit the fact that
    //       the "this" pointer may not directly correspond to the instance
    //       that is waiting using sem_->wait().

    if (SPIIntStatus(base_, true) & SPI_INT_DMATX)
    {
        SPIIntDisable(base_, SPI_INT_DMATX);
        SPIIntClear(base_, SPI_INT_DMATX);
    }
    if (uDMAChannelIsEnabled(dmaChannelIndexRx_))
    {
        /* DMA has not completed if the channel is still enabled */
        return;
    }

    SPIDmaDisable(base_, SPI_RX_DMA | SPI_TX_DMA);
    SPIIntDisable(base_, SPI_INT_DMARX);

    int woken = 0;
    sem_->post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** SPI0 interrupt handler.
 */
__attribute__((optimize("-O3")))
void spi0_interrupt_handler(void)
{
    instances_[0]->interrupt_handler();
}
} // extern C
