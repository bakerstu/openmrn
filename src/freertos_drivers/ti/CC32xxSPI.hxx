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
 * \file CC32xxSPI.hxx
 * This file implements a SPI device driver layer specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 30 May 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXSPI_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXSPI_HXX_

#ifndef gcc
#define gcc
#endif

#include <cstdint>

#include "SPI.hxx"

#include "inc/hw_mcspi.h"
#include "inc/hw_udma.h"
#include "driverlib/udma.h"

/** Specialization of Serial SPI driver for CC32xx devices.
 */
class CC32xxSPI : public SPI
{
public:
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
     * @param dma_threshold the threshold in bytes to use a DMA transaction,
     *                      0 = DMA always disabled
     * @param dma_channel_index_tx UDMA_CH7_GSPI_TX or UDMA_CH31_GSPI_TX
     * @param dma_channel_index_rx UDMA_CH6_GSPI_RX or UDMA_CH30_GSPI_RX
     */
    CC32xxSPI(const char *name, unsigned long base, uint32_t interrupt,
            ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
            OSMutex *bus_lock = nullptr, size_t dma_threshold = 0,
            uint32_t dma_channel_index_tx = UDMA_CH7_GSPI_TX,
            uint32_t dma_channel_index_rx = UDMA_CH6_GSPI_RX);

    /** Destructor.
     */
    ~CC32xxSPI()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    /** Maximum number of bytes transferred in a single DMA transaction */
    static constexpr size_t MAX_DMA_TRANSFER_AMOUNT = 1024;

    /**
     * This lookup table is used to configure the DMA channels for the appropriate
     * (8bit, 16bit or 32bit) transfer sizes.
     * Table for an SPI DMA RX channel.
     */
    static constexpr uint32_t dmaRxConfig_ =
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8  | UDMA_ARB_1;

    /**
     * This lookup table is used to configure the DMA channels for the appropriate
     * (8bit, 16bit or 32bit) transfer sizes.
     * Table for an SPI DMA TX channel
     */
    static constexpr uint32_t dmaTxConfig_ =
        UDMA_SIZE_8  | UDMA_SRC_INC_8  | UDMA_DST_INC_NONE | UDMA_ARB_1;

    /**
     * This lookup table is used to configure the DMA channels for the appropriate
     * (8bit, 16bit or 32bit) transfer sizes when either txBuf or rxBuf are NULL.
     */
    static constexpr uint32_t dmaNullConfig_ =
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1;

    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

    /** Method to transmit/receive the data.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    int transfer(struct spi_ioc_transfer *msg) override;

    /** Conduct multiple message transfers with one stop at the end.
     * @param msgs array of messages to transfer
     * @param num number of messages to transfer
     * @return total number of bytes transfered, -errno upon failure
     */
    int transfer_messages(struct spi_ioc_transfer *msgs, int num) override;

    /** Update the configuration of the bus.
     * @return >= 0 upon success, -errno upon failure
     */
    int update_configuration() override;

    /** Configure a DMA transaction.
     * @param from_isr true if called from an ISR, else false
     */
    inline void config_dma(bool from_isr);

    unsigned long base; /**< base address of this device */
    unsigned long clock; /**< clock rate supplied to the module */
    unsigned long interrupt; /**< interrupt of this device */
    size_t dmaThreshold_; /**< threshold in bytes to start using DMA */
    uint32_t dmaChannelIndexTx_; /**< TX DMA channel index */
    uint32_t dmaChannelIndexRx_; /**< RX DMA channel index */

    /** Semaphore to wakeup task level from ISR */
    OSSem sem_;

    /** a pointer to the message sequence the DMA is working on */
    volatile spi_ioc_transfer *dmaMsg;

    /** total length of the agrigate DMA segments in a message sequence */
    volatile uint32_t dmaMsgNum;

    /** Default constructor.
     */
    CC32xxSPI();

    DISALLOW_COPY_AND_ASSIGN(CC32xxSPI);
};

/** Configure a DMA transaction.
 * @param from_isr true if called from an ISR, else false
 */
__attribute__((optimize("-O3")))
inline void CC32xxSPI::config_dma(bool from_isr)
{
    static uint32_t scratch_buffer __attribute__((aligned(4))) = 0;

    /* use DMA */
    void *buf;
    uint32_t channel_control_options;

    /** @todo support longer SPI transactions */
    //HASSERT(dmaMsg->len <= MAX_DMA_TRANSFER_AMOUNT);

    if (dmaMsg->tx_buf)
    {
        channel_control_options = dmaTxConfig_;
        buf = (void*)dmaMsg->tx_buf;
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
                               (void*)(base + MCSPI_O_TX0), dmaMsg->len);

    if (dmaMsg->rx_buf)
    {
        channel_control_options = dmaRxConfig_;
        buf = (void*)dmaMsg->rx_buf;
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
                               (void*)(base + MCSPI_O_RX0), buf, dmaMsg->len);

    /* A lock is needed because we are accessing shared uDMA memory */
    //if (!from_isr)
    {
        //portENTER_CRITICAL();
    }

    /* Globally disables interrupts. */
    asm("cpsid i\n");

    uDMAChannelAssign(dmaChannelIndexRx_);
    uDMAChannelAssign(dmaChannelIndexTx_);

    /* assert chip select */
    csAssert();

    /* Enable DMA to generate interrupt on SPI peripheral */
    //SPIDmaEnable(base, SPI_RX_DMA | SPI_TX_DMA);
    //SPIIntClear(base, SPI_INT_DMARX);
    //SPIIntEnable(base, SPI_INT_DMARX);
    //SPIWordCountSet(base, dmaMsg->len);

    /* Enable channels & start DMA transfers */
    uDMAChannelEnable(dmaChannelIndexTx_);
    uDMAChannelEnable(dmaChannelIndexRx_);

    //if (!from_isr)
    {
        //portEXIT_CRITICAL();
    }
    asm("cpsie i\n");

    //MAP_SPIEnable(base);
}

#endif /* _FREERTOS_DRIVERS_TI_CC32XXSPI_HXX_ */
