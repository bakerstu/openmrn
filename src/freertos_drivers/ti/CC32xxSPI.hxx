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
#include "inc/hw_types.h"
#include "driverlib/udma.h"

#include <ti/drivers/dma/UDMACC32XX.h>

/** Specialization of Serial SPI driver for CC32xx devices.
 */
class CC32xxSPI : public SPI, private Atomic
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
            OSMutex *bus_lock = nullptr,
            size_t dma_threshold = DEFAULT_DMA_THRESHOLD_BYTES,
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

    /** This method provides a reference to the Mutex used by this device
     * driver.  It can be passed into another SPI driver instance as a bus
     * wide lock such that a SPI bus can be shared between this driver and
     * another use case with another chip select.
     * @return a reference to this device driver instance lock
     */
    OSMutex *get_lock()
    {
        return &lock_;
    }

private:
    static constexpr size_t DEFAULT_DMA_THRESHOLD_BYTES = 64;

    /** Maximum number of bytes transferred in a single DMA transaction */
    static constexpr size_t MAX_DMA_TRANSFER_AMOUNT = 1024;

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit, 16bit or 32bit) transfer sizes.
     * Table for an SPI DMA RX channel.
     */
    static constexpr uint32_t dmaRxConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8  | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1,
        UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1
    };

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit, 16bit or 32bit) transfer sizes.
     * Table for an SPI DMA TX channel
     */
    static constexpr uint32_t dmaTxConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_8  | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_NONE | UDMA_ARB_1
    };

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit, 16bit or 32bit) transfer sizes when either txBuf or
     * rxBuf are NULL.
     */
    static constexpr uint32_t dmaNullConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1
    };

    /** Function to enable device.
     */
    void enable() override
    {
        dmaHandle_ = UDMACC32XX_open();
    }

    /** Function to disable device.
     */
    void disable() override
    {
        UDMACC32XX_close(dmaHandle_);
    }

    /** Update the configuration of the bus.
     * @return >= 0 upon success, -errno upon failure
     */
    int update_configuration() override;

    /** Method to transmit/receive the data.
     * @param msg message to transact.
     */
    __attribute__((optimize("-O3")))
    int transfer(struct spi_ioc_transfer *msg) override
    {
        if (LIKELY(msg->len < dmaThreshold_))
        {
            return transfer_polled(msg);
        }
        else
        {
            /* use DMA */
            config_dma(msg);
        }

        return msg->len;
    }

    /** Method to transmit/receive the data. This is a template in order to
     * preserve execution speed on type specific pointer math.
     * @param msg message to transact.
     */
    template<typename T>
    __attribute__((optimize("-O3")))
    int transfer_polled(struct spi_ioc_transfer *msg)
    {
        T dummy = 0;

        /* we are assuming that at least one byte will be transferred, and
         * we want to start tranfering data as soon as possible
         */
        data_put(msg->tx_buf ? *((T*)msg->tx_buf) : 0xFFFFFFFF);

        T *tx_buf = msg->tx_buf ? ((T*)msg->tx_buf) + 1 : &dummy;
        T *rx_buf = (T*)msg->rx_buf;

        /* note that we already have transmitted one SPI word above, hence the
         * subtract one from the tx_len
         */
        uint32_t rx_len = msg->len / sizeof(T);
        uint32_t tx_len = rx_len - 1;

        unsigned long data;

        do
        {
            /* fill TX FIFO but make sure we don't fill it to overflow */
            if (tx_len && ((rx_len - tx_len) < (32 / sizeof(T))))
            {
                if (data_put_non_blocking(*tx_buf) != 0)
                {
                    if (msg->tx_buf)
                    {
                        ++tx_buf;
                    }
                    --tx_len;
                }
            }

            /* empty RX FIFO */
            if (rx_len)
            {
                if (data_get_non_blocking(&data) != 0)
                {
                    if (msg->rx_buf)
                    {
                        *rx_buf++ = data;
                    }
                    --rx_len;
                }
            }
        }
        while (tx_len || rx_len);

        return msg->len;
    }

    /** Method to transmit/receive the data.
     * @param msg message to transact.
     */
    __attribute__((optimize("-O3")))
    int transfer_polled(struct spi_ioc_transfer *msg) override
    {
        /* set instance specific configuration */
        set_configuration();

        switch (bitsPerWord)
        {
            default:
            case 8:
                return transfer_polled<uint8_t>(msg);
            case 16:
                return transfer_polled<uint16_t>(msg);
            case 32:
                return transfer_polled<uint32_t>(msg);
        }
    }

    /** Configure a DMA transaction.
     * @param msg message to transact.
     */
    void config_dma(struct spi_ioc_transfer *msg);

    /** Receives a word from the specified port.  This function gets a SPI word
     * from the receive FIFO for the specified port.
     * @param data is pointer to receive data variable.
     * @return Returns the number of elements read from the receive FIFO.
     */
    __attribute__((optimize("-O3")))
    long data_get_non_blocking(unsigned long *data)
    {
        if(HWREG(base_ + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_RXS)
        {
            *data = HWREG(base_ + MCSPI_O_RX0);
            return 1;
        }

        return 0;
    }

    /** Transmits a word on the specified port.  This function transmits a SPI
     * word on the transmit FIFO for the specified port.
     * @param data is data to be transmitted.
     * @return Returns the number of elements written to the transmit FIFO.
     */
    __attribute__((optimize("-O3")))
    long data_put_non_blocking(unsigned long data)
    {
        if(HWREG(base_ + MCSPI_O_CH0STAT) & MCSPI_CH0STAT_TXS)
        {
            HWREG(base_ + MCSPI_O_TX0) = data;
            return 1;
        }

        return 0;
    }

    /** Waits until the word is transmitted on the specified port.  This
     * function transmits a SPI word on the transmit FIFO for the specified
     * port. This function waits until the space is available on transmit FIFO.
     * @param data is data to be transmitted.
     */
    void data_put(unsigned long data)
    {
        while(UNLIKELY(!(HWREG(base_ + MCSPI_O_CH0STAT)&MCSPI_CH0STAT_TXS)));

        HWREG(base_ + MCSPI_O_TX0) = data;
    }

    /** Set the instance local configuration.
     */
    void set_configuration()
    {
        HWREG(base_ + MCSPI_O_CH0CTRL) = spiChctrl_;
        HWREG(base_ + MCSPI_O_CH0CONF) = spiChconf_;
        HWREG(base_ + MCSPI_O_XFERLEVEL) = spiXferlevel_;
    }

    UDMACC32XX_Handle dmaHandle_; /**< handle to DMA reference */
    OSSem *sem_; /**< reference to the semaphore belonging to this bus */
    unsigned long base_; /**< base address of this device */
    unsigned long clock_; /**< clock rate supplied to the module */
    unsigned long interrupt_; /**< interrupt of this device */
    size_t dmaThreshold_; /**< threshold in bytes to start using DMA */
    uint32_t dmaChannelIndexTx_; /**< TX DMA channel index */
    uint32_t dmaChannelIndexRx_; /**< RX DMA channel index */
    uint32_t spiChctrl_; /**< instance local copy of configuration */
    uint32_t spiChconf_; /**< instance local copy of configuration */
    uint32_t spiXferlevel_; /**< instance local copy of configuration */

    /** Default constructor.
     */
    CC32xxSPI();

    DISALLOW_COPY_AND_ASSIGN(CC32xxSPI);
};

#endif /* _FREERTOS_DRIVERS_TI_CC32XXSPI_HXX_ */
