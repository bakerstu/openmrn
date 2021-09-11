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
 * \file TivaSPI.hxx
 * This file implements a SPI device driver for Tiva (129).
 *
 * @author Balazs Racz
 * @date 18 Aug 2021
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVASPI_HXX_
#define _FREERTOS_DRIVERS_TI_TIVASPI_HXX_


#include "SPI.hxx"
#include "driverlib/ssi.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/udma.h"

class TivaSPI : public SPI
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
     * @param dma_channel_index_tx UDMA_CHn_SSInTX
     * @param dma_channel_index_rx UDMA_CHn_SSInRX
     */
    TivaSPI(const char *name, unsigned long base, uint32_t interrupt,
            ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
            OSMutex *bus_lock = nullptr,
            size_t dma_threshold = DEFAULT_DMA_THRESHOLD_BYTES,
            uint32_t dma_channel_index_tx = UDMA_CH11_SSI0TX,
            uint32_t dma_channel_index_rx = UDMA_CH10_SSI0RX);
    
    /// Destructor
    ~TivaSPI();

    /// Call this from the SPI device's interrupt handler.
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
    /// Transfers longer than this will be with dma by default.
    /// @todo this should be 64 bytes
    static constexpr size_t DEFAULT_DMA_THRESHOLD_BYTES = 0;

    /// Maximum number of bytes transferred in a single DMA transaction
    static constexpr size_t MAX_DMA_TRANSFER_AMOUNT = 1024;

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit or 16bit) transfer sizes.
     * Table for an SPI DMA RX channel.
     */
    static constexpr uint32_t dmaRxConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8  | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1,
    };

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit or 16bit) transfer sizes.
     * Table for an SPI DMA TX channel
     */
    static constexpr uint32_t dmaTxConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_8  | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_1,
    };

    /**
     * This lookup table is used to configure the DMA channels for the
     * appropriate (8bit or 16bit) transfer sizes when either txBuf or
     * rxBuf are NULL.
     */
    static constexpr uint32_t dmaNullConfig_[] =
    {
        UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1,
    };
    
    /** Function to enable device.
     */
    void enable() override;

    /** Function to disable device.
     */
    void disable() override;

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
        return transfer_polled(msg);
#if 0
        /// @todo support DMA
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
#endif        
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

        uint16_t data;

        do
        {
            /* fill TX FIFO but make sure we don't fill it to overflow */
            if (tx_len && ((rx_len - tx_len) < 8))
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
                DIE("32-bit transers are not supported on this MCU.");
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
    long data_get_non_blocking(uint16_t *data)
    {
        if(HWREG(base_ + SSI_O_SR) & SSI_SR_RNE)
        {
            *data = HWREG(base_ + SSI_O_DR);
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
    long data_put_non_blocking(uint16_t data)
    {
        if(HWREG(base_ + SSI_O_SR) & SSI_SR_TNF)
        {
            HWREG(base_ + SSI_O_DR) = data;
            return 1;
        }

        return 0;
    }

    /** Waits until the word is transmitted on the specified port.  This
     * function transmits a SPI word on the transmit FIFO for the specified
     * port. This function waits until the space is available on transmit FIFO.
     * @param data is data to be transmitted.
     */
    void data_put(uint16_t data)
    {
        while((HWREG(base_ + SSI_O_SR) & SSI_SR_TNF) == 0);
        HWREG(base_ + SSI_O_DR) = data;
    }

    /** Set the instance local configuration.
     */
    void set_configuration()
    {
        // Disable port first for reconfiguration.
        HWREG(base_ + SSI_O_CR1) &= ~SSI_CR1_SSE;
        
        HWREG(base_ + SSI_O_CR0) = spiCr0_;
        HWREG(base_ + SSI_O_CPSR) = spiPrescaler_;
        HWREG(base_ + SSI_O_CR1) = spiCr1_;
    }

    OSSem *sem_; /**< reference to the semaphore belonging to this bus */
    unsigned long base_; /**< base address of this device */
    unsigned long interrupt_; /**< interrupt of this device */
    size_t dmaThreshold_; /**< threshold in bytes to start using DMA */
    uint32_t dmaChannelIndexTx_; /**< TX DMA channel index */
    uint32_t dmaChannelIndexRx_; /**< RX DMA channel index */
    
    uint16_t spiCr0_;  ///< Configuration register 0 local copy.
    uint16_t spiCr1_;  ///< Configuration register 1 local copy.
    uint16_t spiPrescaler_;  ///< Prescale register local copy.

    DISALLOW_COPY_AND_ASSIGN(TivaSPI);
};


#endif // _FREERTOS_DRIVERS_TI_TIVASPI_HXX_
