/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file Stm32Railcom.hxx
 *
 * Device driver for STM32 chips to read one or more UART inputs for railcom
 * data.
 *
 * @author Balazs Racz
 * @date 6 Mar 2023
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32RAILCOM_HXX_
#define _FREERTOS_DRIVERS_ST_STM32RAILCOM_HXX_

#include "stm32f_hal_conf.hxx"

#include "freertos_drivers/common/RailcomImpl.hxx"

#if defined(STM32F072xB) || defined(STM32F091xC)
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_usart.h"
#elif defined(STM32G0B1xx)
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_usart.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#elif defined(STM32F303xC) || defined(STM32F303xE)
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_usart.h"
#elif defined(STM32L431xx) || defined(STM32L432xx)
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#elif defined(STM32F767xx)
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_usart.h"
#else
#error Dont know what STM32 chip you have.
#endif

/// This struct helps casting the UART base addresses to the appropriate
/// type. It can be allocated in read-only memory (flash) as a static array. It
/// can be passed to the STM32 HAL macros as well.
struct RailcomUartHandle
{
    union
    {
        /// Initialized by the constants from the processor header like
        /// USART1_BASE.
        uint32_t baseAddress;
        /// Use this to access the registers.
        USART_TypeDef *Instance;
    };
};

/// This struct helps casting the DMA channel base addresses to the appropriate
/// type. It can be allocated in read-only memory (flash) as a static array.
struct RailcomDmaChannel
{
    union
    {
        /// Initialized by the constants from the processor header like
        /// USART1_BASE.
        uint32_t baseAddress;
        /// Use this to access the registers.
        DMA_Channel_TypeDef *Instance;
    };
};

/*
struct RailcomHw
{
    static const uint32_t CHANNEL_COUNT = 4;
    static const RailcomUartHandle UART[CHANNEL_COUNT];

    // If DMA channel routing is in use, that has to be set up externally
    // (e.g. in hw_preinit).
    static const RailcomDmaChannel DMA[CHANNEL_COUNT];

    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 32;

    static const auto OS_INTERRUPT = USART2_IRQn;

    GPIO_HWPIN(CH1, GpioHwPin, C, 4, U4RX);
    GPIO_HWPIN(CH2, GpioHwPin, C, 6, U3RX);
    GPIO_HWPIN(CH3, GpioHwPin, G, 4, U2RX);
    GPIO_HWPIN(CH4, GpioHwPin, E, 0, U7RX);

    static void hw_init() {
         CH1_Pin::hw_init();
         CH2_Pin::hw_init();
         CH3_Pin::hw_init();
         CH4_Pin::hw_init();
    }

    static void set_input() {
        CH1_Pin::set_input();
        CH2_Pin::set_input();
        CH3_Pin::set_input();
        CH4_Pin::set_input();
    }

    static void set_hw() {
        CH1_Pin::set_hw();
        CH2_Pin::set_hw();
        CH3_Pin::set_hw();
        CH4_Pin::set_hw();
    }

    /// @returns a bitmask telling which pins are active. Bit 0 will be set if
    /// channel 0 is active (drawing current).
    static uint8_t sample() {
        uint8_t ret = 0;
        if (!CH1_Pin::get()) ret |= 1;
        if (!CH2_Pin::get()) ret |= 2;
        if (!CH3_Pin::get()) ret |= 4;
        if (!CH4_Pin::get()) ret |= 8;
        return ret;
    }
};

// The weak attribute is needed if the definition is put into a header file.
const uint32_t RailcomHw::UART[] __attribute__((weak)) = { USART4_BASE,
USART1_BASE, USART3_BASE, USART2_BASE };

const RailcomDmaChannel RailcomHw::DMA[] __attribute__((weak)) = {
DMA1_Channel1_BASE, DMA1_Channel3_BASE, DMA1_Channel6_BASE, DMA1_Channel5_BASE
};

In hw_preinit(), we have this for DMA channel routing:
    //DMA channel routing for railcom UARTs
    MODIFY_REG(DMA1->CSELR, DMA_CSELR_C1S_Msk, DMA1_CSELR_CH1_USART4_RX);
    MODIFY_REG(DMA1->CSELR, DMA_CSELR_C3S_Msk, DMA1_CSELR_CH3_USART1_RX);
    MODIFY_REG(DMA1->CSELR, DMA_CSELR_C6S_Msk, DMA1_CSELR_CH3_USART3_RX);
    MODIFY_REG(DMA1->CSELR, DMA_CSELR_C5S_Msk, DMA1_CSELR_CH3_USART2_RX);

*/
/// Railcom driver for STM32-class microcontrollers using the HAL middleware
/// library.
///
/// This railcom driver supports parallel polling of multiple UART channels for
/// the railcom data.
template <class HW> class Stm32RailcomDriver : public RailcomDriverBase<HW>
{
public:
    /// Constructor. @param path is the device node path (e.g. "/dev/railcom0").
    Stm32RailcomDriver(const char *path)
        : RailcomDriverBase<HW>(path)
    {
#ifdef GCC_CM3
        SetInterruptPriority(HW::OS_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
#else
        SetInterruptPriority(HW::OS_INTERRUPT, 0xff);
#endif
        HAL_NVIC_EnableIRQ(HW::OS_INTERRUPT);
    }

    ~Stm32RailcomDriver()
    {
        HAL_NVIC_DisableIRQ(HW::OS_INTERRUPT);
    }

private:
    /// True when we are currently within a cutout.
    bool inCutout_ = false;

    using RailcomDriverBase<HW>::returnedPackets_;

    /// @param i channel number (0..HW::NUM_CHANNEL)
    /// @return uart hardware instance for channel i.
    static constexpr USART_TypeDef *uart(unsigned i)
    {
        return HW::UART[i].Instance;
    }

    /// @param i channel number (0..HW::NUM_CHANNEL)
    /// @return dma channel instance for channel i.
    static constexpr DMA_Channel_TypeDef *dma_ch(unsigned i)
    {
        return HW::DMA[i].Instance;
    }

    /// Sets a given software interrupt pending.
    /// @param int_nr interrupt number (will be HW::OS_INTERRUPT)
    void int_set_pending(unsigned int_nr) override
    {
        HAL_NVIC_SetPendingIRQ((IRQn_Type)int_nr);
    }

    // File node interface
    void enable() override
    {
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            UART_HandleTypeDef uart_handle;
            memset(&uart_handle, 0, sizeof(uart_handle));
            uart_handle.Init.BaudRate = 250000;
            uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
            uart_handle.Init.StopBits = UART_STOPBITS_1;
            uart_handle.Init.Parity = UART_PARITY_NONE;
            uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
            uart_handle.Init.Mode = UART_MODE_RX;
            uart_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
            uart_handle.Instance = uart(i);
            HAL_UART_DeInit(&uart_handle);
            volatile auto ret = HAL_UART_Init(&uart_handle);
            HASSERT(HAL_OK == ret);

            // Disables the receiver.
            LL_USART_SetTransferDirection(uart(i), LL_USART_DIRECTION_NONE);

            // configure DMA

            // peripheral address
            dma_ch(i)->CPAR = (uint32_t)(&uart(i)->RDR);

            // memory address and num transfers not set, these will come from
            // each cutout.
            // dma_ch->CMAR = (uint32_t)adcSamplesRaw_;
            // dma_ch->CNDTR = NUM_SAMPLES * NUM_CHANNELS;

            uint32_t config = LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT |
                LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_HIGH;
            MODIFY_REG(dma_ch(i)->CCR,
                DMA_CCR_DIR | DMA_CCR_MEM2MEM | DMA_CCR_CIRC | DMA_CCR_PINC |
                    DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE | DMA_CCR_PL |
                    DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE,
                config);

            LL_USART_Enable(uart(i));
        }
    }
    void disable() override
    {
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            LL_USART_Disable(uart(i));
        }
    }

    // RailcomDriver interface
    void feedback_sample() override
    {
        HW::enable_measurement(true);
        auto s = HW::sample();
        HW::disable_measurement();
        this->add_sample(s);
    }

    void start_cutout() override
    {
        Debug::RailcomRxActivate::set(true);
        HW::enable_measurement(false);
        const bool need_ch1_cutout =
            HW::need_ch1_cutout() || (this->feedbackKey_ < 11000);
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            while (LL_USART_IsActiveFlag_RXNE(uart(i)))
            {
                uint8_t data = uart(i)->RDR;
                (void)data;
            }
            if (!returnedPackets_[i])
            {
                returnedPackets_[i] = this->alloc_new_packet(i);
            }
            if (need_ch1_cutout && returnedPackets_[i])
            {
                LL_USART_EnableDMAReq_RX(uart(i));
                LL_USART_SetTransferDirection(uart(i), LL_USART_DIRECTION_RX);
                LL_USART_ClearFlag_FE(uart(i));
                LL_USART_Enable(uart(i));

                dma_ch(i)->CNDTR = 2;
                dma_ch(i)->CMAR = (uint32_t)returnedPackets_[i]->ch1Data;
                dma_ch(i)->CCR |= DMA_CCR_EN; // enable DMA
            }
        }
        Debug::RailcomDriverCutout::set(true);
    }

    void middle_cutout() override
    {
        Debug::RailcomDriverCutout::set(false);
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            if (!returnedPackets_[i])
            {
                continue;
            }
            LL_USART_Disable(uart(i));
            CLEAR_BIT(dma_ch(i)->CCR, DMA_CCR_EN);
            // How many bytes did we transfer?
            returnedPackets_[i]->ch1Size = 2 - dma_ch(i)->CNDTR;
            if (returnedPackets_[i]->ch1Size == 1)
            {
                Debug::RailcomError::toggle();
            }
            if (returnedPackets_[i]->ch1Size)
            {
                Debug::RailcomDataReceived::toggle();
                Debug::RailcomAnyData::set(true);
            }

            if (LL_USART_IsActiveFlag_FE(uart(i)))
            {
                // We reset the receiver circuitry because we got an error
                // in channel 1. Typical cause of this error is if there
                // are multiple locomotives on the block (e.g. if this is
                // the global detector) and they talk over each other
                // during ch1 broadcast. There is a good likelihood that
                // the asynchronous receiver is out of sync with the
                // transmitter, but right now we should be in the
                // between-byte space.
                LL_USART_SetTransferDirection(uart(i), LL_USART_DIRECTION_NONE);
                LL_USART_ClearFlag_FE(uart(i));
                Debug::RailcomError::toggle();
                // Not a valid railcom byte.
                returnedPackets_[i]->ch1Data[0] = 0xF8;
                returnedPackets_[i]->ch1Size = 1;
            }

            // Set up channel 2 reception with DMA.
            dma_ch(i)->CNDTR = 6;
            dma_ch(i)->CMAR = (uint32_t)returnedPackets_[i]->ch2Data;
            dma_ch(i)->CCR |= DMA_CCR_EN; // enable DMA

            LL_USART_SetTransferDirection(uart(i), LL_USART_DIRECTION_RX);
            LL_USART_ClearFlag_FE(uart(i));
            LL_USART_Enable(uart(i));
        }
        HW::middle_cutout_hook();
        Debug::RailcomDriverCutout::set(true);
    }

    void end_cutout() override
    {
        HW::disable_measurement();
        bool have_packets = false;
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            LL_USART_Disable(uart(i));
            if (!returnedPackets_[i])
            {
                continue;
            }
            CLEAR_BIT(dma_ch(i)->CCR, DMA_CCR_EN);
            // How many bytes did we transfer?
            returnedPackets_[i]->ch2Size = 6 - dma_ch(i)->CNDTR;
            if (returnedPackets_[i]->ch2Size)
            {
                Debug::RailcomDataReceived::toggle();
                Debug::RailcomAnyData::set(true);
                Debug::RailcomCh2Data::set(true);
            }
            if (LL_USART_IsActiveFlag_FE(uart(i)))
            {
                Debug::RailcomError::toggle();
                LL_USART_ClearFlag_FE(uart(i));
                if (returnedPackets_[i]->ch2Size < 6)
                {
                    returnedPackets_[i]->add_ch2_data(0xF8);
                }
            }

            LL_USART_SetTransferDirection(uart(i), LL_USART_DIRECTION_NONE);

            Debug::RailcomPackets::toggle();
            have_packets = true;
            this->feedbackQueue_.commit_back();
            returnedPackets_[i] = nullptr;
            HAL_NVIC_SetPendingIRQ(HW::OS_INTERRUPT);
        }
        Debug::RailcomRxActivate::set(false);
        if (!have_packets)
        {
            // Ensures that at least one feedback packet is sent back even when
            // it is with no railcom payload.
            no_cutout();
        }
        Debug::RailcomCh2Data::set(false);
        Debug::RailcomDriverCutout::set(false);
    }

    void no_cutout() override
    {
        // Ensures that at least one feedback packet is sent back even when
        // it is with no railcom payload.
        auto *p = this->alloc_new_packet(15);
        if (p)
        {
            this->feedbackQueue_.commit_back();
            Debug::RailcomPackets::toggle();
            HAL_NVIC_SetPendingIRQ(HW::OS_INTERRUPT);
        }
    }

    /// @copydoc RailcomDriver::set_feedback_key()
    ///
    /// This implementation also preallocates storage for returned packets for
    /// CHANNEL_COUNT entries. Since the packet allocation can take a few usec,
    /// doing it here, ahead of time can avoid running out of time starting
    /// the cutout.
    void set_feedback_key(uint32_t key) override
    {
        Debug::RailComAllocPacketTiming::set(true);
        RailcomDriverBase<HW>::set_feedback_key(key);
        for (unsigned i = 0; i < HW::CHANNEL_COUNT; ++i)
        {
            if (!returnedPackets_[i])
            {
                returnedPackets_[i] = this->alloc_new_packet(i);
            }
        }
        Debug::RailComAllocPacketTiming::set(false);
    }
};

#endif // _FREERTOS_DRIVERS_ST_STM32RAILCOM_HXX_
