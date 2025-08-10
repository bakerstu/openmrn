/** \copyright
 * Copyright (c) 2025, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file Esp32Can.hxx
 *
 * Wrapper for the ESP32 TWAI driver fr arduino environments.
 *
 * @author Balazs Racz
 * @date 2 Aug 2025
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_

#if !defined(ESP_PLATFORM)
#error "This driver is for ESP32 only."
#endif

#if !defined(ARDUINO)
#error "This driver is for Arduino environment only."
#endif

#include <string.h>
#include "driver/twai.h"
#include "freertos_drivers/common/Can.hxx"

/// ESP32 TWAI CAN driver.
class Esp32Can : public openmrn_arduino::Can
{
public:
    /// Constructor.
    /// @param tx_pin GPIO pin for CAN TX.
    /// @param rx_pin GPIO pin for CAN RX.
    Esp32Can(gpio_num_t tx_pin, gpio_num_t rx_pin)
        : openmrn_arduino::Can("")
        , txPin_(tx_pin)
        , rxPin_(rx_pin)
        , txQueueLen_(0)
    {
    }

    /// Initializes the CAN-bus driver.
    /// @return true if successful, false on error.
    bool begin()
    {
        twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, TWAI_MODE_NORMAL);
        txQueueLen_ = g_config.tx_queue_len;
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
        if (result != ESP_OK)
        {
            return false;
        }

        result = twai_start();
        if (result != ESP_OK)
        {
            return false;
        }

        return true;
    }

    int available() override
    {
        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK)
        {
            return 0;
        }
        if (status.state == TWAI_STATE_BUS_OFF)
        {
            twai_initiate_recovery();
        }
        return status.msgs_to_rx;
    }

    int availableForWrite() override
    {
        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK)
        {
            return 0;
        }
        return txQueueLen_ - status.msgs_to_tx;
    }

    int read(struct can_frame *frame) override
    {
        twai_message_t message;
        esp_err_t result = twai_receive(&message, 0);
        if (result != ESP_OK)
        {
            return 0;
        }

        // Copy from twai_message_t to can_frame
        if (message.extd)
        {
            SET_CAN_FRAME_ID_EFF(*frame, message.identifier);
            SET_CAN_FRAME_EFF(*frame);
        }
        else
        {
            SET_CAN_FRAME_ID(*frame, message.identifier);
            CLR_CAN_FRAME_EFF(*frame);
        }

        if (message.rtr)
        {
            SET_CAN_FRAME_RTR(*frame);
        }
        else
        {
            CLR_CAN_FRAME_RTR(*frame);
        }

        CLR_CAN_FRAME_ERR(*frame);

        frame->can_dlc = message.data_length_code;
        memcpy(frame->data, message.data, message.data_length_code);

        return 1;
    }

    int write(const struct can_frame *frame) override
    {
        twai_message_t message;
        memset(&message, 0, sizeof(message));
        message.extd = IS_CAN_FRAME_EFF(*frame) ? 1 : 0;
        message.rtr = IS_CAN_FRAME_RTR(*frame) ? 1 : 0;

        if (IS_CAN_FRAME_ERR(*frame)) {
            // error frames are not transmitted.
            return 1;
        }

        message.identifier = IS_CAN_FRAME_EFF(*frame) ? GET_CAN_FRAME_ID_EFF(*frame) : GET_CAN_FRAME_ID(*frame);
        message.data_length_code = frame->can_dlc;
        memcpy(message.data, frame->data, frame->can_dlc);

        esp_err_t result = twai_transmit(&message, 0);
        if (result == ESP_OK)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

private:
    void enable() override {}
    void disable() override {}
    void tx_msg() override {}

    gpio_num_t txPin_;
    gpio_num_t rxPin_;
    size_t txQueueLen_;
};

#endif // _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_
