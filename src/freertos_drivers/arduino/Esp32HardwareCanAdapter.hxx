/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file Esp32HardwareCanAdapter.hxx
 *
 * ESP32 Hardware CAN adapter code. This utilizes the built in CAN controller
 * to translate the can_frame in OpenMRN to the ESP32 can_message_t used by the
 * ESP-IDF CAN controller code. The ESP32 will still require an external CAN
 * transceiver (MCP2551 or SN65HVD230 as example).
 *
 * @author Mike Dunston
 * @date 19 January 2019
 */

// This include is exclusive against freertos_drivers/arduino/Esp32HardwareCan.hxx
#ifndef _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_
#define _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_

#include "freertos_drivers/arduino/Can.hxx"
#include <driver/can.h>
#include <driver/gpio.h>

class Esp32Can : public Can
{
public:
    Esp32Can(const char *name, gpio_num_t rxPin, gpio_num_t txPin) : Can(name)
    {
        can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_250KBITS();
        can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
        // Note: not using the CAN_GENERAL_CONFIG_DEFAULT macro due to a missing
        // cast for CAN_IO_UNUSED.
        can_general_config_t can_general_config = {
            .mode = CAN_MODE_NORMAL,
            .tx_io = txPin,
            .rx_io = rxPin,
            .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
            .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
            .tx_queue_len = 5,
            .rx_queue_len = 5,
            .alerts_enabled = CAN_ALERT_NONE,
            .clkout_divider = 0
        };
        ESP_ERROR_CHECK(can_driver_install(&can_general_config,
                &can_timing_config, &can_filter_config));
        xTaskCreatePinnedToCore(rx_task, name, OPENMRN_STACK_SIZE, this,
            OPENMRN_TASK_PRIORITY, nullptr, tskNO_AFFINITY);
    }

    ~Esp32Can()
    {
    }

    /**< function to enable device */
    virtual void enable()
    {
        esp_err_t start_result = can_start();
        ESP_ERROR_CHECK(start_result);
    }

    /**< function to disable device */
    virtual void disable()
    {
        esp_err_t stop_result = can_stop();
        ESP_ERROR_CHECK(stop_result);
    }
protected:
    /**< function to try and transmit a message */
    virtual void tx_msg()
    {
        /* see if we can send anything out */
        struct can_frame *can_frame;

        size_t msg_count = txBuf->data_read_pointer(&can_frame);
        unsigned txCount;
        for (txCount = 0; txCount < msg_count; ++txCount, ++can_frame)
        {
            can_message_t msg;
            msg.identifier = can_frame->can_id;
            msg.data_length_code = can_frame->can_dlc;
            msg.data[0] = can_frame->data[0];
            msg.data[1] = can_frame->data[1];
            msg.data[2] = can_frame->data[2];
            msg.data[3] = can_frame->data[3];
            msg.data[4] = can_frame->data[4];
            msg.data[5] = can_frame->data[5];
            msg.data[6] = can_frame->data[6];
            msg.data[7] = can_frame->data[7];
            if(can_frame->can_eff)
            {
                msg.flags |= CAN_MSG_FLAG_EXTD;
            }
            if(can_frame->can_rtr)
            {
                msg.flags |= CAN_MSG_FLAG_RTR;
            }
            esp_err_t tx_result = can_transmit(&msg, pdMS_TO_TICKS(10));
            if(tx_result != ESP_OK)
            {
                // something went wrong with transmit, give up
                break;
            }
        }
        if(txCount)
        {
            txBuf->consume(txCount);
            txBuf->signal_condition();
        }
    }
private:
    /** Default constructor.
     */
    Esp32Can();

    static void rx_task(void *can)
    {
        Esp32Can *parent = reinterpret_cast<Esp32Can *>(can);
        while(true)
        {
            unsigned msg_receive_count = 0;
            can_status_info_t status;
            can_get_status_info(&status);
            if(status.state == CAN_STATE_BUS_OFF)
            {
                // initiate CAN bus recovery
                ESP_ERROR_CHECK(can_initiate_recovery());
                parent->txBuf->flush();
                parent->txBuf->signal_condition();
            }
            else if(status.state != CAN_STATE_RECOVERING)
            {
                if(status.msgs_to_rx)
                {
                    struct can_frame *can_frame;
                    size_t msg_count = parent->rxBuf->data_write_pointer(&can_frame);
                    if(msg_count)
                    {
                        // while there is room and we have frames to receive
                        while(msg_count && status.msgs_to_rx)
                        {
                            can_message_t msg;
                            esp_err_t result = can_receive(&msg, pdMS_TO_TICKS(5));
                            if(result == ESP_OK)
                            {
                                can_frame->can_id = msg.identifier;
                                if(msg.flags & CAN_MSG_FLAG_EXTD)
                                {
                                    can_frame->can_eff = 1;
                                }
                                if(msg.flags & CAN_MSG_FLAG_RTR)
                                {
                                    can_frame->can_rtr = 1;
                                }
                                can_frame->can_dlc = msg.data_length_code;
                                if(msg.flags & CAN_MSG_FLAG_DLC_NON_COMP)
                                {
                                    printf("\nReceived non-spec CAN frame!\n");
                                }
                                for(uint8_t index = 0; index < CAN_MAX_DATA_LEN; index++) {
                                    can_frame->data[index] = msg.data[index];
                                }
                                msg_count--;
                                can_frame++;
                                msg_receive_count++;
                            }
                            can_get_status_info(&status);
                        }
                    }
                    else
                    {
                        parent->overrunCount++;
                    }
                    if(msg_receive_count)
                    {
                        parent->rxBuf->advance(msg_receive_count);
                        parent->rxBuf->signal_condition();
                    }
                }
            }
            // yield to other tasks that are running on the ESP32
            vPortYield();
        }
    }
    DISALLOW_COPY_AND_ASSIGN(Esp32Can);
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_ */