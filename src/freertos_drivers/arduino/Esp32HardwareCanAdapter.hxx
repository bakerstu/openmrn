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
#include <esp_task_wdt.h>

class Esp32HardwareCan : public Can
{
public:
    Esp32HardwareCan(const char *name, gpio_num_t rxPin, gpio_num_t txPin) : Can(name)
    {
        can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_250KBITS();
        can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
        // Note: not using the CAN_GENERAL_CONFIG_DEFAULT macro due to a missing
        // cast for CAN_IO_UNUSED.
        can_general_config_t can_general_config =
        {
            .mode = CAN_MODE_NORMAL,
            .tx_io = txPin,
            .rx_io = rxPin,
            .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
            .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
            .tx_queue_len = config_can_tx_buffer_size(),
            .rx_queue_len = config_can_rx_buffer_size(),
            .alerts_enabled = CAN_ALERT_AND_LOG | CAN_ALERT_TX_FAILED | CAN_ALERT_TX_SUCCESS,
            .clkout_divider = 0
        };
        ESP_ERROR_CHECK(can_driver_install(&can_general_config,
                &can_timing_config, &can_filter_config));

        tx_task_queue_ = xQueueCreate(config_can_tx_buffer_size(), sizeof(can_message_t));
        xTaskCreatePinnedToCore(rx_task, "CAN RX", OPENMRN_STACK_SIZE, this,
            OPENMRN_TASK_PRIORITY, nullptr, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(tx_task, "CAN TX", OPENMRN_STACK_SIZE, this,
            OPENMRN_TASK_PRIORITY, nullptr, tskNO_AFFINITY);
    }

    ~Esp32HardwareCan()
    {
        vQueueDelete(tx_task_queue_);
    }

    /**< function to enable device */
    virtual void enable()
    {
        LOG(INFO, "Enabling CAN driver");
        ESP_ERROR_CHECK(can_start());
    }

    /**< function to disable device */
    virtual void disable()
    {
        LOG(INFO, "Disabling CAN driver");
        ESP_ERROR_CHECK(can_stop());
    }
protected:
    /**< function to try and transmit a message */
    virtual void tx_msg()
    {
        can_status_info_t status;
        ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
        /* see if we can send anything out */
        struct can_frame *can_frame;
        while(status.state == CAN_STATE_RUNNING && txBuf->data_read_pointer(&can_frame))
        {
            can_message_t msg =
            {
                .flags = CAN_MSG_FLAG_NONE,
                .identifier = can_frame->can_id,
                .data_length_code = can_frame->can_dlc,
                .data = {0}
            };
            if(can_frame->can_dlc)
            {
                memcpy(&msg.data, &can_frame->data, can_frame->can_dlc);
            }
            if(can_frame->can_eff)
            {
                msg.flags |= CAN_MSG_FLAG_EXTD;
            }
            if(can_frame->can_rtr)
            {
                msg.flags |= CAN_MSG_FLAG_RTR;
            }
            if(xQueueSend(tx_task_queue_, &msg, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                // successfully queued for TX, remove it from txBuf
                txBuf->consume(1);
                txBuf->signal_condition();
            }
            else
            {
                // something went wrong with transmit, give up
                return;
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
        }
    }
private:
    /** Default constructor.
     */
    Esp32HardwareCan();

    QueueHandle_t tx_task_queue_;

    static void tx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            can_message_t msg;
            if(xQueueReceive(parent->tx_task_queue_, &msg, portMAX_DELAY) == pdTRUE)
            {
                LOG(INFO, "TX to %08x: %02x %02x %02x %02x %02x %02x %02x %02x", msg.identifier,
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                    msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                ESP_ERROR_CHECK_WITHOUT_ABORT(can_transmit(&msg, portMAX_DELAY));
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }

    static void rx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            can_status_info_t status;
            ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
            if(status.state == CAN_STATE_BUS_OFF)
            {
                LOG(INFO, "Bus is OFF, initiating recovery");
                can_initiate_recovery();
            }
            else if(status.state == CAN_STATE_RUNNING)
            {
                if(status.msgs_to_rx > 0 && parent->rxBuf->space() > 0)
                {
                    LOG(INFO, "RX: state: %d, msgs:%d", status.state, status.msgs_to_rx);
                    // while there is room and we have frames to receive
                    struct can_frame *can_frame;
                    while(status.msgs_to_rx > 0 && parent->rxBuf->data_write_pointer(&can_frame))
                    {
                        esp_task_wdt_reset();
                        can_message_t msg = {0};
                        ESP_ERROR_CHECK_WITHOUT_ABORT(can_receive(&msg, pdMS_TO_TICKS(5)));
                        if(msg.flags & CAN_MSG_FLAG_DLC_NON_COMP)
                        {
                            LOG(INFO, "Received non-spec CAN frame!");
                        }
                        else
                        {
                            LOG(INFO, "RX from %08x: %02x %02x %02x %02x %02x %02x %02x %02x",
                                msg.identifier,
                                msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
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
                            if(msg.data_length_code)
                            {
                                memcpy(&can_frame->data, &msg.data, msg.data_length_code);
                            }
                            parent->rxBuf->advance(1);
                            parent->rxBuf->signal_condition();
                        }
                        ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
                    }
                }
                else
                {
                    parent->overrunCount++;
                }
            }
            // yield to other tasks that are running on the ESP32
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    DISALLOW_COPY_AND_ASSIGN(Esp32HardwareCan);
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_ */