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

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32HWCAN_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32HWCAN_HXX_

#include "freertos_drivers/arduino/Can.hxx"
#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

class Esp32HardwareCan : public Can
{
public:
    Esp32HardwareCan(const char *name, gpio_num_t rxPin, gpio_num_t txPin)
        : Can(name)
    {
        can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_125KBITS();
        can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
        // Note: not using the CAN_GENERAL_CONFIG_DEFAULT macro due to a missing
        // cast for CAN_IO_UNUSED.
        can_general_config_t can_general_config = {.mode = CAN_MODE_NORMAL,
            .tx_io = txPin,
            .rx_io = rxPin,
            .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
            .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
            .tx_queue_len = (uint32_t)config_can_tx_buffer_size() / 2,
            .rx_queue_len = (uint32_t)config_can_rx_buffer_size() / 2,
            .alerts_enabled = CAN_ALERT_NONE,
            .clkout_divider = 0};
        LOG(INFO,
            "Configuring CAN driver using RX: %d, TX: %d, TX-QUEUE: %d, "
            "RX-QUEUE: %d",
            can_general_config.rx_io, can_general_config.tx_io,
            can_general_config.rx_queue_len, can_general_config.tx_queue_len);
        ESP_ERROR_CHECK(can_driver_install(
            &can_general_config, &can_timing_config, &can_filter_config));

        xTaskCreatePinnedToCore(rx_task, "CAN RX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 1, nullptr, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(tx_task, "CAN TX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 2, &txTaskHandle_, tskNO_AFFINITY);
    }

    ~Esp32HardwareCan()
    {
    }

    /**< function to enable device */
    virtual void enable()
    {
        ESP_ERROR_CHECK(can_start());
        LOG(INFO, "CAN driver enabled");
    }

    /**< function to disable device */
    virtual void disable()
    {
        ESP_ERROR_CHECK(can_stop());
        LOG(INFO, "CAN driver disabled");
    }

protected:
    /**< function to try and transmit a message */
    void tx_msg() override
    {
        xTaskNotifyGive(txTaskHandle_);
    }

private:
    /** Default constructor.
     */
    Esp32HardwareCan();

    TaskHandle_t txTaskHandle_;

    static void tx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        TickType_t last_status_time = 0;
        while (true)
        {
            esp_task_wdt_reset();
            // periodic CAN driver monitoring task, also covers automatic bus
            // recovery when the CAN driver disables the bus due to error
            // conditions exceeding thresholds.
            auto current_time = xTaskGetTickCount();
            if (current_time > last_status_time + pdMS_TO_TICKS(5000))
            {
                last_status_time = current_time;
                can_status_info_t status;
                can_get_status_info(&status);
                LOG(INFO,
                    "CAN-STATUS: rx-q:%d, tx-q:%d, rx-err:%d, tx-err:%d, "
                    "arb-lost:%d, bus-err:%d, state: %d",
                    status.msgs_to_rx, status.msgs_to_tx,
                    status.rx_error_counter, status.tx_error_counter,
                    status.arb_lost_count, status.bus_error_count,
                    status.state);

                if (status.state == CAN_STATE_BUS_OFF)
                {
                    LOG(INFO, "CAN BUS is OFF, initiating recovery");
                    can_initiate_recovery();
                }
            }
            // check txBuf for any message to transmit
            unsigned count;
            struct can_frame *can_frame = nullptr;
            {
                AtomicHolder h(parent);
                count = parent->txBuf->data_read_pointer(&can_frame);
            }
            if (!count || !can_frame)
            {
                // tx Buf empty; wait for tx_msg to be called.
                ulTaskNotifyTake(pdTRUE, // clear on exit
                    pdMS_TO_TICKS(250));
                continue;
            }

            /// ESP32 native can driver frame
            can_message_t msg = {0};

            msg.flags = CAN_MSG_FLAG_NONE;
            msg.identifier = can_frame->can_id;
            msg.data_length_code = can_frame->can_dlc;
            for (int i = 0; i < can_frame->can_dlc; i++)
            {
                msg.data[i] = can_frame->data[i];
            }
            if (IS_CAN_FRAME_EFF(*can_frame))
            {
                msg.flags |= CAN_MSG_FLAG_EXTD;
            }
            if (IS_CAN_FRAME_RTR(*can_frame))
            {
                msg.flags |= CAN_MSG_FLAG_RTR;
            }
            // Pass the generated CAN frame to the native driver
            // for transmit, if the TX queue is full this will
            // return ESP_ERR_TIMEOUT which will result in the
            // the message being left in txBuf for the next iteration.
            // if this call returns ESP_OK we consider the frame as
            // transmitted by the driver and remove it from txBuf.
            esp_err_t tx_res = can_transmit(&msg, pdMS_TO_TICKS(100));
            if (tx_res == ESP_OK)
            {
                LOG(INFO,
                    "CAN-TX OK id:%08x, flags:%04x, dlc:%02d, "
                    "data:%02x%02x%02x%02x%02x%02x%02x%02x",
                    msg.identifier, msg.flags, msg.data_length_code,
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                    msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                AtomicHolder h(parent);
                parent->txBuf->consume(1);
                parent->txBuf->signal_condition();
            }
            else if (tx_res != ESP_ERR_TIMEOUT)
            {
                LOG(WARNING, "CAN-TX: %s", esp_err_to_name(tx_res));
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } // loop on task
    }

    static void rx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        while (true)
        {
            esp_task_wdt_reset();
            can_message_t msg = {0};
            if (can_receive(&msg, pdMS_TO_TICKS(250)) == ESP_OK)
            {
                if (msg.flags & CAN_MSG_FLAG_DLC_NON_COMP)
                {
                    LOG(WARNING, "Received non-spec CAN frame, dropping frame!");
                }
                else
                {
                    LOG(INFO,
                        "CAN-RX id:%08x, flags:%04x, dlc:%02d, "
                        "data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        msg.identifier, msg.flags, msg.data_length_code,
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    AtomicHolder h(parent);
                    struct can_frame *can_frame = nullptr;
                    if (parent->rxBuf->data_write_pointer(&can_frame) &&
                        can_frame != nullptr)
                    {
                        LOG(INFO, "CAN-RX: building can_frame");
                        memset(can_frame, 0, sizeof(struct can_frame));
                        can_frame->can_id = msg.identifier;
                        can_frame->can_dlc = msg.data_length_code;
                        for (int i = 0; i < msg.data_length_code; i++)
                        {
                            can_frame->data[i] = msg.data[i];
                        }
                        if (msg.flags & CAN_MSG_FLAG_EXTD)
                        {
                            can_frame->can_eff = 1;
                        }
                        if (msg.flags & CAN_MSG_FLAG_RTR)
                        {
                            can_frame->can_rtr = 1;
                        }
                        parent->rxBuf->advance(1);
                        parent->rxBuf->signal_condition();
                    }
                    else
                    {
                        LOG(WARNING, "CAN-RX: buffer overrun");
                        parent->overrunCount++;
                    }
                }
            }
        }
    }
    DISALLOW_COPY_AND_ASSIGN(Esp32HardwareCan);
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_ */
