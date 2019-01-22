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
            .tx_queue_len = (uint32_t)config_can_tx_buffer_size() / 2,
            .rx_queue_len = (uint32_t)config_can_rx_buffer_size() / 2,
            .alerts_enabled = CAN_ALERT_NONE,
            .clkout_divider = 0
        };
        LOG(INFO, "Configuring CAN driver using RX: %d, TX: %d, TX-QUEUE: %d, RX-QUEUE: %d",
            can_general_config.rx_io, can_general_config.tx_io,
            can_general_config.rx_queue_len, can_general_config.tx_queue_len);
        ESP_ERROR_CHECK(can_driver_install(&can_general_config, &can_timing_config, &can_filter_config));

        tx_mutex_ = xSemaphoreCreateMutex();
        if(tx_mutex_ == NULL)
        {
            LOG(FATAL, "Unable to create CAN TX mutex");
        }
        xTaskCreatePinnedToCore(rx_task, "CAN RX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 1, nullptr, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(tx_task, "CAN TX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 1, nullptr, tskNO_AFFINITY);
        // create a low priority task to monitor/report the can status
        xTaskCreatePinnedToCore(can_monitor, "CAN MONITOR", OPENMRN_STACK_SIZE, this,
            tskIDLE_PRIORITY, nullptr, tskNO_AFFINITY);
    }

    ~Esp32HardwareCan()
    {
        vSemaphoreDelete(tx_mutex_);
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
    virtual void tx_msg()
    {
        can_status_info_t status;
        ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
        if(status.state != CAN_STATE_RUNNING)
        {
            LOG(WARNING, "CAN BUS is not RUNNING: %d", status.state);
        }
        else
        {
            xSemaphoreGive(tx_mutex_);
            while(xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(5)) != pdPASS) {
                LOG(WARNING, "tx_task didn't return mutex");
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
private:
    /** Default constructor.
     */
    Esp32HardwareCan();

    SemaphoreHandle_t tx_mutex_;

    static void can_monitor(void *unused)
    {
        TickType_t xLastWakeTime;
        while(true)
        {
            xLastWakeTime = xTaskGetTickCount();
            can_status_info_t status;
            can_get_status_info(&status);
            LOG(INFO, "CAN-STATUS: rx:%d, tx:%d, rx-err:%d, tx-err:%d, arb-lost:%d, bus-err:%d, state: %d",
                status.msgs_to_rx, status.msgs_to_tx,
                status.rx_error_counter, status.tx_error_counter,
                status.arb_lost_count, status.bus_error_count,
                status.state);
            if(status.state == CAN_STATE_BUS_OFF)
            {
                LOG(INFO, "CAN BUS is OFF, initiating recovery");
                can_initiate_recovery();
            }
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
        }
    }

    static void tx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            can_message_t msg = {0};
            if(xSemaphoreTake(parent->tx_mutex_, pdMS_TO_TICKS(50)) == pdPASS)
            {
                bool tx_needed = false;
                struct can_frame *can_frame = nullptr;
                if(parent->txBuf->data_read_pointer(&can_frame) && can_frame != nullptr)
                {
                    LOG(INFO, "can_frame: id:%08x, eff:%d, rtr:%d, dlc:%d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        can_frame->can_id, can_frame->can_eff, can_frame->can_rtr, can_frame->can_dlc,
                        can_frame->data[0], can_frame->data[1], can_frame->data[2], can_frame->data[3],
                        can_frame->data[4], can_frame->data[5], can_frame->data[6], can_frame->data[7]);
                    msg.identifier = can_frame->can_id;
                    msg.data_length_code = can_frame->can_dlc;
                    for(int i = 0; i < can_frame->can_dlc; i++)
                    {
                        msg.data[i] = can_frame->data[i];
                    }
                    if(can_frame->can_eff)
                    {
                        msg.flags |= CAN_MSG_FLAG_EXTD;
                    }
                    if(can_frame->can_rtr)
                    {
                        msg.flags |= CAN_MSG_FLAG_RTR;
                    }
                    parent->txBuf->consume(1);
                    parent->txBuf->signal_condition();
                    tx_needed = true;
                }
                xSemaphoreGive(parent->tx_mutex_);
                if(tx_needed)
                {
                    LOG(INFO, "CAN-TX id:%08x, flags:%04x, dlc:%02d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        msg.identifier, msg.flags, msg.data_length_code,
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    // block here until the CAN driver accepts the message for transmit
                    bool tx_done = false;
                    while(!tx_done)
                    {
                        esp_task_wdt_reset();
                        esp_err_t tx_res = can_transmit(&msg, pdMS_TO_TICKS(250));
                        if(tx_res != ESP_ERR_TIMEOUT && tx_res != ESP_ERR_INVALID_STATE)
                        {
                            LOG(WARNING, "CAN-TX: %s", esp_err_to_name(tx_res));
                            tx_done = true;
                        }
                        // sleep for 1ms to allow driver to catch up
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
            }
        }
    }

    static void rx_task(void *can)
    {
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            can_message_t msg = {0};
            if(can_receive(&msg, pdMS_TO_TICKS(250)) == ESP_OK)
            {
                if(msg.flags & CAN_MSG_FLAG_DLC_NON_COMP)
                {
                    LOG(WARNING, "Received non-spec CAN frame!");
                }
                else
                {
                    LOG(INFO, "CAN-RX id:%08x, flags:%04x, dlc:%02d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        msg.identifier, msg.flags, msg.data_length_code,
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    AtomicHolder h(parent);
                    struct can_frame *can_frame = nullptr;
                    if(parent->rxBuf->data_write_pointer(&can_frame) && can_frame != nullptr)
                    {
                        LOG(INFO, "CAN-RX: building can_frame");
                        memset(can_frame, 0, sizeof(struct can_frame));
                        can_frame->can_id = msg.identifier;
                        can_frame->can_dlc = msg.data_length_code;
                        for(int i = 0; i < msg.data_length_code; i++)
                        {
                            can_frame->data[i] = msg.data[i];
                        }
                        if(msg.flags & CAN_MSG_FLAG_EXTD)
                        {
                            can_frame->can_eff = 1;
                        }
                        if(msg.flags & CAN_MSG_FLAG_RTR)
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