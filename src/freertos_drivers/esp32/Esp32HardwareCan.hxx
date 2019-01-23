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

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_

#include "freertos_drivers/arduino/Can.hxx"
#include "freertos_drivers/esp32/Esp32CanDriver.hxx"
#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

class Esp32Can : public Can
{
public:
    Esp32Can(const char *name, gpio_num_t rxPin, gpio_num_t txPin) : Can(name)
    {
        rx_queue_ = CAN_init(txPin, rxPin,
            CAN_speed_t::CAN_SPEED_250KBPS,
            config_can_rx_buffer_size() / 2);
        tx_mutex_ = xSemaphoreCreateMutex();
        if(tx_mutex_ == NULL)
        {
            LOG(FATAL, "Unable to create CAN TX mutex");
        }
        xTaskCreatePinnedToCore(rx_task, "CAN RX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 1, nullptr, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(tx_task, "CAN TX", OPENMRN_STACK_SIZE, this,
            ESP_TASK_TCPIP_PRIO - 2, nullptr, tskNO_AFFINITY);
        // create a low priority task to monitor/report the can status
        //xTaskCreatePinnedToCore(can_monitor, "CAN MONITOR", OPENMRN_STACK_SIZE, this,
        //    tskIDLE_PRIORITY, nullptr, tskNO_AFFINITY);
    }

    ~Esp32Can()
    {
        vSemaphoreDelete(tx_mutex_);
    }

    /**< function to enable device */
    virtual void enable()
    {
        CAN_start();
        LOG(INFO, "CAN driver enabled");
    }

    /**< function to disable device */
    virtual void disable()
    {
        CAN_stop();
        LOG(INFO, "CAN driver disabled");
    }
protected:
    /**< function to try and transmit a message */
    virtual void tx_msg()
    {
        //can_status_info_t status;
        //ESP_ERROR_CHECK_WITHOUT_ABORT(can_get_status_info(&status));
        //if(status.state != CAN_STATE_RUNNING)
        //{
        //    LOG(WARNING, "CAN BUS is not RUNNING: %d", status.state);
        //}
        //else
        {
            LOG(INFO, "Sending frame for TX");
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
    Esp32Can();

    SemaphoreHandle_t tx_mutex_;
    QueueHandle_t rx_queue_;
/*
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
*/
    static void tx_task(void *can)
    {
        Esp32Can *parent = reinterpret_cast<Esp32Can *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            CAN_frame_t frame = {0};
            if(xSemaphoreTake(parent->tx_mutex_, pdMS_TO_TICKS(50)) == pdPASS)
            {
                LOG(INFO, "txBuf: space: %d, pending: %d", parent->txBuf->space(), parent->txBuf->pending());
                bool tx_needed = false;
                struct can_frame *can_frame = nullptr;
                if(parent->txBuf->data_read_pointer(&can_frame) && can_frame != nullptr)
                {
                    LOG(INFO, "can_frame: id:%08x, eff:%d, rtr:%d, dlc:%d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        can_frame->can_id, can_frame->can_eff, can_frame->can_rtr, can_frame->can_dlc,
                        can_frame->data[0], can_frame->data[1], can_frame->data[2], can_frame->data[3],
                        can_frame->data[4], can_frame->data[5], can_frame->data[6], can_frame->data[7]);
                    frame.MsgID = can_frame->can_id;
                    frame.FIR.B.DLC = can_frame->can_dlc;
                    if(can_frame->can_eff)
                    {
                        frame.FIR.B.FF = CAN_frame_format_t::CAN_frame_ext;
                    }
                    for(int i = 0; i < can_frame->can_dlc; i++)
                    {
                        frame.data.u8[i] = can_frame->data[i];
                    }
                    if(can_frame->can_rtr)
                    {
                        frame.FIR.B.RTR = CAN_RTR_t::CAN_RTR;
                    }
                    parent->txBuf->consume(1);
                    parent->txBuf->signal_condition();
                    tx_needed = true;
                }
                xSemaphoreGive(parent->tx_mutex_);
                if(tx_needed)
                {
                    LOG(INFO, "CAN-TX id:%08x, eff:%02d, rtr:%02d, dlc:%02d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        frame.MsgID, frame.FIR.B.FF, frame.FIR.B.RTR, frame.FIR.B.DLC,
                        frame.data.u8[0], frame.data.u8[1], frame.data.u8[2], frame.data.u8[3],
                        frame.data.u8[4], frame.data.u8[5], frame.data.u8[6], frame.data.u8[7]);
                    // block here until the CAN driver accepts the message for transmit
                    while(CAN_write_frame(&frame))
                    {
                        esp_task_wdt_reset();
                        // sleep for 1ms to allow driver to catch up
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
            }
        }
    }

    static void rx_task(void *can)
    {
        Esp32Can *parent = reinterpret_cast<Esp32Can *>(can);
        while(true)
        {
            esp_task_wdt_reset();
            CAN_frame_t frame = {0};
            if(xQueueReceive(parent->rx_queue_, &frame, pdMS_TO_TICKS(250)) == pdTRUE)
            {
                LOG(INFO, "CAN-RX id:%08x, eff:%02d, rtr:%02d, dlc:%02d, data:%02x%02x%02x%02x%02x%02x%02x%02x",
                    frame.MsgID, frame.FIR.B.FF, frame.FIR.B.RTR, frame.FIR.B.DLC,
                    frame.data.u8[0], frame.data.u8[1], frame.data.u8[2], frame.data.u8[3],
                    frame.data.u8[4], frame.data.u8[5], frame.data.u8[6], frame.data.u8[7]);
                AtomicHolder h(parent);
                struct can_frame *can_frame = nullptr;
                if(parent->rxBuf->data_write_pointer(&can_frame) && can_frame != nullptr)
                {
                    LOG(INFO, "CAN-RX: building can_frame");
                    memset(can_frame, 0, sizeof(struct can_frame));
                    can_frame->can_id = frame.MsgID;
                    can_frame->can_dlc = frame.FIR.B.DLC;
                    for(int i = 0; i < frame.FIR.B.DLC; i++)
                    {
                        can_frame->data[i] = frame.data.u8[i];
                    }
                    if(frame.FIR.B.FF)
                    {
                        can_frame->can_eff = 1;
                    }
                    if(frame.FIR.B.RTR)
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
    DISALLOW_COPY_AND_ASSIGN(Esp32Can);
};

#endif /* _FREERTOS_DRIVERS_ESP32_ESP32CAN_HXX_ */