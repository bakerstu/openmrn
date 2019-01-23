/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *               2017, Jaime Breva, jbreva@nayarsystems.com
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_intr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <math.h>
#include <soc/dport_reg.h>

// from arduino-esp32
#include <esp32-hal-cpu.h>
#include <esp32-hal-log.h>

#include "utils/logging.h"

#include "freertos_drivers/esp32/Esp32CanDriver.hxx"
#include "freertos_drivers/esp32/Esp32CanRegisters.hxx"

static void CAN_read_frame_phy(BaseType_t *higherPriorityTaskWoken);
static void CAN_isr(void *arg_p);
static int CAN_write_frame_phy(const CAN_frame_t *p_frame);
static SemaphoreHandle_t sem_tx_complete;

QueueHandle_t rx_queue = nullptr; /**< \brief Handler to FreeRTOS RX queue. */

static void CAN_isr(void *arg_p)
{
    // read interrupt register (this also resets it)
    uint32_t interrupt = MODULE_CAN->IR.U;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    // Handle RX frame available interrupt
    if (interrupt & __CAN_IRQ_RX)
    {
        isr_log_v("frame received");
        CAN_read_frame_phy(&higherPriorityTaskWoken);
    }

    // Handle TX complete interrupt
    // Handle error interrupts.
    if ((interrupt &
            (__CAN_IRQ_TX                // 0x02
                | __CAN_IRQ_ERR          // 0x04
                | __CAN_IRQ_DATA_OVERRUN // 0x08
                | __CAN_IRQ_WAKEUP       // 0x10
                | __CAN_IRQ_ERR_PASSIVE  // 0x20
                | __CAN_IRQ_ARB_LOST     // 0x40
                | __CAN_IRQ_BUS_ERR      // 0x80
                )) != 0)
    {
        MODULE_CAN->CMR.B.TR = 0;
        isr_log_v("TX done, %04x", interrupt);
        if (interrupt & __CAN_IRQ_BUS_ERR)
        {
            isr_log_v("BUS-ERR: %02x", MODULE_CAN->ECC.B.ECC & 0x1F);
        }
        if (interrupt & __CAN_IRQ_ERR_PASSIVE)
        {
            isr_log_v("ERR-PASV: %02x", MODULE_CAN->ECC.B.ECC & 0x1F);
        }
        xSemaphoreGive(sem_tx_complete);
    }

    // check if any higher priority task has been woken by any handler
    if (higherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void CAN_read_frame_phy(BaseType_t *higherPriorityTaskWoken)
{
    // byte iterator
    uint8_t __byte_i;

    // frame read buffer
    CAN_frame_t __frame;

    // check if we have a queue. If not, operation is aborted.
    if (rx_queue == nullptr)
    {
        // Let the hardware know the frame has been read.
        MODULE_CAN->CMR.B.RRB = 1;
        return;
    }

    // get FIR
    __frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;

    // check if this is a standard or extended CAN frame
    // standard frame
    if (__frame.FIR.B.FF == CAN_frame_std)
    {
        // Get Message ID
        __frame.MsgID = _CAN_GET_STD_ID;

        // deep copy data bytes
        for (__byte_i = 0; __byte_i < __frame.FIR.B.DLC; __byte_i++)
        {
            __frame.data.u8[__byte_i] =
                MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[__byte_i];
        }
    }
    // extended frame
    else
    {
        // Get Message ID
        __frame.MsgID = _CAN_GET_EXT_ID;

        // deep copy data bytes
        for (__byte_i = 0; __byte_i < __frame.FIR.B.DLC; __byte_i++)
        {
            __frame.data.u8[__byte_i] =
                MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[__byte_i];
        }
    }

    // send frame to input queue
    isr_log_v("Passing frame to RX queue");
    xQueueSendToBackFromISR(rx_queue, &__frame, higherPriorityTaskWoken);

    // Let the hardware know the frame has been read.
    MODULE_CAN->CMR.B.RRB = 1;
}

static int CAN_write_frame_phy(const CAN_frame_t *p_frame)
{
    // byte iterator
    uint8_t __byte_i;
    log_v("pushing frame to CAN");

    // copy frame information record
    MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = p_frame->FIR.U;

    // standard frame
    if (p_frame->FIR.B.FF == CAN_frame_std)
    {
        log_v("standard frame");
        // Write message ID
        _CAN_SET_STD_ID(p_frame->MsgID);

        // Copy the frame data to the hardware
        for (__byte_i = 0; __byte_i < p_frame->FIR.B.DLC; __byte_i++)
        {
            MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[__byte_i] =
                p_frame->data.u8[__byte_i];
        }
    }
    // extended frame
    else
    {
        log_v("extended frame");
        // Write message ID
        _CAN_SET_EXT_ID(p_frame->MsgID);

        // Copy the frame data to the hardware
        for (__byte_i = 0; __byte_i < p_frame->FIR.B.DLC; __byte_i++)
        {
            MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[__byte_i] =
                p_frame->data.u8[__byte_i];
        }
    }

    log_v("requesting xmit");
    // Transmit frame
    MODULE_CAN->CMR.B.TR = 1;

    return 0;
}

QueueHandle_t CAN_init(gpio_num_t tx_pin_id, gpio_num_t rx_pin_id,
    CAN_speed_t speed, uint8_t rx_buffer_size)
{
    LOG(INFO,
        "Configuring CAN driver using RX-PIN: %d, TX-PIN: %d, RX-QUEUE-SIZE: "
        "%d",
        rx_pin_id, tx_pin_id, rx_buffer_size);
    rx_queue = xQueueCreate(rx_buffer_size, sizeof(CAN_frame_t));

    // Stop the controller for configuration
    CAN_stop();

    MODULE_CAN->MOD.B.LOM = 0;
    MODULE_CAN->MOD.B.STM = 0;
    MODULE_CAN->MOD.B.SM = 0;

    // Time quantum
    double __tq;

    // enable module
    periph_module_enable(PERIPH_CAN_MODULE);

    // configure TX pin
    gpio_set_level(tx_pin_id, 1);
    gpio_set_pull_mode(tx_pin_id, GPIO_FLOATING);
    gpio_set_direction(tx_pin_id, GPIO_MODE_OUTPUT);
    gpio_matrix_out(tx_pin_id, CAN_TX_IDX, false, false);
    gpio_pad_select_gpio(tx_pin_id);

    // configure RX pin
    gpio_set_pull_mode(rx_pin_id, GPIO_FLOATING);
    gpio_matrix_in(rx_pin_id, CAN_RX_IDX, false);
    gpio_pad_select_gpio(rx_pin_id);
    gpio_set_direction(rx_pin_id, GPIO_MODE_INPUT);

    // set to PeliCAN mode
    MODULE_CAN->CDR.B.CAN_M = 0x1;

    // turn off clock out
    MODULE_CAN->CDR.B.COD = 0;
    MODULE_CAN->CDR.B.COFF = 1;

    // synchronization jump width is the same for all baud rates
    MODULE_CAN->BTR0.B.SJW = 0x2;

    // TSEG2 is the same for all baud rates
    MODULE_CAN->BTR1.B.TSEG2 = 0x1;

    // select time quantum and set TSEG1
    switch (speed)
    {
        case CAN_SPEED_1000KBPS:
            MODULE_CAN->BTR1.B.TSEG1 = 0x4;
            __tq = 0.125;
            break;

        case CAN_SPEED_800KBPS:
            MODULE_CAN->BTR1.B.TSEG1 = 0x6;
            __tq = 0.125;
            break;

        case CAN_SPEED_200KBPS:
            MODULE_CAN->BTR1.B.TSEG1 = 0xc;
            MODULE_CAN->BTR1.B.TSEG2 = 0x5;
            __tq = 0.25;
            break;

        default:
            MODULE_CAN->BTR1.B.TSEG1 = 0xc;
            __tq = ((float)1000 / speed) / 16;
    }

    // TODO -- add frequency update after wrapping this as a class instance
    // addApbChangeCallback(nullptr, [](void * arg, apb_change_ev_t ev_type,
    // uint32_t old_apb, uint32_t new_apb)
    //{
    //	MODULE_CAN->BTR0.B.BRP = (uint8_t) round((((new_apb * __tq) / 2) - 1) /
    //1000000) - 1;
    //});

    // set baud rate prescaler
    MODULE_CAN->BTR0.B.BRP =
        (uint8_t)round((((getApbFrequency() * __tq) / 2) - 1) / 1000000) - 1;

    /* Set sampling
     * 1 -> triple; the bus is sampled three times; recommended for low/medium
     * speed buses (class A and B) where filtering spikes on the bus line is
     * beneficial 0 -> single; the bus is sampled once; recommended for high
     * speed buses (SAE class C)
     */
    switch (speed)
    {
        case CAN_SPEED_1000KBPS:
        case CAN_SPEED_800KBPS:
        case CAN_SPEED_500KBPS:
            MODULE_CAN->BTR1.B.SAM = 0x0;
            break;
        default:
            MODULE_CAN->BTR1.B.SAM = 0x1;
            break;
    }

    // enable all interrupts
    MODULE_CAN->IER.U = 0xff;

    // no acceptance filtering, as we want to fetch all messages
    MODULE_CAN->MOD.B.AFM = 1;
    MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
    MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
    MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
    MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;

    // set to normal mode
    MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM;

    // clear error counters
    MODULE_CAN->TXERR.U = 0;
    MODULE_CAN->RXERR.U = 0;
    (void)MODULE_CAN->ECC;

    // clear interrupt flags
    (void)MODULE_CAN->IR.U;

    // install CAN ISR
    esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, CAN_isr, NULL, NULL);

    // allocate the tx complete semaphore
    sem_tx_complete = xSemaphoreCreateBinary();

    // Showtime. Release Reset Mode.
    CAN_start();

    return rx_queue;
}

int CAN_write_frame(const CAN_frame_t *p_frame)
{
    if (sem_tx_complete == NULL)
    {
        log_v("No TX semaphore");
        return -1;
    }

    if (MODULE_CAN->SR.B.TBS)
    {
        // Write the frame to the controller
        CAN_write_frame_phy(p_frame);

        // wait for the frame tx to complete
        xSemaphoreTake(sem_tx_complete, portMAX_DELAY);
    }
    else
    {
        return 1;
    }
    return 0;
}

int CAN_start()
{
    // exit reset mode
    MODULE_CAN->MOD.B.RM = 0;

    return 0;
}

int CAN_stop()
{
    // enter reset mode
    MODULE_CAN->MOD.B.RM = 1;

    return 0;
}

int CAN_reset()
{
    // disable all interrupts
    MODULE_CAN->IER.U = 0x00;

    CAN_stop();

    // enable all interrupts
    MODULE_CAN->IER.U = 0xff;

    // set to normal mode
    MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM;

    // clear error counters
    MODULE_CAN->TXERR.U = 0;
    MODULE_CAN->RXERR.U = 0;
    (void)MODULE_CAN->ECC;

    // clear interrupt flags
    (void)MODULE_CAN->IR.U;

    // Showtime. Release Reset Mode.
    CAN_start();

    return 0;
}