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
 * \file Stm32RailcomSender.hxx
 *
 * Implements a RailcomDriver that sends outgoing railcom data through a UART
 * TX on an STM32 target. Designed to be invoked from the priority zero
 * interrupt of the DCC decoder.
 *
 * @author Balazs Racz
 * @date July 10, 2021
 */

#include "freertos_drivers/st/Stm32RailcomSender.hxx"

#include "stm32f_hal_conf.hxx"

/// Called at the beginning of the first window.
void Stm32RailcomSender::start_cutout()
{
    const auto* pkt = ch1Pkt_;
    if (!pkt || pkt->feedbackKey != expectedFeedbackKey_)
    {
        // Nothing to send or came too late and should not be sent.
        return;
    }
    if (!__HAL_UART_GET_IT(&uartHandle, UART_IT_TXE)) {
        // Transmission is not complete yet. That's weird.
        return;
    }
    if (pkt->ch1Size == 0) {
        // Nothing to send.
        return;
    }
    uartHandle.Instance->TDR = pkt->ch1Data[0];
    if (pkt->ch1Size > 1)
    {
        txBuf->put(pkt->ch1Data + 1, 1);
        __HAL_UART_ENABLE_IT(&uartHandle, UART_IT_TXE);
    }
}

void Stm32RailcomSender::middle_cutout()
{
    const auto* pkt = ch2Pkt_;
    if (!pkt || pkt->feedbackKey != expectedFeedbackKey_)
    {
        // Nothing to send or came too late and should not be sent.
        return;
    }
    if (!__HAL_UART_GET_IT(&uartHandle, UART_IT_TXE)) {
        // Transmission is not complete yet. That's weird.
        return;
    }
    if (pkt->ch2Size == 0) {
        // Nothing to send.
        return;
    }
    uartHandle.Instance->TDR = pkt->ch2Data[0];
    if (pkt->ch2Size > 1)
    {
        txBuf->put(pkt->ch2Data + 1, pkt->ch2Size - 1);
        __HAL_UART_ENABLE_IT(&uartHandle, UART_IT_TXE);
    }
}
