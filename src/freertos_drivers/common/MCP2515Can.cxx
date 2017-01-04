/** @copyright
 * Copyright (c) 2017 Stuart W Baker
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
 * @file MCP2515Can.cxx
 * This file implements the CAN driver for the MCP2515 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 3 January 2017
 */

#include "MCP2515Can.hxx"

#include <fcntl.h>
#include <unistd.h>

/*
 * MCP2515Can()
 */
MCP2515Can::MCP2515Can(const char *name, const char *spi_name,
                       void (*interrupt_enable)(void),
                       void (*interrupt_disable)(void))
    : Can(name)
    , OSThread()
    , interrupt_enable(interrupt_enable)
    , interrupt_disable(interrupt_disable)
    , spi(::open(spi_name, O_RDWR))
    , sem()
{
    HASSERT(spi >= 0);
}

/*
 * enable()
 */
void MCP2515Can::enable()
{
    /* there is a mutex lock above us, so the following sequence is atomic */
    if (!is_created())
    {
        /* start the thread */
        /** @todo make this the highest possible thread priority */
        start(name, 0, 1024);
    }

    /* reset device */
    uint8_t reset = RESET;
    ::write(spi, &reset, 1);

    /* setup RX Buf 0 to receive any message */
    Write rxb0ctrl(RXB0CTRL, 0x60);
    ::write(spi, rxb0ctrl.packet, sizeof(rxb0ctrl.packet));
    interrupt_enable();
}

/*
 * disable()
 */
void MCP2515Can::disable()
{
    interrupt_disable();

    /* reset device */
    uint8_t reset = RESET;
    ::write(spi, &reset, 1);
}

/* 
 * tx_msg()
 */
void MCP2515Can::tx_msg()
{
}

/*
 * entry()
 */
void *MCP2515Can::entry()
{
    for ( ; /* forever */ ; )
    {
        sem.wait();

        uint8_t active;
        do
        {
            {
                Read caninte(CANINTE);
                ::read(spi, caninte.packet, sizeof(caninte.packet));

                Read canintf(CANINTF);
                ::read(spi, canintf.packet, sizeof(canintf.packet));

                active = caninte.data & canintf.data;
            }
            if (active & ERRI)
            {
                /* error interrupt active */
            }
            if (active & RX0I)
            {
                /* receive interrupt active */
            }
        } while (active);

        interrupt_enable();
    }

    return NULL;
}

/* 
 * interrupt_handler()
 */
void MCP2515Can::interrupt_handler()
{
    int woken = false;
    interrupt_disable();
    sem.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}
