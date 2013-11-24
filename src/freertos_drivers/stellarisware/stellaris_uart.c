/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file stellaris_can.c
 * This file implements a USB CDC device driver layer specific to stellarisware.
 *
 * @author Stuart W. Baker
 * @date 11 January 2013
 */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "serial.h"

/* prototypes */
static int stellaris_uart_init(devtab_t *dev);
static void stellaris_uart_enable(devtab_t *dev);
static void stellaris_uart_disable(devtab_t *dev);
static void stellaris_uart_tx_char(devtab_t *dev);

/** Private data for this implementation of serial.
 */
typedef struct stellaris_uart_priv
{
    SerialPriv serialPriv; /**< common private data */
    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    char txPending; /**< transmission currently pending */
} StellarisUartPriv;

/* private data for the serial device */
static StellarisUartPriv uart_priv[8] =
{
    {
        .base = UART0_BASE,
        .interrupt = INT_UART0,
        .txPending = 0
    },
    {
        .base = UART1_BASE,
        .interrupt = INT_UART1,
        .txPending = 0
    },
    {
        .base = UART2_BASE,
        .interrupt = INT_UART2,
        .txPending = 0
    },
    {
        .base = UART3_BASE,
        .interrupt = INT_UART3,
        .txPending = 0
    },
    {
        .base = UART4_BASE,
        .interrupt = INT_UART4,
        .txPending = 0
    },
    {
        .base = UART5_BASE,
        .interrupt = INT_UART5,
        .txPending = 0
    },
    {
        .base = UART6_BASE,
        .interrupt = INT_UART6,
        .txPending = 0
    },
    {
        .base = UART7_BASE,
        .interrupt = INT_UART7,
        .txPending = 0
    }
};

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart0, "/dev/ser0", stellaris_uart_init, &uart_priv[0]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart1, "/dev/ser1", stellaris_uart_init, &uart_priv[1]);

#if 0
/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart2, "/dev/ser2", stellaris_uart_init, &uart_priv[2]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart3, "/dev/ser3", stellaris_uart_init, &uart_priv[3]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart4, "/dev/ser4", stellaris_uart_init, &uart_priv[4]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart5, "/dev/ser5", stellaris_uart_init, &uart_priv[5]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart6, "/dev/ser6", stellaris_uart_init, &uart_priv[6]);

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(uart7, "/dev/ser7", stellaris_uart_init, &uart_priv[7]);
#endif

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int stellaris_uart_init(devtab_t *dev)
{
    StellarisUartPriv *priv = dev->priv;
    
    switch (priv->base)
    {
        default:
            return -1;
        case UART0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
            break;
        case UART1_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
            break;
        case UART2_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
            break;
        case UART3_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
            break;
        case UART4_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
            break;
        case UART5_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
            break;
        case UART6_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
            break;
        case UART7_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
            break;
    }
    
    MAP_UARTConfigSetExpClk(priv->base, MAP_SysCtlClockGet(), 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(priv->base);
    MAP_UARTTxIntModeSet(priv->base, UART_TXINT_MODE_EOT);
    MAP_IntEnable(priv->interrupt);
    MAP_UARTIntEnable(priv->base, UART_INT_RX | UART_INT_RT);
    
    priv->serialPriv.enable = stellaris_uart_enable;
    priv->serialPriv.disable = stellaris_uart_disable;
    priv->serialPriv.tx_char = stellaris_uart_tx_char;
    
    return serial_init(dev);
}

/** Enable use of the device.
 * @param dev device to enable
 */
static void stellaris_uart_enable(devtab_t *dev)
{
    StellarisUartPriv *priv = dev->priv;
    MAP_UARTEnable(priv->base);
}

/** Disable use of the device.
 * @param dev device to disable
 */
static void stellaris_uart_disable(devtab_t *dev)
{
    StellarisUartPriv *priv = dev->priv;
    MAP_UARTDisable(priv->base);
}

/* Try and transmit a message.
 * @param dev device to transmit message on
 */
static void stellaris_uart_tx_char(devtab_t *dev)
{
    StellarisUartPriv *priv = dev->priv;
    
    if (priv->txPending == 0)
    {
        unsigned char data;
        if (MAP_UARTSpaceAvail(priv->base) &&
            os_mq_timedreceive(priv->serialPriv.txQ, &data, 0) == OS_MQ_NONE)
        {
            MAP_IntDisable(priv->interrupt);
            priv->txPending = 1;
            MAP_UARTCharPutNonBlocking(priv->base, data);
            MAP_UARTIntEnable(priv->base, UART_INT_TX);
            MAP_IntEnable(priv->interrupt);
        }
    }
}

/** Common interrupt handler for all UART devices.
 * @param dev device to handle and interrupt for
 */
void uart_interrupt_handler(devtab_t *dev)
{
    StellarisUartPriv *priv = dev->priv;
    int woken = false;
    
    /* get and clear the interrupt status */
    unsigned long status = MAP_UARTIntStatus(priv->base, true);    
    MAP_UARTIntClear(priv->base, status);
    

    /* receive charaters as long as we can */
    while (MAP_UARTCharsAvail(priv->base))
    {
        long data = MAP_UARTCharGetNonBlocking(priv->base);
        if (data >= 0)
        {
            unsigned char c = data;
            if (os_mq_send_from_isr(priv->serialPriv.rxQ, &c, &woken) == OS_MQ_FULL)
            {
                priv->serialPriv.overrunCount++;
            }
        }
    }
    /* tranmit a character if we have pending tx data */
    if (priv->txPending)
    {
        if (MAP_UARTSpaceAvail(priv->base))
        {
            unsigned char data;
            if (os_mq_receive_from_isr(priv->serialPriv.txQ, &data, &woken) == OS_MQ_NONE)
            {
                MAP_UARTCharPutNonBlocking(priv->base, data);
            }
            else
            {
                /* no more data pending */
                priv->txPending = 0;
                MAP_UARTIntDisable(priv->base, UART_INT_TX);
            }
        }
    }
}

/** UART0 interrupt handler.
 */
void uart0_interrupt_handler(void)
{
    uart_interrupt_handler(&uart0);
}

/** UART1 interrupt handler.
 */
void uart1_interrupt_handler(void)
{
    uart_interrupt_handler(&uart1);
}
#if 0

/** UART2 interrupt handler.
 */
void uart2_interrupt_handler(void)
{
    uart_interrupt_handler(&uart2);
}

/** UART3 interrupt handler.
 */
void uart3_interrupt_handler(void)
{
    uart_interrupt_handler(&uart3);
}
/** UART4 interrupt handler.
 */
void uart4_interrupt_handler(void)
{
    uart_interrupt_handler(&uart4);
}

/** UART5 interrupt handler.
 */
void uart5_interrupt_handler(void)
{
    uart_interrupt_handler(&uart5);
}

/** UART6 interrupt handler.
 */
void uart6_interrupt_handler(void)
{
    uart_interrupt_handler(&uart6);
}

/** UART7 interrupt handler.
 */
void uart7_interrupt_handler(void)
{
    uart_interrupt_handler(&uart7);
}
#endif
