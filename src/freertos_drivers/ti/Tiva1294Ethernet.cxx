/** \copyright
 * Copyright (c) 2016, Sidney McHarg
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
 * \file Tiva1294Ethernet.cxx
 * This file provides the network layer interface to the FreeRTOSPlus TCP stack.
 *
 *
 * @author Sidney McHarg
 * @date 23 March 2016
 */

#if !defined(PART_TM4C1294NCPDT)
#error "This driver is only valid for the TM4C1294NCPDT"
#endif

#define TraceNet 0

#include <stdint.h>
#include <new>
#include <stdio.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_emac.h"
#include "inc/hw_flash.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/emac.h"
#include "driverlib/flash.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "Tiva1294Ethernet.hxx"
#include "FreeRTOSIPConfig.h"
#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"
#include "NetworkBufferManagement.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"

extern const long unsigned cm3_cpu_clock_hz;

// empty debugging routines
void PrintBuffer(uint8_t *Buffer, size_t Len)
{
}
void PrintStr(const char *Str)
{
}
void PrintHex(uint32_t Val)
{
}
void Print(char ch)
{
}

typedef struct
{
    tEMACDMADescriptor DMA;
    volatile uint32_t Flags;
} EMACIODescriptor;

const int EMACIODescriptorExtra =
    (sizeof(EMACIODescriptor) - sizeof(tEMACDMADescriptor) + 3) / 4;

// Values for Flags in EMACIODescriptor
#define IOBUSY 0x01 //< IO presently active

#define NUM_TX_DESCRIPTORS 3
#define NUM_RX_DESCRIPTORS 5
static EMACIODescriptor RxDescriptor[NUM_RX_DESCRIPTORS],
    TxDescriptor[NUM_TX_DESCRIPTORS];
static uint16_t RxDescIndex, TxDescIndex;

static QueueHandle_t TxBufferQueue = NULL;

static QueueHandle_t xEMACEventQueue = NULL;

static TaskHandle_t xDIHTask = NULL;

static bool LinkDown, Initialised = false;

#ifdef STATIC_BUFFERS
#define BUFFER_SIZE (ipTOTAL_ETHERNET_FRAME_SIZE + ipBUFFER_PADDING)
#define BUFFER_SIZE_ROUNDED_UP ((BUFFER_SIZE + 7) & ~0x07UL)

uint8_t __attribute__((aligned(4)))
Buffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS][BUFFER_SIZE_ROUNDED_UP];

void vNetworkInterfaceAllocateRAMToBuffers(NetworkBufferDescriptor_t
        pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS]);

uint8_t RxBuffers[NUM_RX_DESCRIPTORS][ipTOTAL_ETHERNET_FRAME_SIZE],
    TxBuffers[NUM_TX_DESCRIPTORS][ipTOTAL_ETHERNET_FRAME_SIZE];
#else
#error "This implementation requires use of STATIC_BUFFERS"
#endif

static bool get_mac_address(uint8_t MACAddr[6]);
bool network_layer_preinit(void);
static bool InitialiseEthernet(void);
static bool InitialiseDIH(void);
static EMACIODescriptor *GetNextRxDescriptor(void);
static void SendData(EMACIODescriptor *pxTxIOD, uint32_t DataLength);

extern "C" void prvEMACDeferredInterruptHandlerTask(void *pvParameters);

int FramesRecv = 0, FrameLen = 0;
unsigned char *Frame;

static bool get_mac_address(uint8_t MACAddr[6])
{
    uint32_t User0, User1;
    FlashUserGet(&User0, &User1);
    if ((User0 == 0xffffffff) || (User1 == 0xffffffff))
    {
        // MAC address not programmed
        MACAddr[0] = 0x00;
        MACAddr[1] = 0x11;
        MACAddr[2] = 0x22;
        MACAddr[3] = 0x33;
        MACAddr[4] = 0x44;
        MACAddr[5] = 0x55;
        return (true);
    }
    else
    {
        // pack into MACAddr
        MACAddr[0] = (User0 >> 0) & 0xff;
        MACAddr[1] = (User0 >> 8) & 0xff;
        MACAddr[2] = (User0 >> 16) & 0xff;
        MACAddr[3] = (User1 >> 0) & 0xff;
        MACAddr[4] = (User1 >> 8) & 0xff;
        MACAddr[5] = (User1 >> 16) & 0xff;
        return (false);
    }
}

bool network_layer_preinit(void)
{
    static const uint8_t IPAddr[4] = {192, 168, 0, 100},
                         NetMask[4] = {255, 255, 255, 0},
                         Gateway[4] = {192, 168, 0, 1},
                         DNSAddr[4] = {192, 168, 0, 1};
    uint8_t MACAddr[6];

    get_mac_address(MACAddr);

    if (FreeRTOS_IPInit(IPAddr, NetMask, Gateway, DNSAddr, MACAddr) != pdPASS)
    {
        return (true);
    }
    return (false);
}

static bool InitialiseDIH(void)
{
    // create queue for deferred interrupt routine
    if (xEMACEventQueue == NULL)
        xEMACEventQueue =
            xQueueCreate(ipconfigEVENT_QUEUE_LENGTH, sizeof(uint32_t));
    if (xEMACEventQueue == NULL)
    {
        // couldn't create event queue
        return (true);
    }

    // create deferred ISR task
    if (xDIHTask == NULL)
    {
        if (xTaskCreate(prvEMACDeferredInterruptHandlerTask, "EthernetISR",
                2 * configMINIMAL_STACK_SIZE, &xDIHTask,
                configMAX_PRIORITIES - 2, NULL) != pdPASS)
        {
            return (true);
        }
    }
    return (false);
}

static bool InitialiseEthernet(void)
{
    // Ethernet

    uint8_t PHYAddr = 0;
    uint8_t MACAddr[6];
    int x;
    EMACIODescriptor *TxIOD;

    RxDescIndex = 0;
    TxDescIndex = NUM_TX_DESCRIPTORS - 1;

    if (!Initialised)
    {
        // start deferred interrupt handler task
        if (InitialiseDIH())
        {
            // failed to initiate
            return (true);
        }

        // create transmit buffer queue
        if (TxBufferQueue == 0)
            TxBufferQueue =
                xQueueCreate(NUM_TX_DESCRIPTORS, sizeof(EMACIODescriptor *));
        if (TxBufferQueue == 0)
        {
            // failed to create transmit buffer queue
            return (true);
        }

        // note initialised
        Initialised = true;
    }
    else
    {
        // reset the device
        EMACReset(EMAC0_BASE);

        // reset transmit buffer queue
        xQueueReset(TxBufferQueue);
    }

    // now run with interrupts disabled
    taskDISABLE_INTERRUPTS();

    // obtain MAC address
    if (get_mac_address(MACAddr))
    {
        // address not programmed - use default
        ;
    }

    // set up LEDs
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinConfigure(GPIO_PF0_EN0LED0);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0);
    MAP_GPIOPinConfigure(GPIO_PF4_EN0LED1);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_4);

    // setup up MAC and PHY
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);

    // wait for ready
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0))
    {
        ;
    }
    // configure for use with internal PHY
    EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN |
            EMAC_PHY_AN_100B_T_FULL_DUPLEX);

    // enable linkstate interrupts
    EMACPHYWrite(EMAC0_BASE, PHYAddr, EPHY_MISR1, EPHY_MISR1_LINKSTATEN);
    // enable PHY interrupts
    EMACPHYWrite(EMAC0_BASE, PHYAddr, EPHY_SCR, EPHY_SCR_INTEN);
    // enable PHY Interrupt Mask
    HWREG(EMAC0_BASE + EMAC_O_EPHYIM) = 1;
    // clear any outstanding interrupts
    HWREG(EMAC0_BASE + EMAC_O_EPHYMISC) = 1;

    // reset the MAC to latch the PHY
    EMACReset(EMAC0_BASE);
    // initialize the MAC and set to DMA mode
    EMACInit(EMAC0_BASE, cm3_cpu_clock_hz,
        EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED, 4, 4,
        EMACIODescriptorExtra);
    // set MAC configuration options
    EMACConfigSet(EMAC0_BASE, EMAC_CONFIG_FULL_DUPLEX |
            EMAC_CONFIG_CHECKSUM_OFFLOAD | EMAC_CONFIG_7BYTE_PREAMBLE |
            EMAC_CONFIG_IF_GAP_96BITS | EMAC_CONFIG_USE_MACADDR0 |
            EMAC_CONFIG_SA_FROM_DESCRIPTOR | EMAC_CONFIG_BO_LIMIT_1024,
        EMAC_MODE_RX_STORE_FORWARD | EMAC_MODE_TX_STORE_FORWARD |
            EMAC_MODE_TX_THRESHOLD_64_BYTES | EMAC_MODE_RX_THRESHOLD_64_BYTES,
        0);
    // initialise DMA descriptors
    for (x = 0; x < NUM_TX_DESCRIPTORS; x++)
    {
        TxIOD = &TxDescriptor[x];
        TxIOD->DMA.ui32Count = 0;
        TxIOD->DMA.DES3.pLink =
            (tEMACDMADescriptor *)((x == NUM_TX_DESCRIPTORS - 1)
                    ? TxDescriptor
                    : &TxDescriptor[x + 1]);
        TxIOD->DMA.ui32CtrlStatus = DES0_TX_CTRL_LAST_SEG |
            DES0_TX_CTRL_FIRST_SEG | DES0_TX_CTRL_INTERRUPT |
            DES0_TX_CTRL_CHAINED |
            0; // checksums dont seem to work DES0_TX_CTRL_IP_ALL_CKHSUMS;
        TxIOD->DMA.pvBuffer1 = TxBuffers[x];
        TxIOD->Flags = 0;
        if (xQueueSend(TxBufferQueue, &TxIOD, 0) != pdTRUE)
        {
            /* error insert buffer into available queue
             *
             */
        }
    }
    for (x = 0; x < NUM_RX_DESCRIPTORS; x++)
    {
        RxDescriptor[x].DMA.ui32CtrlStatus = 0;
        RxDescriptor[x].DMA.ui32Count = DES1_RX_CTRL_CHAINED |
            (ipTOTAL_ETHERNET_FRAME_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S);
        RxDescriptor[x].DMA.pvBuffer1 = RxBuffers[x];
        RxDescriptor[x].DMA.DES3.pLink =
            (tEMACDMADescriptor *)((x == NUM_RX_DESCRIPTORS - 1)
                    ? RxDescriptor
                    : &RxDescriptor[x + 1]);
        RxDescriptor[x].Flags = 0;
    }
    // set DMA descriptors in hardware
    EMACRxDMADescriptorListSet(EMAC0_BASE, (tEMACDMADescriptor *)RxDescriptor);
    EMACTxDMADescriptorListSet(EMAC0_BASE, (tEMACDMADescriptor *)TxDescriptor);
    // set hardware with MAC address
    EMACAddrSet(EMAC0_BASE, 0, MACAddr);
    // wait for link to become active: this is expected by IP
    LinkDown = true;
    while (
        (EMACPHYRead(EMAC0_BASE, PHYAddr, EPHY_BMSR) & EPHY_BMSR_LINKSTAT) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    LinkDown = false;
    // set MAC filtering options: self, broadcast, multicast
    EMACFrameFilterSet(EMAC0_BASE,
#if 0 // accept all frames
    		EMAC_FRMFILTER_RX_ALL);
#else

        EMAC_FRMFILTER_SADDR | EMAC_FRMFILTER_PASS_MULTICAST |
            EMAC_FRMFILTER_PASS_NO_CTRL);
#endif

    // clear any pending interrupts
    EMACIntClear(EMAC0_BASE,EMACIntStatus(EMAC0_BASE,false));
    // mark all receive buffers as available for DMA
    for (x = 0; x < NUM_RX_DESCRIPTORS; x++)
    {
        RxDescriptor[x].DMA.ui32CtrlStatus = DES0_RX_CTRL_OWN;
    }
    // enable transmitter and receiver
    EMACTxEnable(EMAC0_BASE);
    EMACRxEnable(EMAC0_BASE);
    // enable ethernet interrupt
	MAP_IntPrioritySet(INT_EMAC0, 0xa0);
    IntEnable(INT_EMAC0);
    // enable ethernet interrupts
    EMACIntEnable(EMAC0_BASE,
    		EMAC_INT_RECEIVE |
			EMAC_INT_TRANSMIT |
			EMAC_INT_BUS_ERROR |
			EMAC_INT_RX_WATCHDOG |
			EMAC_INT_RX_STOPPED |
			EMAC_INT_TX_UNDERFLOW |
			EMAC_INT_RX_OVERFLOW |
			EMAC_INT_TX_JABBER |
			EMAC_INT_TX_STOPPED |
			EMAC_INT_PHY);

    // reenable interrupts
    taskENABLE_INTERRUPTS();

    return(false);
}

#ifdef STATIC_BUFFERS
void vNetworkInterfaceAllocateRAMToBuffers(NetworkBufferDescriptor_t
        pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS])
{
    for (int x = 0; x < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; x++)
    {
        // set the buffer point to point past the padding area
        pxNetworkBuffers[x].pucEthernetBuffer = &(Buffers[x][ipBUFFER_PADDING]);
        // set the pointer with the buffer back to the NetworkBuffer
        *((uint32_t *)&Buffers[x][0]) = (uint32_t) & (pxNetworkBuffers[x]);
    }
}
#endif

BaseType_t xNetworkInterfaceInitialise(void)
{
    if (InitialiseEthernet())
        return (pdFAIL);
    else
        return (pdPASS);
}

static EMACIODescriptor *GetNextRxDescriptor(void)
{
    uint16_t inx = RxDescIndex;
    taskENTER_CRITICAL();
    while (RxDescriptor[inx].DMA.ui32CtrlStatus & DES0_RX_CTRL_OWN)
    {
        inx++;
        if (inx >= NUM_RX_DESCRIPTORS)
            inx = 0;
        if (inx == RxDescIndex)
        {
            taskEXIT_CRITICAL();
            return (NULL);
        }
    }
    RxDescIndex = (inx == NUM_RX_DESCRIPTORS - 1) ? 0 : (inx + 1);
    taskEXIT_CRITICAL();
    return (&RxDescriptor[inx]);
}

static void SendData(EMACIODescriptor *pxTxIOD, uint32_t DataLength)
{
    if (TraceNet)
        PrintBuffer((uint8_t *)pxTxIOD->DMA.pvBuffer1, DataLength);
    pxTxIOD->DMA.ui32Count = // DES1_TX_CTRL_SADDR_INSERT |
        (DataLength) << DES1_TX_CTRL_BUFF1_SIZE_S;
    pxTxIOD->DMA.ui32CtrlStatus = DES0_TX_CTRL_LAST_SEG |
        DES0_TX_CTRL_FIRST_SEG | DES0_TX_CTRL_INTERRUPT |
        // DES0_TX_CTRL_IP_HDR_CHKSUM | //DES0_TX_CTRL_IP_HDR_PAY_CHKSUM |
        // //DES0_TX_CTRL_IP_ALL_CKHSUMS |
        DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN;
    EMACTxDMAPollDemand(EMAC0_BASE);
}

BaseType_t xNetworkInterfaceOutput(
    NetworkBufferDescriptor_t *const pxDescriptor, BaseType_t xReleaseAfterSend)
{
    EMACIODescriptor *pxTxIOD;

    /* Obtain a DMA transmit descriptor */
    if (xQueueReceive(TxBufferQueue, &pxTxIOD, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        // no buffer available
        return pdFALSE;
    }
    /* mark buffer as busy */
    pxTxIOD->Flags = IOBUSY;

    /* copy data to dedicated TxBuffer */
    memcpy(pxTxIOD->DMA.pvBuffer1, pxDescriptor->pucEthernetBuffer,
        pxDescriptor->xDataLength);

    /* Send the data */
    SendData(pxTxIOD, pxDescriptor->xDataLength);

    /* Call the standard trace macro to log the send event. */
    iptraceNETWORK_INTERFACE_TRANSMIT();

    if (xReleaseAfterSend != pdFALSE)
    {
        /* Release the NetworkBufferDescriptor
         */
        vReleaseNetworkBufferAndDescriptor(pxDescriptor);
        // PrintStr("No wait\n");
    }

    return pdTRUE;
}

void prvEMACDeferredInterruptHandlerTask(void *pvParameters)
{
    uint32_t IntStatus, CtrlStatus;
    NetworkBufferDescriptor_t *pxDescriptor;
    EMACIODescriptor *pxRxIOD;

    IPStackEvent_t xRxEvent;

    for (;;)
    {
        /* Wait for the Ethernet MAC interrupt */

        while (
            xQueueReceive(xEMACEventQueue, &IntStatus, portMAX_DELAY) == pdTRUE)
        {
            if (IntStatus & EMAC_INT_RECEIVE)
            {
                /* Deal with completed receive DMA descriptors */
                while ((pxRxIOD = GetNextRxDescriptor()) != NULL)
                {
                    /* Allocate new NetworkBufferDescriptor with buffer to pass
                     * results IPTask
                     */

                    pxDescriptor = pxGetNetworkBufferWithDescriptor(
                        ipTOTAL_ETHERNET_FRAME_SIZE, pdMS_TO_TICKS(50));
                    if (pxDescriptor == NULL)
                    {
                        /* No buffer available: break out of receive while loop
                         */
                        //__asm("BKPT #01");
                        break;
                    }

                    CtrlStatus = pxRxIOD->DMA.ui32CtrlStatus;
                    pxDescriptor->xDataLength =
                        (CtrlStatus & DES0_RX_STAT_FRAME_LENGTH_M) >>
                        DES0_RX_STAT_FRAME_LENGTH_S;
                    memcpy(pxDescriptor->pucEthernetBuffer,
                        (uint8_t *)pxRxIOD->DMA.pvBuffer1,
                        pxDescriptor->xDataLength);

                    // temp debugging
                    if (TraceNet)
                    {
                        Frame = pxDescriptor->pucEthernetBuffer;
                        FrameLen = pxDescriptor->xDataLength;
                        PrintHex(pxRxIOD->DMA.ui32CtrlStatus);
                        PrintStr("-");
                        PrintBuffer(Frame, FrameLen);
                    }

                    // mark as available for DMA
                    pxRxIOD->DMA.ui32CtrlStatus = DES0_RX_CTRL_OWN;
                    // and ensure DMA is active for Rx
                    EMACRxDMAPollDemand(EMAC0_BASE);
                    if (!(CtrlStatus & DES0_RX_STAT_ERR))
                    {
                        // valid frame
                        if (CtrlStatus & DES0_RX_STAT_LAST_DESC)
                        {
                            // last frame
                            if (eConsiderFrameForProcessing(
                                    pxDescriptor->pucEthernetBuffer) ==
                                eProcessBuffer)
                            {
                                /* The event about to be sent to the TCP/IP is
                                 * an Rx event. */
                                xRxEvent.eEventType = eNetworkRxEvent;

                                /* pvData is used to point to the network buffer
                                descriptor that
                                references the received data. */
                                xRxEvent.pvData = (void *)pxDescriptor;

                                /* Send the data to the TCP/IP stack. */
                                if (xSendEventStructToIPTask(&xRxEvent, 0) ==
                                    pdFALSE)
                                {
                                    /* The buffer could not be sent to the IP
                                    task so the buffer
                                    must be released. */
                                    vReleaseNetworkBufferAndDescriptor(
                                        pxDescriptor);

                                    /* Make a call to the standard trace macro
                                    to log the
                                    occurrence. */
                                    iptraceETHERNET_RX_EVENT_LOST();
                                    PrintStr("RX Event Lost\n");
                                }
                                else
                                {
                                    /* The message was successfully sent to the
                                    TCP/IP stack.
                                    Call the standard trace macro to log the
                                    occurrence. */
                                    iptraceNETWORK_INTERFACE_RECEIVE();
                                }
                            }
                            else
                            {
                                /* The Ethernet frame can be dropped, but the
                                Ethernet buffer
                                must be released. */
                                vReleaseNetworkBufferAndDescriptor(
                                    pxDescriptor);
                            }
                        }
                    }
                }
            }

            if (IntStatus & EMAC_INT_ABNORMAL_INT)
            {
                // some sort of error condition occurred
                PrintStr("EMAC Abnormal Int");
                PrintHex(IntStatus);
                PrintStr("\n");
            }

            if (IntStatus & EMAC_INT_PHY)
            {
                // PHY report
                bool OldState = LinkDown;
                if ((EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) &
                        EPHY_BMSR_LINKSTAT) == 0)
                {
                    // link down
                    LinkDown = true;
                }
                else
                {
                    // link up
                    LinkDown = false;
                }
                // clear PHY interrupt
                HWREG(EMAC0_BASE + EMAC_O_EPHYMISC) = 1;
                PrintStr("EMAC PHY Int");
                if (OldState != LinkDown)
                {
                    if (LinkDown)
                    {
                        PrintStr(" link down\n");
                        FreeRTOS_NetworkDown();
                    }
                    else
                    {
                        // there is no counterpart to NetworkDown
                        PrintStr(" link up\n");
                    }
                }
                else
                    Print('\n');
            }
        }
    }
}

extern "C" void ethernet_interrupt_handler(void)
{
    uint32_t status = EMACIntStatus(EMAC0_BASE, true);
    BaseType_t TaskWoken = pdFALSE;
    EMACIODescriptor *pxTxIOD;
    EMACIntClear(EMAC0_BASE, status);
    if (status & EMAC_INT_PHY)
    {
        // reset the PHY interrupt to avoid recurring interrupts by reading
        // MISR1
        uint16_t misr1 = EMACPHYRead(EMAC0_BASE, 0, EPHY_MISR1);
        (void)misr1;
    }

    if ((status & EMAC_INT_TRANSMIT) || (status & EMAC_INT_TX_STOPPED))
    {
        // process transmit finish here in ISR
        for (int inx = 0; inx < NUM_TX_DESCRIPTORS; inx++)
        {
            pxTxIOD = &TxDescriptor[inx];
            if (((pxTxIOD->DMA.ui32CtrlStatus & DES0_TX_CTRL_OWN) == 0) &&
                (pxTxIOD->Flags & IOBUSY))
            {
                // completed IO - reset IOBUSY flag
                pxTxIOD->Flags = 0;
                xQueueSendFromISR(TxBufferQueue, &pxTxIOD, &TaskWoken);
            }
        }
        status &= ~(EMAC_INT_TRANSMIT | EMAC_INT_TX_STOPPED);
    }

    if (status != 0)
    {
        // still have an interrupt to report
        xQueueSendFromISR(xEMACEventQueue, &status, &TaskWoken);
    }

    if (TaskWoken == pdTRUE)
        taskYIELD();
}
