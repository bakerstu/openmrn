/*
 * FreeRTOSIPConfig.h
 *
 *  Created on: Nov 24, 2015
 *      Author: Sidney McHarg
 */

#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

// RTOS task related
#define ipconfigIP_TASK_STACK_SIZE_WORDS				256
#define ipconfigIP_TASK_PRIORITY						(configMAX_PRIORITIES-1)
#define ipconfigEVENT_QUEUE_LENGTH						(5+ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS)

// driver specific
#define ipconfigBYTE_ORDER								pdFREERTOS_LITTLE_ENDIAN
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM			0
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM			1
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES		0

#define ipconfigNETWORK_MTU     						1526
#define ipconfigPACKET_FILLER_SIZE						0
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS			15
#define ipconfigUSE_LINKED_RX_MESSAGES					0

// TCP specific
#define ipconfigUSE_TCP									1
#define ipconfigUSE_TCP_WIN       						1
#define ipconfigTCP_HANG_PROTECTION						1
#define ipconfigTCP_HANG_PROTECTION_TIME				30

#define ipconfigTCP_MSS         						1460
#define ipconfigTCP_TX_BUFFER_LENGTH  					( 2 * ipconfigTCP_MSS )
#define ipconfigTCP_RX_BUFFER_LENGTH  					( 2 * ipconfigTCP_MSS )

// DNS
#define ipconfigUSE_DNS									0

// DHCP
#define ipconfigUSE_DHCP								1

// Callback hooks
#define ipconfigUSE_NETWORK_EVENT_HOOK					1

// SignalSocket
#define ipconfigSUPPORT_SIGNALS							1

// select
#define ipconfigSUPPORT_SELECT_FUNCTION					1

// rand function prototype to avoid compile warnings
int rand(void);

#endif /* FREERTOSIPCONFIG_H_ */
