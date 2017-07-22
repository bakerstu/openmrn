/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file simplelink_v1.h
 * Compatibility header for compiling with the CC3200-sdk.
 *
 * @author Balazs Racz
 * @date 19 Mar 2017
 */

#ifndef _FREEERTOS_DRIVERS_TI_CC3200_COMPAT_SIMPLELINK_V1_H_
#define _FREEERTOS_DRIVERS_TI_CC3200_COMPAT_SIMPLELINK_V1_H_

#ifndef SL_API_V2

#define SL_FileLen FileLen
#define SL_AllocatedLen AllocatedLen
#define SL_ssid ssid
#define SL_ssid_len ssid_len
#define SL_sec_type sec_type
#define SL_rssi rssi
#define SL_Event Event
#define SL_EventData EventData
#define SL_ssid_name ssid_name
#define SL_reason_code reason_code
#define SL_ipAcquiredV4 ipAcquiredV4
#define SL_ip ip
#define SL_ipLeased ipLeased
#define SL_ip_address ip_address
#define SL_socketAsyncEvent socketAsyncEvent
#define SL_status status
#define SL_type type
#define SL_token_value token_value
#define SL_data data
#define SL_len len
#define SL_httpTokenName httpTokenName
#define SL_ChipFwAndPhyVersion(ver) ver.ChipFwAndPhyVersion
#define SL_NETAPP_MAX_TOKEN_VALUE_LEN MAX_TOKEN_VALUE_LEN
#define SL_NonblockingEnabled NonblockingEnabled

#define SL_WLAN_EVENT_CONNECT     SL_WLAN_CONNECT_EVENT
#define SL_WLAN_EVENT_DISCONNECT  SL_WLAN_DISCONNECT_EVENT
#define SL_WLAN_EVENT_STA_ADDED   SL_WLAN_STA_CONNECTED_EVENT
#define SL_WLAN_EVENT_STA_REMOVED SL_WLAN_STA_DISCONNECTED_EVENT

#endif

#endif // _FREEERTOS_DRIVERS_TI_CC3200_COMPAT_SIMPLELINK_V1_H_
