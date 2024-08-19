/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxWiFi.cxx
 * This file instantiates and initializes the CC32xx Wi-Fi.
 *
 * @author Stuart W. Baker
 * @date 18 March 2016
 */

//#define LOGLEVEL INFO

#define SUPPORT_SL_R1_API

#include "CC32xxWiFi.hxx"
#include "CC32xxSocket.hxx"

#include "freertos_drivers/common/WifiDefs.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "utils/format_utils.hxx"
#include "utils/logging.h"

#include <unistd.h>

// Simplelink includes
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/source/protocol.h>

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::WlanEvent : public ::SlWlanEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::NetAppEvent : public ::SlNetAppEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::SockEvent : public ::SlSockEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::SockTriggerEvent : public ::SlSockTriggerEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::HttpServerEvent : public ::SlNetAppHttpServerEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::HttpServerResponse : public ::SlNetAppHttpServerResponse_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::NetAppRequest : public ::SlNetAppRequest_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::NetAppResponse : public ::SlNetAppResponse_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::FatalErrorEvent : public ::SlDeviceFatal_t {};

/** This is not a class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs.
 */
static SlFdSet_t rfds;

/** This is not a class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs.
 */
static SlFdSet_t wfds;

/** This is not a class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs.
 */
static SlFdSet_t efds;

/** the highest file descriptor to select on */
static int fdHighest;

/** This is not a class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs.  @ref slSockets keeps track of
 * all the possible CC32x sockets.  On the CC3200, SL_MAX_SOCKETS is 8.  On the
 * CC3220, SL_MAX_SOCKETS is 16.  Unused slots will have a value of -1, which
 * is initialized in the @ref CC32xxWifi class constructor.  If a socket
 * becomes interesting (is added to a select() call list), it will be added to
 * an open slot in @ref slSockets by its CC32xx socket descriptor space value.
 * When a socket is closed, it is removed from the from @ref slSockets and the
 * resulting empty slot is returned to a value of -1.
 *
 * Note:  not all socket descriptors will be added to @ref slSockets for
 * tracking.  Only those sockets which are added to a select() call list will
 * be tracked by @ref slSockets.
 */
static int16_t slSockets[SL_MAX_SOCKETS];

/** Find the new highest fd to select on.
 */
void new_highest()
{
    fdHighest = 0;
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] != -1)
        {
            if (slSockets[i] > fdHighest)
            {
                if (SL_SOCKET_FD_ISSET(slSockets[i], &rfds) ||
                    SL_SOCKET_FD_ISSET(slSockets[i], &wfds) ||
                    SL_SOCKET_FD_ISSET(slSockets[i], &efds))
                {
                    fdHighest = slSockets[i];
                }
            }
        }
    }
}

/** Add an interesting socket.
 * @param sd number to add
 */
void add_socket(int16_t sd)
{
    if (sd > fdHighest)
    {
        fdHighest = sd;
    }
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == sd)
        {
            /* already known */
            return;
        }
    }
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == -1)
        {
            slSockets[i] = sd;
            break;
        }
    }
}

/** Delete an interesting socket.
 * @parqam sd number to delete
 */
void del_socket(int16_t sd)
{
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == sd)
        {
            slSockets[i] = -1;
            break;
        }
    }
    new_highest();
}

/*
 * CC32xxWiFi::CC32xxWiFi()
 */
CC32xxWiFi::CC32xxWiFi()
    : ipAddress(0)
    , ipAcquiredCallback_(nullptr)
    , wakeup(-1)
    , rssi(0)
    , wlanRole(WlanRole::UNKNOWN)
    , started(false)
    , connected(0)
    , connectionFailed(0)
    , ipAcquired(0)
    , ipLeased(0)
    , smartConfigStart(0)
    , securityFailure(0)
{
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        slSockets[i] = -1;
    }
    SL_SOCKET_FD_ZERO(&rfds);
    SL_SOCKET_FD_ZERO(&wfds);
    SL_SOCKET_FD_ZERO(&efds);
    ssid[0] = '\0';

    add_http_get_token_callback(
        "__SL_G_UNA", bind(&CC32xxWiFi::http_get_ip_address, this));
}

uint8_t CC32xxWiFi::security_type_to_simplelink(SecurityType sec_type)
{
    switch (sec_type)
    {
        default:
        case SEC_OPEN:
            return SL_WLAN_SEC_TYPE_OPEN;
        case SEC_WEP:
            return SL_WLAN_SEC_TYPE_WEP;
        case SEC_WPA2:
            return SL_WLAN_SEC_TYPE_WPA_WPA2;
    }
}

CC32xxWiFi::SecurityType CC32xxWiFi::security_type_from_scan(unsigned sec_type)
{
    auto t = SL_WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sec_type);
    switch (t)
    {
        default:
        case SL_WLAN_SECURITY_TYPE_BITMAP_OPEN:
            return SEC_OPEN;
        case SL_WLAN_SECURITY_TYPE_BITMAP_WEP:
            return SEC_WEP;
        case SL_WLAN_SECURITY_TYPE_BITMAP_WPA:
        case SL_WLAN_SECURITY_TYPE_BITMAP_WPA2:
        case SL_WLAN_SECURITY_TYPE_BITMAP_WPA |
            SL_WLAN_SECURITY_TYPE_BITMAP_WPA2:
            return SEC_WPA2;
    }
}


CC32xxWiFi::SecurityType CC32xxWiFi::security_type_from_simplelink(uint8_t sec_type)
{
    switch (sec_type)
    {
        default:
        case SL_WLAN_SEC_TYPE_OPEN:
            return SEC_OPEN;
        case SL_WLAN_SEC_TYPE_WEP:
            return SEC_WEP;
        case SL_WLAN_SEC_TYPE_WPS_PBC:
        case SL_WLAN_SEC_TYPE_WPS_PIN:
        case SL_WLAN_SEC_TYPE_WPA_ENT:
        case SL_WLAN_SEC_TYPE_WPA_WPA2:
            return SEC_WPA2;
    }
}

/*
 * CC32xxWiFi::wlan_country_code_get()
 */
CountryCode CC32xxWiFi::wlan_country_code_get()
{
    uint16_t config_option = SL_WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE;
    uint16_t len = 3;
    uint8_t country[3];

    int result = sl_WlanGet(SL_WLAN_CFG_GENERAL_PARAM_ID, &config_option, &len,
                            (uint8_t*)country);
    if (result >= 0)
    {
        if (!strcmp((const char*)country, "US"))
        {
            return CountryCode::US;
        }
        else if (!strcmp((const char*)country, "EU"))
        {
            return CountryCode::EU;
        }
        else if (!strcmp((const char*)country, "JP"))
        {
            return CountryCode::JP;
        }
    }

    return CountryCode::UNKNOWN;
}

/*
 * CC32xxWiFi::wlan_country_code_set()
 */
int CC32xxWiFi::wlan_country_code_set(CountryCode cc, bool restart)
{
    if (cc != wlan_country_code_get())
    {
        const char *country;
        switch (cc)
        {
            case CountryCode::US:
                country = "US";
                break;
            case CountryCode::EU:
                country = "EU";
                break;
            case CountryCode::JP:
                country = "JP";
                break;
            default:
                return -1;
        }

        int result = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
                                SL_WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2,
                                (const uint8_t*)country);

        if (result < 0)
        {
            return -1;
        }
        if (restart)
        {
            sl_Stop(0xFFFF);
            sl_Start(0, 0, 0);
        }
    }

    return 0;
}

/*
 * CC32xxWiFi::wlan_set_scan_params()
 */
void CC32xxWiFi::wlan_set_scan_params(int mask, int min_rssi)
{
    SlWlanScanParamCommand_t param_config = {0};
    uint16_t option = SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS;
    uint16_t param_len = sizeof(param_config);
    int ret = sl_WlanGet(SL_WLAN_CFG_GENERAL_PARAM_ID, &option, &param_len,
        (_u8 *)&param_config);
    SlCheckResult(ret);
    bool apply = false;
    if (mask >= 0 && param_config.ChannelsMask != (uint32_t)mask)
    {
        param_config.ChannelsMask = mask;
        apply = true;
    }
    if (min_rssi < 0 && param_config.RssiThreshold != min_rssi)
    {
        param_config.RssiThreshold = min_rssi;
        apply = true;
    }
    if (!apply)
    {
        return;
    }
    ret = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
        SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS, sizeof(param_config),
        (_u8 *)&param_config);
    SlCheckResult(ret);
}

/*
 * CC32xxWiFi::wlan_profile_add()
 */
int CC32xxWiFi::wlan_profile_add(const char *ssid, SecurityType sec_type,
                                 const char *key, unsigned priority)
{
    SlWlanSecParams_t sec_params;
    sec_params.Key = (int8_t*)key;
    sec_params.KeyLen = (key == nullptr) ? 0 : strlen(key);
    sec_params.Type = security_type_to_simplelink(sec_type);

    int16_t result = sl_WlanProfileAdd((const int8_t*)ssid, strlen(ssid),
                                       nullptr, &sec_params, nullptr,
                                       priority, 0);

    return (result >= 0) ? result : -1;
}

/*
 * CC32xxWiFi::wlan_profile_del()
 */
int CC32xxWiFi::wlan_profile_del(int index)
{
    return sl_WlanProfileDel(index);
}

/*
 * CC32xxWiFi::wlan_profile_del()
 */
int CC32xxWiFi::wlan_profile_del(const char *ssid)
{
    for (int i = 0; i < 7; ++i)
    {
        char name[33];
        if (wlan_profile_get(i, name, nullptr, nullptr) != 0)
        {
            /* invalid entry, move onto the next one */
            continue;
        }

        if (strcmp(name, ssid) == 0)
        {
            /* found a match */
            return sl_WlanProfileDel(i);
        }
    }

    /* no match found */
    return -1;
}

/*
 * CC32xxWiFi::wlan_profile_get()
 */
int CC32xxWiFi::wlan_profile_get(int index, char ssid[],
                                 SecurityType *sec_type, uint32_t *priority)
{
    SlWlanSecParams_t sec_params;
    int16_t ssid_len;
    
    int16_t result = sl_WlanProfileGet(index, (int8_t*)ssid, &ssid_len,
                                       nullptr, &sec_params, nullptr, priority);

    if (result < 0)
    {
        return -1;
    }

    ssid[ssid_len] = '\0';

    if (sec_type)
    {
        *sec_type = security_type_from_simplelink(sec_params.Type);
    }

    return 0;
}

/*
 * CC32xxWiFi::wlan_profile_text_none()
 */
bool CC32xxWiFi::wlan_profile_test_none()
{
    for (int i = 0; i < 7; ++i)
    {
        char ssid[33];
        if (wlan_profile_get(i, ssid, nullptr, nullptr) == 0)
        {
            return false;
        }
    }

    return true;
}

/*
 * CC32xxWiFi::wlan_power_policy_get()
 */
int CC32xxWiFi::wlan_power_policy_get(WlanPowerPolicy *wpp)
{
    uint8_t sl_wpp = 0;
    SlWlanPmPolicyParams_t params;
    int length = sizeof(params);

    int result = sl_WlanPolicyGet(SL_WLAN_POLICY_PM, &sl_wpp, (uint8_t*)&params,
                                  (uint8_t*)&length);

    if (result != 0)
    {
        return -1;
    }

    switch (sl_wpp)
    {
        default:
            return -1;
        case SL_WLAN_ALWAYS_ON_POLICY:
            *wpp = WLAN_ALWAYS_ON_POLICY;
            break;
        case SL_WLAN_LOW_LATENCY_POLICY:
            *wpp = WLAN_LOW_LATENCY_POLICY;
            break;
        case SL_WLAN_NORMAL_POLICY:
            *wpp = WLAN_NORMAL_POLICY;
            break;
        case SL_WLAN_LOW_POWER_POLICY:
            *wpp = WLAN_LOW_POWER_POLICY;
            break;
    }

    return 0;
}

/*
 * CC32xxWiFi::wlan_power_policy_set()
 */
int CC32xxWiFi::wlan_power_policy_set(WlanPowerPolicy wpp)
{
    int result;
    WlanPowerPolicy temp_wpp;
    result = wlan_power_policy_get(&temp_wpp);
    if (result != 0)
    {
        return -1;
    }

    if (temp_wpp != wpp)
    {
        uint8_t sl_wpp;
        switch (wpp)
        {
            default:
                return -1;
            case WLAN_ALWAYS_ON_POLICY:
                sl_wpp = SL_WLAN_ALWAYS_ON_POLICY;
                break;
            case WLAN_LOW_LATENCY_POLICY:
                sl_wpp = SL_WLAN_LOW_LATENCY_POLICY;
                break;
            case WLAN_NORMAL_POLICY:
                sl_wpp = SL_WLAN_NORMAL_POLICY;
                break;
            case WLAN_LOW_POWER_POLICY:
                sl_wpp = SL_WLAN_LOW_POWER_POLICY;
                break;
        }
        result = sl_WlanPolicySet(SL_WLAN_POLICY_PM, sl_wpp, NULL, 0);
    }

    return (result != 0) ? -1 : 0;
}

void CC32xxWiFi::wlan_connection_policy_set(WlanConnectionPolicy policy) {
    uint8_t past_policy;
    uint8_t mask;
    uint8_t desired_policy;

    int length = sizeof(past_policy);
    int ret;
    ret = sl_WlanPolicyGet(
        SL_WLAN_POLICY_CONNECTION, &past_policy, 0, (_u8 *)&length);
    if (!ret)
    {
        LOG(WARNING, "Failed to get past connection policy.");
        past_policy = 0; // This will typically result in a write.
    }

    if (policy == WLAN_CONNECTION_NO_CHANGE)
    {
        //  SL_WLAN_CONNECTION_POLICY(Auto,Fast,anyP2P,autoProvisioning)
        mask = SL_WLAN_CONNECTION_POLICY(1, 0, 1, 1);
        desired_policy = SL_WLAN_CONNECTION_POLICY(1, 0, 0, 0);
    }
    else
    {
        mask = SL_WLAN_CONNECTION_POLICY(1, 1, 1, 1);
        desired_policy = SL_WLAN_CONNECTION_POLICY(
            1, policy == WLAN_CONNECTION_FAST_RECONNECT ? 1 : 0, 0, 0);
    }

    if ((past_policy & mask) != (desired_policy & mask))
    {
        sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, desired_policy, NULL, 0);
    }
}

/*
 * CC32xxWiFi::wlan_rescan()
 */
void CC32xxWiFi::wlan_rescan()
{
    // This will trigger an immediate rescan and then every 10 minutes a new
    // scan will be done if we have no connection.
    unsigned long intervalInSeconds = 600;
    sl_WlanPolicySet(SL_WLAN_POLICY_SCAN, 1 /*enable*/, (unsigned char *)
                     &intervalInSeconds,sizeof(intervalInSeconds));    
}
    
/*
 * CC32xxWiFi::wlan_network_list_get()
 */
int CC32xxWiFi::wlan_network_list_get(WlanNetworkEntry *entries, size_t count)
{
    SlWlanNetworkEntry_t* sl_entries = new SlWlanNetworkEntry_t[count];

    int result = sl_WlanGetNetworkList(0, count, sl_entries);

    for (int i = 0; i < result; ++i)
    {
        entries[i].ssid.assign((char*)sl_entries[i].Ssid, sl_entries[i].SsidLen);
        entries[i].sec_type = security_type_from_scan(sl_entries[i].SecurityInfo);
        entries[i].rssi = sl_entries[i].Rssi;
    }

    delete [] sl_entries;
    return result;
}

/*
 * CC32xxWiFi::wlan_mac()
 */
void CC32xxWiFi::wlan_mac(uint8_t mac[6])
{
    uint16_t  len = 6;

    sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, nullptr, &len, mac);
}

/*
 * CC32xxWiFi::wlan_set_mac()
 */
void CC32xxWiFi::wlan_set_mac(uint8_t mac[6])
{
    sl_NetCfgSet(SL_NETCFG_MAC_ADDRESS_SET, 1, 6, mac);
}

/*
 * CC32xxWiFi::test_mode_start()
 */
void CC32xxWiFi::test_mode_start()
{
    os_thread_create(nullptr, "sl_Task", os_thread_get_priority_max(), 2048,
                     sl_Task, nullptr);

    // the following code sequnce is taken from the Radio Test Tool
    // application example.

    #define CHANNEL_MASK_ALL 0x1FFF
    #define RSSI_TH_MAX      -95

    int mode = sl_Start(0, 0, 0);
    if (mode != ROLE_STA)
    {
        mode = sl_WlanSetMode(ROLE_STA);
        sl_Stop(0xFF);
        sl_Start(0, 0, 0);
    }

    // set policy to auto only
    sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                     SL_WLAN_CONNECTION_POLICY(1, 0, 0, 0), nullptr, 0);

    // disable auto provisioning
    sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, nullptr, 0);

    // delete all stored Wi-Fi profiles
    sl_WlanProfileDel(0xFF);

    // enable DHCP client
    sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0, 0);

    // disable IPv6
    uint32_t if_bitmap = !(SL_NETCFG_IF_IPV6_STA_LOCAL |
                           SL_NETCFG_IF_IPV6_STA_GLOBAL);
    sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE, sizeof(if_bitmap),
                 (const uint8_t*)&if_bitmap);

    // configure scan parameters to default
    SlWlanScanParamCommand_t scan_default;
    scan_default.ChannelsMask = CHANNEL_MASK_ALL;
    scan_default.RssiThreshold = RSSI_TH_MAX;
    sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
               SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS,
               sizeof(scan_default), (uint8_t*)&scan_default);

    // disable scans
    uint8_t config_opt = SL_WLAN_SCAN_POLICY(0, 0);
    sl_WlanPolicySet(SL_WLAN_POLICY_SCAN, config_opt, nullptr, 0);

    // set TX power 1v1 to max
    uint8_t power = 0;
    sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
               SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (uint8_t*)&power);

    // set NWP power policy to "normal"
    sl_WlanSet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, 0, 0);

    // unregister mDNS services
    sl_NetAppMDNSUnRegisterService(0, 0, 0);

    // remove all 64 RX filters (8*8)
    SlWlanRxFilterOperationCommandBuff_t rx_filter_id_mask;
    memset(rx_filter_id_mask.FilterBitmap, 0xFF, 8);
    sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE,
               sizeof(SlWlanRxFilterOperationCommandBuff_t),
               (uint8_t*)&rx_filter_id_mask);

    // set NWP role as STA
    sl_WlanSetMode(ROLE_STA);

    // restart the NWP
    sl_Stop(0xFF);
    sl_Start(0, 0, 0);
}

/*
 * CC32xxWiFi::start()
 */
void CC32xxWiFi::start(WlanRole role, WlanPowerPolicy power_policy, WlanConnectionPolicy connection_policy)
{
    /* We use OSThread::get_priority_max() - 1 for the thread priorities because
     * we want a to be among the highest, but want to reserve one higher
     * priority so that device drivers have the option of using the highest
     * priority for deferred processing.
     */
    wlanRole = role;
    wlanPowerPolicy = power_policy;
    connectionPolicy = connection_policy;

    os_thread_create(nullptr, "sl_Task", OSThread::get_priority_max(), 2048,
                     sl_Task, nullptr);

    os_thread_create(nullptr, "Wlan Task", OSThread::get_priority_max() - 1,
                     2048, wlan_task_entry, nullptr);
}

/*
 * CC32xxWiFi::stop()
 */
void CC32xxWiFi::stop()
{
    ipAcquired = false;
    connected = false;
    started = false;
    sl_Stop(0xFF);
}

/*
 * CC32xxWiFi::wlan_connect()
 */
WlanConnectResult CC32xxWiFi::wlan_connect(const char *ssid,
                                           const char* security_key,
                                           SecurityType security_type)
{
    connected = 0;
    ipAcquired = 0;
    securityFailure = 0;
    this->ssid[0] = '\0';
    if (ipAcquiredCallback_)
    {
        ipAcquiredCallback_(false);
    }

    SlWlanSecParams_t sec_params;
    sec_params.Key = (_i8*)security_key;
    sec_params.KeyLen = strlen(security_key);
    sec_params.Type = security_type_to_simplelink(security_type);

    int result = sl_WlanConnect((signed char*)ssid, strlen(ssid), 0,
                                &sec_params, 0);

    switch (result)
    {
        default:
            SlCheckError(result);
            return WlanConnectResult::CONNECT_OK;
        case SL_ERROR_WLAN_PASSWORD_ERROR:
            return WlanConnectResult::PASSWORD_INVALID;
    }
}

/*
 * CC32xxWiFi::wlan_disconnect()
 */
void CC32xxWiFi::wlan_disconnect()
{
    connected = 0;
    ipAcquired = 0;
    securityFailure = 0;
    ssid[0] = '\0';
    if (ipAcquiredCallback_)
    {
        ipAcquiredCallback_(false);
    }
    sl_WlanDisconnect();
}

/*
 * CC32xxWiFi::wlan_wps_pbc_initiate()
 */
void CC32xxWiFi::wlan_wps_pbc_initiate()
{
    connected = 0;
    ipAcquired = 0;
    securityFailure = 0;
    ssid[0] = '\0';
    if (ipAcquiredCallback_)
    {
        ipAcquiredCallback_(false);
    }

    SlWlanSecParams_t sec_params;
    sec_params.Key = (signed char*)"";
    sec_params.KeyLen = 0;
    sec_params.Type = SL_WLAN_SEC_TYPE_WPS_PBC;

    int result = sl_WlanConnect((signed char*)"WPS_AP", strlen("WPS_AP"), NULL,
                                &sec_params, NULL);
    HASSERT(result >= 0);
}

/*
 * CC32xxWiFi::wlan_setup_ap()
 */
void CC32xxWiFi::wlan_setup_ap(const char *ssid, const char *security_key,
                               SecurityType security_type)
{
    if (ssid)
    {
        HASSERT(strlen(ssid) <= 32);
        sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_SSID, strlen(ssid),
            (uint8_t *)ssid);
        if (wlanRole == WlanRole::AP)
        {
            str_populate(this->ssid, ssid);
        }
    }

    uint8_t sec_type = security_type_to_simplelink(security_type);
    sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_SECURITY_TYPE, 1,
               (uint8_t*)&sec_type);

    if (sec_type == SL_WLAN_SEC_TYPE_OPEN ||
        security_key == nullptr)
    {
        return;
    }

    HASSERT(strlen(security_key) <= 64);
    sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_PASSWORD,
               strlen(security_key), (uint8_t*)security_key);
}

/*
 * CC32xxWiFi::wlan_get_ap_config()
 */
void CC32xxWiFi::wlan_get_ap_config(string *ssid, SecurityType *security_type)
{
    if (ssid)
    {
        // Reads AP SSID configuration from NWP.
        ssid->clear();
        ssid->resize(33);
        uint16_t len = ssid->size();
        uint16_t config_opt = SL_WLAN_AP_OPT_SSID;
        sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt, &len, (_u8*) &(*ssid)[0]);
        ssid->resize(len);
    }
    if (security_type)
    {
        uint16_t len = sizeof(*security_type);
        uint16_t config_opt = SL_WLAN_AP_OPT_SECURITY_TYPE;
        uint8_t sl_sec_type = 0;
        sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt, &len, &sl_sec_type);
        *security_type = security_type_from_simplelink(sl_sec_type);
    }
}

int CC32xxWiFi::wlan_get_ap_station_count()
{
    if (wlanRole != WlanRole::AP)
    {
        return 0;
    }
    uint8_t num_connected = 0;
    uint16_t len = sizeof(num_connected);
    auto status = sl_NetCfgGet(
        SL_NETCFG_AP_STATIONS_NUM_CONNECTED, NULL, &len, &num_connected);
    if (status)
    {
        return -1;
    }
    return num_connected;
}

void CC32xxWiFi::connecting_update_blinker()
{
    if (!connected)
    {
        resetblink(WIFI_BLINK_NOTASSOCIATED);
    }
    else if (!ipAcquired)
    {
        resetblink(WIFI_BLINK_ASSOC_NOIP);
    }
}

/*
 * CC32xxWiFi::set_default_state()
 */
void CC32xxWiFi::set_default_state()
{
    long result = sl_Start(0, 0, 0);
    if (result == SL_ERROR_ROLE_STA_ERR)
    {
        sl_Stop(0xFF);
        result = sl_Start(0, 0, 0);
    }

    SlCheckError(result);
    if (wlanRole == WlanRole::AP ||
        (wlanRole == WlanRole::UNKNOWN && result == ROLE_AP))
    {
        wlanRole = WlanRole::AP;
        if (result != ROLE_AP)
        {
            sl_WlanSetMode(ROLE_AP);
            sl_Stop(0xFF);
            sl_Start(0, 0, 0);
        }
        // Reads AP SSID configuration from NWP.
        uint16_t len = sizeof(ssid);
        memset(ssid, 0, len);
        uint16_t config_opt = SL_WLAN_AP_OPT_SSID;
        sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt, &len, (_u8 *)ssid);
    }
    else
    {
        wlanRole = WlanRole::STA;
        if (wlan_profile_test_none())
        {
            /* no profiles saved, add the default profile */
            wlan_profile_add(WIFI_SSID,
                             strlen(WIFI_PASS) > 0 ? SEC_WPA2 : SEC_OPEN,
                             WIFI_PASS, 0);
        }
        if (result != ROLE_STA)
        {
            sl_WlanSetMode(ROLE_STA);
            sl_Stop(0xFF);
            sl_Start(0, 0, 0);
        }

        wlan_connection_policy_set(connectionPolicy);

        if (wlanPowerPolicy != WLAN_NO_CHANGE_POLICY)
        {
            wlan_power_policy_set(wlanPowerPolicy);
        }
    }
    started = true;
}

/*
 * CC32xxWiFi::wlan_set_role()
 */
void CC32xxWiFi::wlan_set_role(WlanRole new_role)
{
    switch (new_role)
    {
        case WlanRole::STA:
            sl_WlanSetMode(ROLE_STA);
            break;
        case WlanRole::AP:
            sl_WlanSetMode(ROLE_AP);
            break;
        default:
            DIE("Unsupported wlan role");
    }
}

/*
 * CC32xxWiFi::wlan_task()
 */
void CC32xxWiFi::wlan_task()
{
    int result;
    set_default_state();

    /* adjust to a lower priority task */
    vTaskPrioritySet(NULL, configMAX_PRIORITIES / 2);

    SlSockAddrIn_t address;

    wakeup = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, 0);
    HASSERT(wakeup >= 0);

    address.sin_family = SL_AF_INET;
    address.sin_port = sl_Htons(8000);
    address.sin_addr.s_addr = SL_INADDR_ANY;
    result = sl_Bind(wakeup, (SlSockAddr_t*)&address, sizeof(address));
    HASSERT(result >= 0);

    portENTER_CRITICAL();
    SL_SOCKET_FD_SET(wakeup, &rfds);
    add_socket(wakeup);
    portEXIT_CRITICAL();

    unsigned next_wrssi_poll = (os_get_time_monotonic() >> 20) + 800;

    for ( ; /* forever */ ; )
    {

        std::vector<std::function<void()> > callbacks_to_run;
        {
            OSMutexLock l(&lock_);
            if (callbacks_.size()) {
                callbacks_to_run.swap(callbacks_);
            }
        }
        for (unsigned i = 0; i < callbacks_to_run.size(); ++i) {
            callbacks_to_run[i]();
        }

        SlFdSet_t rfds_tmp = rfds;
        SlFdSet_t wfds_tmp = wfds;
        SlFdSet_t efds_tmp = efds;
        SlTimeval_t tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        result = sl_Select(fdHighest + 1, &rfds_tmp, &wfds_tmp, &efds_tmp, &tv);

        if (result < 0)
        {
            continue;
        }

        if ((os_get_time_monotonic() >> 20) > next_wrssi_poll)
        {
            next_wrssi_poll = (os_get_time_monotonic() >> 20) + 800;
            /* timeout, get the RSSI value */
            SlWlanGetRxStatResponse_t response;
            if (sl_WlanRxStatGet(&response, 0) == 0)
            {
                if (response.AvarageMgMntRssi) {
                    rssi = response.AvarageMgMntRssi;
                }
            }
        }

        for (int i = 0; i < SL_MAX_SOCKETS && result > 0; ++i)
        {
            if (slSockets[i] == -1)
            {
                /* socket slot not in use */
                continue;
            }
            if (SL_SOCKET_FD_ISSET(slSockets[i], &wfds_tmp))
            {
                --result;
                portENTER_CRITICAL();
                SL_SOCKET_FD_CLR(slSockets[i], &wfds);
                new_highest();
                CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(slSockets[i]);
                s->writeActive = true;
                s->select_wakeup(&s->selInfoWr);
                portEXIT_CRITICAL();
            }
            if (SL_SOCKET_FD_ISSET(slSockets[i], &rfds_tmp))
            {
                --result;
                if (slSockets[i] == wakeup)
                {
                    /* this is the socket we use as a signal */
                    char data;
                    sl_Recv(wakeup, &data, 1, 0);
                }
                else
                {
                    /* standard application level socket */
                    portENTER_CRITICAL();
                    SL_SOCKET_FD_CLR(slSockets[i], &rfds);
                    new_highest();
                    CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(slSockets[i]);
                    s->readActive = true;
                    s->select_wakeup(&s->selInfoRd);
                    portEXIT_CRITICAL();
                }
            }
            if (SL_SOCKET_FD_ISSET(slSockets[i], &efds_tmp))
            {
                --result;
                portENTER_CRITICAL();
                SL_SOCKET_FD_CLR(slSockets[i], &efds);
                new_highest();
                portEXIT_CRITICAL();
                /* currently we don't handle any errors */
            }
        }
    }
}

/*
 * CC32xxWiFi::select_wakeup()
 */
void CC32xxWiFi::select_wakeup()
{
    if (wakeup >= 0)
    {
        SlSockAddrIn_t address;
        SlSocklen_t length = sizeof(SlSockAddr_t);
        address.sin_family = SL_AF_INET;
        address.sin_port = sl_Htons(8000);
        address.sin_addr.s_addr = sl_Htonl(SL_IPV4_VAL(127,0,0,1));

        /* note that loopback messages only work as long as the we are
         * connected to an AP, or acting as an AP ourselves.  If neither of
         * these is the case, the wakeup of sl_Select() using this method is
         * not necessary.  The sl_Select() API has a timeout to maintain state
         * periodically.
         */
        char data = -1;
        ssize_t result = sl_SendTo(wakeup, &data, 1, 0, (SlSockAddr_t*)&address,
                                   length);
        while (result != 1 && connected)
        {
            usleep(MSEC_TO_USEC(50));
            result = sl_SendTo(wakeup, &data, 1, 0, (SlSockAddr_t*)&address,
                               length);
        }
    }
}

void CC32xxWiFi::run_on_network_thread(std::function<void()> callback)
{
    {
        OSMutexLock l(&lock_);
        callbacks_.emplace_back(std::move(callback));
    }
    select_wakeup();
}

/*
 * CC32xxWiFi::fd_remove()
 */
void CC32xxWiFi::fd_remove(int16_t sd)
{
    portENTER_CRITICAL();
    SL_SOCKET_FD_CLR(sd, &rfds);
    SL_SOCKET_FD_CLR(sd, &wfds);
    SL_SOCKET_FD_CLR(sd, &efds);
    del_socket(sd);
    portEXIT_CRITICAL();
}

/*
 * CC32xxWiFi::fd_set_read()
 */
void CC32xxWiFi::fd_set_read(int16_t sd)
{
    portENTER_CRITICAL();
    if (SL_SOCKET_FD_ISSET(sd, &rfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_SOCKET_FD_SET(sd, &rfds);
    add_socket(sd);
    portEXIT_CRITICAL();
    select_wakeup();
}

/*
 * CC32xxWiFi::fd_set_write()
 */
void CC32xxWiFi::fd_set_write(int16_t sd)
{
    portENTER_CRITICAL();
    if (SL_SOCKET_FD_ISSET(sd, &wfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_SOCKET_FD_SET(sd, &wfds);
    add_socket(sd);
    portEXIT_CRITICAL();
    select_wakeup();
}

/*
 * CC32xxWiFi::wlan_event_handler()
 */
void CC32xxWiFi::wlan_event_handler(WlanEvent *event)
{
    if (event == nullptr)
    {
        return;
    }

    switch (event->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
        {
            connected = 1;
            connectionFailed = 0;
            securityFailure = 0;

            {
                /* Station mode */
                auto connect = &event->Data.Connect;
                portENTER_CRITICAL();
                memcpy(ssid, connect->SsidName, connect->SsidLen);
                ssid[connect->SsidLen] = '\0';
                portEXIT_CRITICAL();
            }
        
            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            // slWlanConnectAsyncResponse_t *event_data = NULL;
            // event_data = &event->EventData.STAandP2PModeWlanConnected;
            //
            break;
        }
        case SL_WLAN_EVENT_DISCONNECT:
        {
            const auto *disconnect = &event->Data.Disconnect;

            connected = 0;
            ipAcquired = 0;
            connectionFailed = 1;
            ssid[0] = '\0';
            if (SL_WLAN_DISCONNECT_SECURITY_FAILURE == disconnect->ReasonCode)
            {
                securityFailure = 1;
            }
            else
            {
                securityFailure = 0;
            }
            if (ipAcquiredCallback_)
            {
                ipAcquiredCallback_(false);
            }
            break;
        }
        case SL_WLAN_EVENT_STA_ADDED:
            // when device is in AP mode and any client connects to device cc3xxx
            break;
        case SL_WLAN_EVENT_STA_REMOVED:
            // when client disconnects from device (AP)
            break;
        case SL_WLAN_EVENT_PROVISIONING_STATUS:
            LOG(INFO, "provisioning status %u %u %u", event->Data.ProvisioningStatus.ProvisioningStatus, event->Data.ProvisioningStatus.Role, event->Data.ProvisioningStatus.WlanStatus);
            // when the auto provisioning kicks in
            break;
        case SL_WLAN_EVENT_PROVISIONING_PROFILE_ADDED:
            LOG(INFO, "provisioning profile added %s %s", event->Data.ProvisioningProfileAdded.Ssid, event->Data.ProvisioningProfileAdded.Reserved);
            // when the auto provisioning created a profile
            break;
        default:
            HASSERT(0);
            break;
    }
}

/*
 * CC32xxWiFi::net_app_event_handler()
 */
void CC32xxWiFi::net_app_event_handler(NetAppEvent *event)
{
    if (event == nullptr)
    {
        return;
    }

    switch (event->Id)
    {
        case SL_NETAPP_EVENT_DHCP_IPV4_ACQUIRE_TIMEOUT:
            /* DHCP acquisition still continues, this is just a warning */
            break;
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            const auto* ip_aquired = &event->Data.IpAcquiredV4;
            ipAddress = ip_aquired->Ip;
        }
        // fall through
        case SL_NETAPP_EVENT_IPV6_ACQUIRED:
            ipAcquired = 1;

            //
            // Information about the IPv4 & IPv6 details (like IP, gateway,dns
            // etc) will be available in 'SlIpV4AcquiredAsync_t / 
            // SlIpV6AcquiredAsync_t' - Applications can use it if required
            //
            // For IPv4:
            //
            // SlIpV4AcquiredAsync_t *event_data = NULL;
            // event_data = &event->EventData.ipAcquiredV4;
            //
            // For IPv6:
            //
            // SlIpV6AcquiredAsync_t *event_data = NULL;
            // event_data = &event->EventData.ipAcquiredV6;
            //
            if (ipAcquiredCallback_)
            {
                ipAcquiredCallback_(true);
            }
            break;
        case SL_NETAPP_EVENT_DHCPV4_LEASED:
        {
            ipLeased = 1;

            //
            // Information about the IP-Leased details(like IP-Leased,lease-time,
            // mac etc) will be available in 'SlIpLeasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpLeasedAsync_t *event_data = NULL;
            // event_data = &event->EventData.ipLeased;
            //

            break;
        }
        case SL_NETAPP_EVENT_IP_COLLISION:
            break;
        case SL_NETAPP_EVENT_IPV4_LOST:
        {
            ipAddress = 0;
            if (ipAcquiredCallback_)
            {
                ipAcquiredCallback_(false);
            }
            break;
        }
        case SL_NETAPP_EVENT_DHCPV4_RELEASED:
            ipLeased = 0;

            //
            // Information about the IP-Released details (like IP-address, mac 
            // etc) will be available in 'SlIpReleasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpReleasedAsync_t *event_data = NULL;
            // event_data = &event->EventData.ipReleased;
            //

            break;
        default:
            HASSERT(0);
            break;
    }
}

/*
 * CC32xxWiFi::sock_event_handler()
 */
void CC32xxWiFi::sock_event_handler(SockEvent *event)
{
    if (event == nullptr)
    {
        return;
    }

    //
    // Events are not expected
    //
    switch (event->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            LOG(ALWAYS, "Socket tx fail status %d, sd %u",
                (int)event->SocketAsyncEvent.SockTxFailData.Status,
                (unsigned)event->SocketAsyncEvent.SockTxFailData.Sd);
            switch (event->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE: 
                    break;
                default: 
                    break;
            }
            break;
        case SL_SOCKET_ASYNC_EVENT:
            LOG(ALWAYS, "Socket async event %d, sd %u type %u val %d",
                (int)event->SocketAsyncEvent.SockAsyncData.Type,
                (unsigned)event->SocketAsyncEvent.SockAsyncData.Sd,
                (unsigned)event->SocketAsyncEvent.SockAsyncData.Type,
                (int)event->SocketAsyncEvent.SockAsyncData.Val);
            switch (event->SocketAsyncEvent.SockAsyncData.Type)
            {
                default:
                    break;
            }
            break;
        default:
            LOG(ALWAYS, "Socket event %d", (int)event->Event);
            break;
    }
}

/*
 * CC32xxWiFi::trigger_event_handler()
 */
void CC32xxWiFi::trigger_event_handler(SockTriggerEvent *event)
{
    if (!event)
    {
        return;
    }

    LOG(INFO, "Socket trigger event %u %d", (unsigned)event->Event,
        (unsigned)event->EventData);
}

/*
 * CC32xxWiFi::http_server_callback()
 */
void CC32xxWiFi::http_server_callback(HttpServerEvent *event,
                                      HttpServerResponse *response)
{
    if(!event || !response)
    {
        return;
    }

    switch (event->Event)
    {
        case SL_NETAPP_EVENT_HTTP_TOKEN_GET:
        {
            unsigned char *ptr;
            ptr = response->ResponseData.TokenValue.pData;
            response->ResponseData.TokenValue.Len = 0;
            string token((const char *)event->EventData.HttpTokenName.pData,
                event->EventData.HttpTokenName.Len);
            LOG(VERBOSE, "token get %s", token.c_str());
            {
                OSMutexLock l(&lock_);
                for (unsigned i = 0; i < httpGetTokenCallbacks_.size(); ++i)
                {
                    if (token == httpGetTokenCallbacks_[i].first)
                    {
                        string result = httpGetTokenCallbacks_[i].second();
                        // clip string if required
                        if (result.size() >= SL_NETAPP_MAX_TOKEN_VALUE_LEN)
                        {
                            result.resize(SL_NETAPP_MAX_TOKEN_VALUE_LEN);
                        }
                        memcpy(ptr, result.data(), result.size());
                        ptr += result.size();
                        response->ResponseData.TokenValue.Len += result.size();
                        break;
                    }
                }
            }
            break;
        }
        case SL_NETAPP_EVENT_HTTP_TOKEN_POST:
        {
            string token(
                (const char *)event->EventData.HttpPostData.TokenName.pData,
                event->EventData.HttpPostData.TokenName.Len);
            string val(
                (const char *)event->EventData.HttpPostData.TokenValue.pData,
                event->EventData.HttpPostData.TokenValue.Len);
            LOG(VERBOSE, "token post %s=%s", token.c_str(), val.c_str());
            break;
        }
        default:
            break;
    }
}

/*
 * CC32xxWiFi::netapp_request_callback()
 */
void CC32xxWiFi::netapp_request_callback(
    NetAppRequest *request, NetAppResponse *response)
{
    if (!request || !response || request->AppId != SL_NETAPP_HTTP_SERVER_ID)
    {
        return;
    }

    string uri;
    uint32_t content_length = 0xFFFFFFFFu;

    uint8_t *meta_curr = request->requestData.pMetadata;
    uint8_t *meta_end = meta_curr + request->requestData.MetadataLen;
    while (meta_curr < meta_end)
    {
        uint8_t meta_type = *meta_curr; /* meta_type is one byte */
        meta_curr++;
        uint16_t meta_len = *(_u16 *)meta_curr; /* Length is two bytes */
        meta_curr += 2;
        switch (meta_type)
        {
            case SL_NETAPP_REQUEST_METADATA_TYPE_HTTP_CONTENT_LEN:
                memcpy(&content_length, meta_curr, meta_len);
                break;
            case SL_NETAPP_REQUEST_METADATA_TYPE_HTTP_REQUEST_URI:
                uri.assign((const char *)meta_curr, meta_len);
                break;
        }
        meta_curr += meta_len;
    }
    // Do we have more data to come?
    bool has_more = request->requestData.Flags &
        SL_NETAPP_REQUEST_RESPONSE_FLAGS_CONTINUATION;
    bool found = false;
    switch (request->Type)
    {
        case SL_NETAPP_REQUEST_HTTP_POST:
        {
            OSMutexLock l(&lock_);
            for (unsigned i = 0; i < httpPostCallbacks_.size(); ++i)
            {
                if (uri == httpPostCallbacks_[i].first)
                {
                    found = true;
                    httpPostCallbacks_[i].second(request->Handle,
                        content_length, request->requestData.pMetadata,
                        request->requestData.MetadataLen,
                        request->requestData.pPayload,
                        request->requestData.PayloadLen, has_more);
                    break;
                }
            }
            if (found)
            {
                break;
            }
        }
            // Fall through to 404.
        default:
            response->Status = SL_NETAPP_HTTP_RESPONSE_404_NOT_FOUND;
            response->ResponseData.pMetadata = NULL;
            response->ResponseData.MetadataLen = 0;
            response->ResponseData.pPayload = NULL;
            response->ResponseData.PayloadLen = 0;
            response->ResponseData.Flags = 0;
            return;
    }
}

/*
 * CC32xxWiFi::get_post_data()
 */
bool CC32xxWiFi::get_post_data(uint16_t handle, void *buf, size_t *len)
{
    uint16_t ulen = *len;
    uint32_t flags;
    int ret = sl_NetAppRecv(handle, &ulen, (uint8_t *)buf, &flags);
    if (ret < 0 || (flags & SL_NETAPP_REQUEST_RESPONSE_FLAGS_ERROR))
    {
        *len = 0;
        return false;
    }
    *len = ulen;
    return (flags & SL_NETAPP_REQUEST_RESPONSE_FLAGS_CONTINUATION) != 0;
}

/*
 * CC32xxWiFi::send_post_response()
 */
void CC32xxWiFi::send_post_respose(
    uint16_t handle, uint16_t http_status, const string &redirect)
{
    // Pieces together a valid metadata structure for SimpleLink.
    string md;
    uint16_t len;
    if (!redirect.empty())
    {
        http_status = SL_NETAPP_HTTP_RESPONSE_302_MOVED_TEMPORARILY;
    }
    md.push_back(SL_NETAPP_REQUEST_METADATA_TYPE_STATUS);
    len = 2;
    md.append((const char *)&len, 2);
    md.append((const char *)&http_status, 2);
    if (!redirect.empty())
    {
        md.push_back(SL_NETAPP_REQUEST_METADATA_TYPE_HTTP_LOCATION);
        len = redirect.size();
        md.append((const char *)&len, 2);
        md += redirect;
    }
    // Now we need to move the metadata content to a buffer that is
    // malloc'ed. This buffer will be freed asynchronously through a callback.
    void *buf = malloc(md.size());
    memcpy(buf, md.data(), md.size());

    uint32_t flags = SL_NETAPP_REQUEST_RESPONSE_FLAGS_METADATA;
    sl_NetAppSend(handle, md.size(), (uint8_t *)buf, flags);
}

/*
 * CC32xxWiFi::fatal_error_callback()
 */
void CC32xxWiFi::fatal_error_callback(CC32xxWiFi::FatalErrorEvent* event)
{
    LOG(WARNING, "Simplelink experienced a fatal error.");
}


static void append_num(std::string* s, uint32_t d) {
    char num[10];
    integer_to_buffer(d, num);
    s->append(num);
}


template<class NUM>
static void append_ver_4(std::string* s, NUM d[4]) {
    for (int i = 0; i < 4; ++i) {
        if (i) s->push_back('.');
        append_num(s, d[i]);
    }
}

std::string CC32xxWiFi::get_version() {
    std::string v;
    SlDeviceVersion_t ver;
    uint8_t config_opt = SL_DEVICE_GENERAL_VERSION;
    uint16_t config_len = sizeof(ver);
    
    int lRetVal = sl_DeviceGet(SL_DEVICE_GENERAL, &config_opt, 
                               &config_len, (unsigned char *)(&ver));
    HASSERT(lRetVal == 0);

    v.push_back('H');
    v += SL_DRIVER_VERSION;
    v += " N";
    append_ver_4(&v, ver.NwpVersion);
    v += " F";
    append_ver_4(&v, ver.FwVersion);
    v += " P";
    uint32_t pv[4];
    pv[0] = ver.PhyVersion[0];
    pv[1] = ver.PhyVersion[1];
    pv[2] = ver.PhyVersion[2];
    pv[3] = ver.PhyVersion[3];
    append_ver_4(&v, pv);
    v += " R";
    append_num(&v, ver.RomVersion);
    v += " C";
    append_num(&v, ver.ChipId);
    return v;
}

// Override weak MDNS implementations for the CC32xx platform

/*
 * mdns_publish()
 */
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    string full_name(name);
    full_name.push_back('.');
    full_name.append(service);
    full_name.append(".local");
    sl_NetAppMDNSRegisterService((const signed char *)full_name.c_str(),
        full_name.size(), (const signed char *)"OLCB", strlen("OLCB"), port,
        200, SL_NETAPP_MDNS_OPTIONS_IS_NOT_PERSISTENT);
}

/*
 * mdns_unpublish()
 */
void mdns_unpublish(const char *name, const char *service)
{
    string full_name(name);
    full_name.push_back('.');
    full_name.append(service);
    full_name.append(".local");
    sl_NetAppMDNSUnRegisterService((const signed char *)full_name.c_str(),
        full_name.size(), SL_NETAPP_MDNS_OPTIONS_IS_NOT_PERSISTENT);
}

extern "C"
{
/** This function handles WLAN events.
 * @param pSlWlanEvent pointer to WLAN Event Info
 */
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    CC32xxWiFi::instance()->wlan_event_handler(
        static_cast<CC32xxWiFi::WlanEvent *>(pSlWlanEvent));
}

/** This function handles network events such as IP acquisition, IP leased,
 * IP released etc.
 * @param pNetAppEvent pointer indicating device acquired IP
 */
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    CC32xxWiFi::instance()->net_app_event_handler(
        static_cast<CC32xxWiFi::NetAppEvent *>(pNetAppEvent));
}

/** This function handles general events.
 * @param pDevEvent pointer to General Event Info 
 */
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* currently unused */
}

/** This function handles socket events indication.
 * @param pSock pointer to Socket Event Info
 */
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    CC32xxWiFi::instance()->sock_event_handler(
        static_cast<CC32xxWiFi::SockEvent *>(pSock));
}

/** This function gets triggered when HTTP Server receives Application
 * defined GET and POST HTTP Tokens.
 * @param pSlHttpServerEvent pointer indicating http server event
 * @param pSlHttpServerResponse pointer indicating http server response
 */
void SimpleLinkHttpServerEventHandler(
                             SlNetAppHttpServerEvent_t *pHttpServerEvent, 
                             SlNetAppHttpServerResponse_t *pHttpServerResponse)
{
    CC32xxWiFi::instance()->http_server_callback(
        static_cast<CC32xxWiFi::HttpServerEvent *>(pHttpServerEvent),
        static_cast<CC32xxWiFi::HttpServerResponse *>(pHttpServerResponse));
}

/**
 *  \brief      This function handles resource request
 *  \param[in]  pNetAppRequest - Contains the resource requests
 *  \param[in]  pNetAppResponse - Should be filled by the user with the
 *                                relevant response information
 *  \return     None
 */
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t  *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
    LOG(VERBOSE,
        "netapprq app %d type %d hdl %d mdlen %u payloadlen %u flags %u",
        pNetAppRequest->AppId, pNetAppRequest->Type, pNetAppRequest->Handle,
        pNetAppRequest->requestData.MetadataLen,
        pNetAppRequest->requestData.PayloadLen,
        (unsigned)pNetAppRequest->requestData.Flags);
    CC32xxWiFi::instance()->netapp_request_callback(
        static_cast<CC32xxWiFi::NetAppRequest *>(pNetAppRequest),
        static_cast<CC32xxWiFi::NetAppResponse *>(pNetAppResponse));
}

/**
 *  \brief      This function handles resource release
 *  \param[in]  buffer - Contains the resource requests
 *  \param[in]  pNetAppResponse - Should be filled by the user with the
 *                                relevant response information
 *  \return     None
 */
void SimpleLinkNetAppRequestMemFreeEventHandler (uint8_t *buffer)
{
    free(buffer);
}

/** This Function Handles the Fatal errors
 *  @param  slFatalErrorEvent - Contains the fatal error data
 *  @return     None
 */
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    CC32xxWiFi::instance()->fatal_error_callback(
        static_cast<CC32xxWiFi::FatalErrorEvent*>(slFatalErrorEvent));
}

/** Notifies the service about a wifi asynchronous socket event callback. This
 * means that sl_Select needs to be re-run and certain sockets might need
 * wakeup.
 * @param event parameters from the socket. */
void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t *event)
{
    CC32xxWiFi::instance()->trigger_event_handler(
        static_cast<CC32xxWiFi::SockTriggerEvent *>(event));
}

extern int slcb_SetErrno(int Errno);

/** Helper function called by SimpleLink driver to set OS-specific errno value.
    The implementation just forwards the errno value to the newlib_nano errno
    value.
    @param Errno is the new value to be written to errno.
 */
int slcb_SetErrno(int Errno)
{
    errno = Errno;
    return 0;
}

} /* extern "C" */
