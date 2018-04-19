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

#define SUPPORT_SL_R1_API

#include "CC32xxWiFi.hxx"
#include "CC32xxSocket.hxx"

#include "freertos_drivers/common/WifiDefs.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "utils/format_utils.hxx"
#include "utils/logging.h"

#include <unistd.h>

// Simplelink includes
#include "CC3200_compat/simplelink.h"

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::WlanEvent : public ::SlWlanEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::NetAppEvent : public ::SlNetAppEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::SockEvent : public ::SlSockEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::HttpServerEvent : public ::SlHttpServerEvent_t {};

/** CC32xx forward declaration Helper */
struct CC32xxWiFi::HttpServerResponse : public ::SlHttpServerResponse_t {};

#ifdef SL_API_V2
/** CC32xx forward declaration Helper */
struct CC32xxWiFi::FatalErrorEvent : public ::SlDeviceFatal_t {};
#endif

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
                if (SL_FD_ISSET(slSockets[i], &rfds) ||
                    SL_FD_ISSET(slSockets[i], &wfds) ||
                    SL_FD_ISSET(slSockets[i], &efds))
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
    , wakeup(-1)
    , rssi(0)
    , wlanRole(WlanRole::UNKNOWN)
    , started(false)
    , connected(0)
    , connectionFailed(0)
    , ipAcquired(0)
    , ipLeased(0)
{
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        slSockets[i] = -1;
    }
    SL_FD_ZERO(&rfds);
    SL_FD_ZERO(&wfds);
    SL_FD_ZERO(&efds);
    ssid[0] = '\0';

    add_http_get_callback(make_pair(bind(&CC32xxWiFi::http_get_ip_address,
                                         this), "__SL_G_UNA"));
}

uint8_t CC32xxWiFi::security_type_to_simplelink(SecurityType sec_type)
{
    switch (sec_type)
    {
        default:
        case SEC_OPEN:
            return SL_SEC_TYPE_OPEN;
        case SEC_WEP:
            return SL_SEC_TYPE_WEP;
        case SEC_WPA2:
            return SL_SEC_TYPE_WPA_WPA2;
    }
}

CC32xxWiFi::SecurityType CC32xxWiFi::security_type_from_scan(unsigned sec_type)
{
#ifdef SL_API_V2
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
#else
    return security_type_from_simplelink(sec_type);
#endif    
}


CC32xxWiFi::SecurityType CC32xxWiFi::security_type_from_simplelink(uint8_t sec_type)
{
    switch (sec_type)
    {
        default:
        case SL_SEC_TYPE_OPEN:
            return SEC_OPEN;
        case SL_SEC_TYPE_WEP:
            return SEC_WEP;
        case SL_SEC_TYPE_WPS_PBC:
        case SL_SEC_TYPE_WPS_PIN:
        case SL_SEC_TYPE_WPA_ENT:
        case SL_SEC_TYPE_WPA_WPA2:
            return SEC_WPA2;
    }
}

/*
 * CC32xxWiFi::wlan_profile_add()
 */
int CC32xxWiFi::wlan_profile_add(const char *ssid, SecurityType sec_type,
                                 const char *key, unsigned priority)
{
    SlSecParams_t sec_params;
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
    SlSecParams_t sec_params;
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
#ifdef SL_API_V2
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
#else
    return -1;
#endif
}

/*
 * CC32xxWiFi::wlan_power_policy_set()
 */
int CC32xxWiFi::wlan_power_policy_set(WlanPowerPolicy wpp)
{
    int result;
#ifdef SL_API_V2
    WlanPowerPolicy temp_wpp;
    result = wlan_power_policy_get(&temp_wpp);
    if (result != 0)
    {
        return -1;
    }

    if (temp_wpp != wpp)
#endif
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

/*
 * CC32xxWiFi::wlan_rescan()
 */
void CC32xxWiFi::wlan_rescan()
{
    // This will trigger an immediate rescan and then every 10 minutes a new
    // scan will be done if we have no connection.
    unsigned long intervalInSeconds = 600;
    sl_WlanPolicySet(SL_POLICY_SCAN, 1 /*enable*/, (unsigned char *)
                     &intervalInSeconds,sizeof(intervalInSeconds));    
}
    
/*
 * CC32xxWiFi::wlan_network_list_get()
 */
int CC32xxWiFi::wlan_network_list_get(WlanNetworkEntry *entries, size_t count)
{
    Sl_WlanNetworkEntry_t* sl_entries = new Sl_WlanNetworkEntry_t[count];

    int result = sl_WlanGetNetworkList(0, count, sl_entries);

    for (int i = 0; i < result; ++i)
    {
        entries[i].ssid.assign((char*)sl_entries[i].SL_ssid, sl_entries[i].SL_ssid_len);
        entries[i].sec_type = security_type_from_scan(sl_entries[i].SL_sec_type);
        entries[i].rssi = sl_entries[i].SL_rssi;
    }

    delete [] sl_entries;
    return result;
}

/*
 * CC32xxWiFi::wlan_mac()
 */
void CC32xxWiFi::wlan_mac(uint8_t mac[6])
{
#ifdef SL_API_V2
    uint16_t  len = 6;
#else
    uint8_t  len = 6;
#endif
    sl_NetCfgGet(SL_MAC_ADDRESS_GET, nullptr, &len, mac);
}

#ifndef SL_API_V2
void *vSimpleLinkSpawnTask(void *pvParameters)
{
    tSimpleLinkSpawnMsg Msg;
    portBASE_TYPE ret=pdFAIL;

    for(;;)
    {
        ret = xQueueReceive( xSimpleLinkSpawnQueue, &Msg, portMAX_DELAY );
        if(ret == pdPASS)
        {
                Msg.pEntry(Msg.pValue);
        }
    }
    return nullptr;
}
extern TaskHandle_t xSimpleLinkSpawnTaskHndl;
#endif

/*
 * CC32xxWiFi::start()
 */
void CC32xxWiFi::start(WlanRole role, WlanPowerPolicy power_policy)
{
    wlanRole = role;
    wlanPowerPolicy = power_policy;
#ifdef SL_API_V2
    os_thread_create(nullptr, "SimpleLink Task", configMAX_PRIORITIES - 1, 2048,
        sl_Task, nullptr);
#else
#if 0
    VStartSimpleLinkSpawnTask(configMAX_PRIORITIES - 1);
#else
    xSimpleLinkSpawnQueue = xQueueCreate(3, sizeof( tSimpleLinkSpawnMsg ) );
    os_thread_create(&xSimpleLinkSpawnTaskHndl, "SimpleLink",
                     configMAX_PRIORITIES - 1, 2048, vSimpleLinkSpawnTask, NULL);
#endif // if 0
#endif
    os_thread_create(nullptr, "Wlan Task", configMAX_PRIORITIES - 1, 2048,
                     wlan_task_entry, nullptr);
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
void CC32xxWiFi::wlan_connect(const char *ssid, const char* security_key,
                              SecurityType security_type)
{
    SlSecParams_t sec_params;
    sec_params.Key = (_i8*)security_key;
    sec_params.KeyLen = strlen(security_key);
    sec_params.Type = security_type_to_simplelink(security_type);

    int result = sl_WlanConnect((signed char*)ssid, strlen(ssid), 0,
                                &sec_params, 0);
    HASSERT(result >= 0);

    while (!wlan_ready())
    {
        connecting_update_blinker();
        usleep(10000);
    }
}

/*
 * CC32xxWiFi::wlan_wps_pbc_initiate()
 */
void CC32xxWiFi::wlan_wps_pbc_initiate()
{
    SlSecParams_t sec_params;
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
    HASSERT(strlen(ssid) <= 32);
    HASSERT(strlen(security_key) <= 64);

    uint8_t sec_type = security_type_to_simplelink(security_type);

    sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(ssid),
               (uint8_t*)ssid);

    sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1,
               (uint8_t*)&sec_type);

    if (sec_type == SL_SEC_TYPE_OPEN)
    {
        return;
    }

    sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_PASSWORD,
               strlen(security_key), (uint8_t*)security_key);
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
    SlCheckError(result);
    if (wlanRole == WlanRole::AP)
    {
        if (result != ROLE_AP)
        {
            sl_WlanSetMode(ROLE_AP);
            sl_Stop(0xFF);
            sl_Start(0, 0, 0);
        }
    }
    else
    {
        if (true || wlan_profile_test_none())
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

        /* auto connection policy */
        sl_WlanPolicySet(SL_POLICY_CONNECTION,SL_CONNECTION_POLICY(1,0,0,0,0),
                         NULL,0);

        if (wlanPowerPolicy != WLAN_NO_CHANGE_POLICY)
        {
            wlan_power_policy_set(wlanPowerPolicy);
        }
    }
    started = true;
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
    SL_FD_SET(wakeup, &rfds);
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

        if (result == 0 || (os_get_time_monotonic() >> 20) > next_wrssi_poll)
        {
            next_wrssi_poll = (os_get_time_monotonic() >> 20) + 800;
            /* timeout, get the RSSI value */
            SlGetRxStatResponse_t response;
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
            if (SL_FD_ISSET(slSockets[i], &rfds_tmp))
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
                    SL_FD_CLR(slSockets[i], &rfds);
                    new_highest();
                    CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(slSockets[i]);
                    portEXIT_CRITICAL();
                    s->readActive = true;
                    s->select_wakeup(&s->selInfoRd);
                }
            }
            if (SL_FD_ISSET(slSockets[i], &wfds_tmp))
            {
                --result;
                portENTER_CRITICAL();
                SL_FD_CLR(slSockets[i], &wfds);
                new_highest();
                CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(slSockets[i]);
                portEXIT_CRITICAL();
                s->writeActive = true;
                s->select_wakeup(&s->selInfoWr);
            }
            if (SL_FD_ISSET(slSockets[i], &efds_tmp))
            {
                --result;
                portENTER_CRITICAL();
                SL_FD_CLR(slSockets[i], &efds);
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
        sl_SendTo(wakeup, &data, 1, 0, (SlSockAddr_t*)&address, length);
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
    SL_FD_CLR(sd, &rfds);
    SL_FD_CLR(sd, &wfds);
    SL_FD_CLR(sd, &efds);
    del_socket(sd);
    portEXIT_CRITICAL();
}

/*
 * CC32xxWiFi::fd_set_read()
 */
void CC32xxWiFi::fd_set_read(int16_t sd)
{
    portENTER_CRITICAL();
    if (SL_FD_ISSET(sd, &rfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_FD_SET(sd, &rfds);
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
    if (SL_FD_ISSET(sd, &wfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_FD_SET(sd, &wfds);
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

    switch (event->SL_Event)
    {
        case SL_WLAN_EVENT_CONNECT:
        {
            connected = 1;
            connectionFailed = 0;

            const auto *event_data =
                &event->SL_EventData.STAandP2PModeWlanConnected;
#ifndef SL_API_V2
            if (event_data->connection_type != 0)
            {
                break;
            }
#endif
            {
                /* Station mode */
                portENTER_CRITICAL();
                memcpy(ssid, event_data->SL_ssid_name, event_data->SL_ssid_len);
                ssid[event_data->SL_ssid_len] = '\0';
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
            const auto *event_data =
                &event->SL_EventData.STAandP2PModeDisconnected;

            connected = 0;
            ipAcquired = 0;
            ssid[0] = '\0';


            // If the user has initiated 'Disconnect' request, 
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION 
            if(SL_USER_INITIATED_DISCONNECTION == event_data->SL_reason_code)
            {
            }
            else
            {
            }
            break;
        }
        case SL_WLAN_EVENT_STA_ADDED:
            // when device is in AP mode and any client connects to device cc3xxx
            break;
        case SL_WLAN_EVENT_STA_REMOVED:
            // when client disconnects from device (AP)
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

    switch (event->SL_Event)
    {
#if defined (SL_API_V2)
        case SL_NETAPP_EVENT_DHCP_IPV4_ACQUIRE_TIMEOUT:
            /* DHCP acquisition still continues, this is just a warning */
            break;
#endif
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            //SlIpV4AcquiredAsync_t *event_data = NULL;
            const auto* event_data = &event->SL_EventData.SL_ipAcquiredV4;
            ipAddress = event_data->SL_ip;
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

            SlIpLeasedAsync_t *event_data = &event->SL_EventData.SL_ipLeased;
            ipAddress = event_data->SL_ip_address;
            break;
        }
#if defined (SL_API_V2)
        case SL_NETAPP_EVENT_IP_COLLISION:
            break;
        case SL_NETAPP_EVENT_IPV4_LOST:
        {
            ipAddress = 0;
            break;
        }
#endif
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
            switch (event->SL_socketAsyncEvent.SockTxFailData.SL_status)
            {
                case SL_ECLOSE: 
                    break;
                default: 
                    break;
            }
            break;
        case SL_SOCKET_ASYNC_EVENT:
            switch (event->SL_socketAsyncEvent.SockAsyncData.SL_type)
            {
                default:
                    break;
            }
            break;
        default:
            break;
    }
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

            ptr = response->ResponseData.SL_token_value.SL_data;
            response->ResponseData.SL_token_value.SL_len = 0;

            for (unsigned i = 0; i < httpGetCallbacks_.size(); ++i)
            {
                if (strcmp((const char *)event->EventData.SL_httpTokenName.SL_data,
                           httpGetCallbacks_[i].second) == 0)
                {
                    string result = httpGetCallbacks_[i].first();
                    // clip string if required
                    if (result.size() >= SL_NETAPP_MAX_TOKEN_VALUE_LEN)
                    {
                        result.erase(SL_NETAPP_MAX_TOKEN_VALUE_LEN);
                    }
                    memcpy(ptr, result.c_str(), result.size());
                    ptr += result.size();
                    response->ResponseData.SL_token_value.SL_len += result.size();
                    break;
                }
            }
            break;
        }
        case SL_NETAPP_EVENT_HTTP_TOKEN_POST:
        {
            break;
        }
        default:
            break;
    }
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
    SlVersionFull ver;
    uint8_t ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
#ifdef SL_API_V2
    uint16_t ucConfigLen = sizeof(ver);
#else
    uint8_t ucConfigLen = sizeof(ver);
#endif
    
    int lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                            &ucConfigLen, (unsigned char *)(&ver));
    HASSERT(lRetVal == 0);

    v.push_back('H');
    v += SL_DRIVER_VERSION;
    v += " N";
    append_ver_4(&v, ver.NwpVersion);
    v += " F";
    append_ver_4(&v, SL_ChipFwAndPhyVersion(ver).FwVersion);
    v += " P";
    uint32_t pv[4];
    pv[0] = SL_ChipFwAndPhyVersion(ver).PhyVersion[0];
    pv[1] = SL_ChipFwAndPhyVersion(ver).PhyVersion[1];
    pv[2] = SL_ChipFwAndPhyVersion(ver).PhyVersion[2];
    pv[3] = SL_ChipFwAndPhyVersion(ver).PhyVersion[3];
    append_ver_4(&v, pv);
    v += " R";
    append_num(&v, ver.RomVersion);
    v += " C";
    append_num(&v, SL_ChipFwAndPhyVersion(ver).ChipId);
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
        sl_NetAppMDNSRegisterService((const signed char*)full_name.c_str(),
                                     full_name.size(),
                                     (const signed char*)"OLCB", strlen("OLCB"), 
                                     port, 200, 0);
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
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpServerEvent, 
                                  SlHttpServerResponse_t *pHttpServerResponse)
{
    CC32xxWiFi::instance()->http_server_callback(
        static_cast<CC32xxWiFi::HttpServerEvent *>(pHttpServerEvent),
        static_cast<CC32xxWiFi::HttpServerResponse *>(pHttpServerResponse));
}

#ifdef SL_API_V2

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
    /* Unused in this application */
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

#endif 

} /* extern "C" */
