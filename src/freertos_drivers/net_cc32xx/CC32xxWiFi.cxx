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

#include "CC32xxWiFi.hxx"
#include "CC32xxSocket.hxx"

#include "freertos_drivers/common/WifiDefs.hxx"
#include "utils/format_utils.hxx"

#include <unistd.h>

// Simplelink includes
#include "osi.h"
#include "simplelink.h"

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

CC32xxWiFi *CC32xxWiFi::instance_ = nullptr;

/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
/*static*/ SlFdSet_t rfds;
/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
/*static*/ SlFdSet_t wfds;
/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
/*static*/ SlFdSet_t efds;

/** the highest file descriptor to select on */
/*static*/ int fdHighest;

/*static*/ int16_t slSockets[SL_MAX_SOCKETS];

/** Find the new highest fd to select on.
 */
void new_highest()
{
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
 * @param socket number to add
 */
void add_socket(int16_t socket)
{
    if (socket > fdHighest)
    {
        fdHighest = socket;
    }
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == socket)
        {
            /* already known */
            return;
        }
    }
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == -1)
        {
            slSockets[i] = socket;
            break;
        }
    }
}

/** Delete an interesting socket.
 * @parqam socket number to delete
 */
void del_socket(int16_t socket)
{
    for (int i = 0; i < SL_MAX_SOCKETS; ++i)
    {
        if (slSockets[i] == socket)
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
    , connected(0)
    , connectionFailed(0)
    , ipAcquired(0)
    , ipLeased(0)
    , smartConfigStart(0)
{
    HASSERT(instance_ == nullptr);
    instance_ = this;
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
        entries[i].ssid.assign((char*)sl_entries[i].ssid, sl_entries[i].ssid_len);
        entries[i].sec_type = security_type_from_simplelink(sl_entries[i].sec_type);
        entries[i].rssi = sl_entries[i].rssi;
    }

    delete [] sl_entries;
    return result;
}

/*
 * CC32xxWiFi::wlan_mac()
 */
void CC32xxWiFi::wlan_mac(uint8_t mac[6])
{
    uint8_t  len = 6;
    sl_NetCfgGet(SL_MAC_ADDRESS_GET, nullptr, &len, mac);
}

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

/*
 * CC32xxWiFi::start()
 */
void CC32xxWiFi::start(WlanRole role)
{
    wlanRole = role;
#if 0
    VStartSimpleLinkSpawnTask(configMAX_PRIORITIES - 1);
#else
    xSimpleLinkSpawnQueue = xQueueCreate(3, sizeof( tSimpleLinkSpawnMsg ) );
    os_thread_create(&xSimpleLinkSpawnTaskHndl, "SimpleLink",
                     configMAX_PRIORITIES - 1, 2048, vSimpleLinkSpawnTask, NULL);
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
 * CC32xxWiFi::wlan_setup_ap()
 */
void CC32xxWiFi::wlan_setup_ap(const char *ssid, const char *security_key,
                               SecurityType security_type)
{
    HASSERT(strlen(ssid) <= 32);
    HASSERT(strlen(security_key) <= 64);

    int sec_type = security_type_to_simplelink(security_type);
    uint16_t len;
    uint16_t config_opt;

    char old_ssid[32];
    len = 32;
    config_opt = WLAN_AP_OPT_SSID;
    sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (uint8_t*)old_ssid);
    // simplify strcmp by null terminating the old_ssid
    if (len != strlen(ssid) || strncmp(old_ssid, ssid, len))
    {
        sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(ssid),
                   (uint8_t*)ssid);
    }

    uint8_t old_sec_type;
    len = 1;
    config_opt = WLAN_AP_OPT_SECURITY_TYPE;
    sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (uint8_t*)&old_sec_type);
    if (old_sec_type != sec_type)
    {
        uint8_t sec_type = sec_type;
        sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1,
                   (uint8_t*)&sec_type);
    }

    if (sec_type == SL_SEC_TYPE_OPEN)
    {
        return;
    }

    char old_password[64];
    len = 64;
    config_opt = WLAN_AP_OPT_PASSWORD;
    sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (uint8_t*)old_password);
    if (len != strlen(security_key) || strncmp(old_password, security_key, len))
    {
        sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_PASSWORD,
                   strlen(security_key), (uint8_t*)security_key);
    }
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
#if 0
    /* make socket non-blocking */
    SlSockNonblocking_t sl_option_value;
    sl_option_value.NonblockingEnabled = 1;
    result = sl_SetSockOpt(wakeup, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                           &sl_option_value, sizeof(sl_option_value));
#endif
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
                    int16_t data;
                    int status = sl_Recv(wakeup, &data, 2, 0);
                    if (status < 2)
                    {
                        continue;
                    }
                    switch (data)
                    {
                        default:
                            /* special action to close socket */
                            HASSERT(data >= 0);
                            portENTER_CRITICAL();
                            SL_FD_CLR(data, &rfds);
                            SL_FD_CLR(data, &wfds);
                            SL_FD_CLR(data, &efds);
                            del_socket(data);
                            delete CC32xxSocket::get_instance_from_sd(data);
                            CC32xxSocket::remove_instance_from_sd(data);
                            portEXIT_CRITICAL();

                            sl_Close(data);
                        case SELECT_WAKE_NO_ACTION:
                            break;
                        case SELECT_WAKE_EXIT:
                            /* shutdown Wi-Fi stack and helper thread */
                            sl_Close(wakeup);
                            return;
                    }
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
void CC32xxWiFi::select_wakeup(int16_t data)
{
    if (wakeup >= 0)
    {
        SlSockAddrIn_t address;
        SlSocklen_t length = sizeof(SlSockAddr_t);
        address.sin_family = SL_AF_INET;
        address.sin_port = sl_Htons(8000);
        address.sin_addr.s_addr = sl_Htonl(SL_IPV4_VAL(127,0,0,1));

        int result = sl_SendTo(wakeup, &data, 2, 0, (SlSockAddr_t*)&address, length);
        while (result == SL_EAGAIN && data != SELECT_WAKE_NO_ACTION);
        {
            result = sl_SendTo(wakeup, &data, 2, 0, (SlSockAddr_t*)&address, length);
            usleep(10000);
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
 * CC32xxWiFi::fd_set_read()
 */
void CC32xxWiFi::fd_set_read(int16_t socket)
{
    portENTER_CRITICAL();
    if (SL_FD_ISSET(socket, &rfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_FD_SET(socket, &rfds);
    add_socket(socket);
    portEXIT_CRITICAL();
    select_wakeup();
}

/*
 * CC32xxWiFi::fd_set_write()
 */
void CC32xxWiFi::fd_set_write(int16_t socket)
{
    portENTER_CRITICAL();
    if (SL_FD_ISSET(socket, &wfds))
    {
        /* already set */
        portEXIT_CRITICAL();
        return;
    }
    SL_FD_SET(socket, &wfds);
    add_socket(socket);
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

    switch (event->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            connected = 1;
            connectionFailed = 0;

            slWlanConnectAsyncResponse_t *event_data;
            event_data = &event->EventData.STAandP2PModeWlanConnected;
            if (event_data->connection_type == 0)
            {
                /* Station mode */
                portENTER_CRITICAL();
                memcpy(ssid, event_data->ssid_name, event_data->ssid_len);
                ssid[event_data->ssid_len] = '\0';
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
        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t *event_data;

            event_data = &event->EventData.STAandP2PModeDisconnected;

            connected = 0;
            ipAcquired = 0;
            ssid[0] = '\0';


            // If the user has initiated 'Disconnect' request, 
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION 
            if(SL_USER_INITIATED_DISCONNECTION == event_data->reason_code)
            {
            }
            else
            {
            }
            break;
        }
        case SL_WLAN_STA_CONNECTED_EVENT:
            // when device is in AP mode and any client connects to device cc3xxx
            break;
        case SL_WLAN_STA_DISCONNECTED_EVENT:
            // when client disconnects from device (AP)
            connected = 0;
            ipLeased = 0;

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *event_data = NULL;
            // event_data = &event->EventData.APModestaDisconnected;
            //
            break;
        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
            smartConfigStart = 1;

            //
            // Information about the SmartConfig details (like Status, SSID, 
            // Token etc) will be available in 'slSmartConfigStartAsyncResponse_t' 
            // - Applications can use it if required
            //
            //  slSmartConfigStartAsyncResponse_t *event_data = NULL;
            //  event_data = &event->EventData.smartConfigStartResponse;
            //
            break;
        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
            // SmartConfig operation finished
            smartConfigStart = 0;

            //
            // Information about the SmartConfig details (like Status, padding 
            // etc) will be available in 'slSmartConfigStopAsyncResponse_t' - 
            // Applications can use it if required
            //
            // slSmartConfigStopAsyncResponse_t *event_data = NULL;
            // event_data = &event->EventData.smartConfigStopResponse;
            //
            break;
        case SL_WLAN_P2P_DEV_FOUND_EVENT:
            HASSERT(0);
            break;
        case SL_WLAN_P2P_NEG_REQ_RECEIVED_EVENT:
            HASSERT(0);
            break;
        case SL_WLAN_CONNECTION_FAILED_EVENT:
            // If device gets any connection failed event
            connectionFailed = 1;
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

    switch (event->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *event_data = NULL;
            event_data = &event->EventData.ipAcquiredV4;
            ipAddress = event_data->ip;
        }
        // fall through
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
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
        case SL_NETAPP_IP_LEASED_EVENT:
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

            SlIpLeasedAsync_t *event_data = &event->EventData.ipLeased;
            ipAddress = event_data->ip_address;
            break;
        }
        case SL_NETAPP_IP_RELEASED_EVENT:
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
            switch (event->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE: 
                    break;
                default: 
                    break;
            }
            break;
        case SL_SOCKET_ASYNC_EVENT:
            switch (event->socketAsyncEvent.SockAsyncData.type)
            {
                case SSL_ACCEPT:/*accept failed due to ssl issue ( tcp pass)*/
                    break;
                case RX_FRAGMENTATION_TOO_BIG:
                    break;
                case OTHER_SIDE_CLOSE_SSL_DATA_NOT_ENCRYPTED:
                    break;
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
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            unsigned char *ptr;

            ptr = response->ResponseData.token_value.data;
            response->ResponseData.token_value.len = 0;

            for (unsigned i = 0; i < httpGetCallbacks_.size(); ++i)
            {
                if (strcmp((const char *)event->EventData.httpTokenName.data,
                           httpGetCallbacks_[i].second) == 0)
                {
                    string result = httpGetCallbacks_[i].first();
                    // clip string if required
                    if (result.size() >= MAX_TOKEN_VALUE_LEN)
                    {
                        result.erase(MAX_TOKEN_VALUE_LEN);
                    }
                    memcpy(ptr, result.c_str(), result.size());
                    ptr += result.size();
                    response->ResponseData.token_value.len += result.size();
                    break;
                }
            }
            break;
        }
        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {
            break;
        }
        default:
            break;
    }
}


static void append_num(std::string* s, uint32_t d) {
    char num[10];
    integer_to_buffer(d, num);
    s->append(num);
}

static void append_ver_4(std::string* s, uint32_t* d) {
    for (int i = 0; i < 4; ++i) {
        if (i) s->push_back('.');
        append_num(s, d[i]);
    }
}

std::string CC32xxWiFi::get_version() {
    std::string v;
    SlVersionFull ver;
    uint8_t ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    uint8_t ucConfigLen = sizeof(ver);
    int lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                            &ucConfigLen, (unsigned char *)(&ver));
    HASSERT(lRetVal == 0);

    v.push_back('H');
    v += SL_DRIVER_VERSION;
    v += " N";
    append_ver_4(&v, ver.NwpVersion);
    v += " F";
    append_ver_4(&v, ver.ChipFwAndPhyVersion.FwVersion);
    v += " P";
    uint32_t pv[4];
    pv[0] = ver.ChipFwAndPhyVersion.PhyVersion[0];
    pv[1] = ver.ChipFwAndPhyVersion.PhyVersion[1];
    pv[2] = ver.ChipFwAndPhyVersion.PhyVersion[2];
    pv[3] = ver.ChipFwAndPhyVersion.PhyVersion[3];
    append_ver_4(&v, pv);
    v += " R";
    append_num(&v, ver.RomVersion);
    v += " C";
    append_num(&v, ver.ChipFwAndPhyVersion.ChipId);
    return v;
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

} /* extern "C" */
