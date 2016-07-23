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

#include <unistd.h>

// Simplelink includes
#include "osi.h"
#include "simplelink.h"

CC32xxWiFi *CC32xxWiFi::instance_ = nullptr;

/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
static SlFdSet_t rfds;
/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
static SlFdSet_t wfds;
/** these are not class members so that including CC32xxWiFi.hxx does not
 * pollute the namespace with simplelink APIs
 */
static SlFdSet_t efds;

/*
 * CC32xxWiFi::CC32xxWiFi()
 */
CC32xxWiFi::CC32xxWiFi()
    : ipAddress(0)
    , wakeup(-1)
    , connected(0)
    , connectionFailed(0)
    , ipAquired(0)
    , ipLeased(0)
    , smartConfigStart(0)
{
    HASSERT(instance_ == nullptr);
    instance_ = this;
    SL_FD_ZERO(&rfds);
    SL_FD_ZERO(&wfds);
    SL_FD_ZERO(&efds);
}

/*
 * CC32xxWiFi::start()
 */
void CC32xxWiFi::start()
{
    VStartSimpleLinkSpawnTask(configMAX_PRIORITIES - 1);
    //int result = sl_start(0, 0, 0);
    //HASSERT(result >= 0);
    osi_TaskCreate(wlan_task_entry, (const signed char*)"Wlan Task", 2048,
                   NULL, configMAX_PRIORITIES - 1, NULL);
}

/*
 * CC32xxWiFi::wlan_connect()
 */
void CC32xxWiFi::wlan_connect(const char *ssid, const char* security_key,
                         uint8_t security_type)
{
    SlSecParams_t sec_params;
    sec_params.Key = (_i8*)security_key;
    sec_params.KeyLen = strlen(security_key);
    sec_params.Type = security_type;

    int result = sl_WlanConnect((signed char*)ssid, strlen(ssid), 0, &sec_params, 0);
    HASSERT(result >= 0);

    while (true)
    {
        if (!connected) {
            resetblink(WIFI_BLINK_NOTASSOCIATED);
        } else if (!ipAquired) {
            resetblink(WIFI_BLINK_ASSOC_NOIP);
        } else {
            resetblink(WIFI_BLINK_CONNECTING);
            break;
        }
        usleep(10000);
    }
}

/*
 * CC32xxWiFi::set_default_state()
 */
void CC32xxWiFi::set_default_state()
{
    long result = sl_Start(0, 0, 0);
    if (result != ROLE_STA)
    {
        sl_WlanSetMode(ROLE_STA);
        sl_Stop(0xFF);
        sl_Start(0, 0, 0);
    }
}

/*
 * CC32xxWiFi::wlan_task()
 */
void CC32xxWiFi::wlan_task()
{
    int result;
    set_default_state();

    //wlan_connect("GoogleGuest", "", SL_SEC_TYPE_OPEN);
    wlan_connect("CC31xxSSID", "testtest", SL_SEC_TYPE_WPA);

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
    portEXIT_CRITICAL();

    for ( ; /* forever */ ; )
    {
        SlFdSet_t rfds_tmp = rfds;
        SlFdSet_t wfds_tmp = wfds;
        SlFdSet_t efds_tmp = efds;
        SlTimeval_t tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        result = sl_Select(0x20, &rfds_tmp, &wfds_tmp, &efds_tmp, &tv);

        if (result < 0)
        {
            continue;
        }

        if (result == 0)
        {
            /* timeout, get the RSSI value */
            SlGetRxStatResponse_t response;
            if (sl_WlanRxStatGet(&response, 0) == 0)
            {
                rssi = response.AvarageMgMntRssi;
            }
            continue;
        }

        for (int i = 0x1F; i >= 0 && result > 0; --i)
        {
            if (SL_FD_ISSET(i, &rfds_tmp))
            {
                --result;
                if ((i & BSD_SOCKET_ID_MASK) == wakeup)
                {
                    /* this is the socket we use as a signal */
                    int16_t data;
                    int status = sl_Recv(wakeup, &data, 2, 0);
                    if (status < 2)
                    {
                        continue;
                    }
                    if (data != -1)
                    {
                        /* special action to close socket */
                        portENTER_CRITICAL();
                        SL_FD_CLR(data, &rfds);
                        SL_FD_CLR(data, &wfds);
                        SL_FD_CLR(data, &efds);
                        delete CC32xxSocket::get_instance_from_sd(data);
                        CC32xxSocket::remove_instance_from_sd(data);
                        portEXIT_CRITICAL();

                        sl_Close(data);
                    }
                }
                else
                {
                    /* standard application level socket */
                    SL_FD_CLR(i, &rfds);
                    CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(i);
                    s->readActive = true;
                    s->select_wakeup(&s->selInfoRd);
                }
            }
            if (SL_FD_ISSET(i, &wfds_tmp))
            {
                --result;
                SL_FD_CLR(i, &wfds);
                CC32xxSocket *s = CC32xxSocket::get_instance_from_sd(i);
                s->writeActive = true;
                s->select_wakeup(&s->selInfoWr);
            }
            if (SL_FD_ISSET(i, &efds_tmp))
            {
                SL_FD_CLR(i, &efds);
                /* currently we don't handle any errors */
                --result;
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

        sl_SendTo(wakeup, &data, 2, 0, (SlSockAddr_t*)&address, length);
    }
}

/*
 * CC32xxWiFi::fd_set_read()
 */
void CC32xxWiFi::fd_set_read(int16_t socket)
{
    SL_FD_SET(socket, &rfds);
    select_wakeup();
}

/*
 * CC32xxWiFi::fd_set_write()
 */
void CC32xxWiFi::fd_set_write(int16_t socket)
{
    SL_FD_SET(socket, &wfds);
    select_wakeup();
}

/*
 * CC32xxWiFi::wlan_event_handler()
 */
void CC32xxWiFi::wlan_event_handler(void *context)
{
    if (context == NULL)
    {
        return;
    }

    SlWlanEvent_t *event = static_cast<SlWlanEvent_t *>(context);

    switch (event->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
            connected = 1;
            connectionFailed = 0;

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            // slWlanConnectAsyncResponse_t *event_data = NULL;
            // event_data = &event->EventData.STAandP2PModeWlanConnected;
            //
            break;
        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t *event_data;

            event_data = &event->EventData.STAandP2PModeDisconnected;

            connected = 0;
            ipAquired = 0;


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
            HASSERT(0);
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
void CC32xxWiFi::net_app_event_handler(void *context)
{
    if (context == NULL)
    {
        return;
    }

    SlNetAppEvent_t *event = static_cast<SlNetAppEvent_t *>(context);

    switch (event->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
            ipAquired = 1;

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
void CC32xxWiFi::sock_event_handler(void *context)
{
    if (context == NULL)
    {
        return;
    }

    SlSockEvent_t *event = static_cast<SlSockEvent_t *>(context);

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

extern "C"
{
/** This function handles WLAN events.
 * @param pSlWlanEvent pointer to WLAN Event Info
 */
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    CC32xxWiFi::instance()->wlan_event_handler(pSlWlanEvent);
}

/** This function handles network events such as IP acquisition, IP leased,
 * IP released etc.
 * @param pNetAppEvent pointer indicating device acquired IP
 */
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    CC32xxWiFi::instance()->net_app_event_handler(pNetAppEvent);
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
    CC32xxWiFi::instance()->sock_event_handler(pSock);
}

/** This function gets triggered when HTTP Server receives Application
 * defined GET and POST HTTP Tokens.
 * @param pSlHttpServerEvent pointer indicating http server event
 * @param pSlHttpServerResponse pointer indicating http server response
 */
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, 
                                  SlHttpServerResponse_t *pSlHttpServerResponse)
{
    /* currently unused */
}
} /* extern "C" */
