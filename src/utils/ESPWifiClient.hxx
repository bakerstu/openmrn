/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file ESPWifiClient.hxx
 *
 * Uses the ESPConn API to connect to a wifi accesspoint, and setup a TCP
 * connection to a gridconnect server.
 *
 * @author Balazs Racz
 * @date 10 Mar 2015
 */

#ifndef _UTILS_ESPWIFICLIENT_HXX_
#define _UTILS_ESPWIFICLIENT_HXX_

extern "C" {
#include "ip_addr.h"
#include "espconn.h"
#include "user_interface.h"
#include "ets_sys.h"
#define os_printf printf
}

#include "utils/Singleton.hxx"

class ESPWifiClient : public Singleton<ESPWifiClient>, private HubPort
{
public:
    ESPWifiClient(const string &ssid, const string &password,
        CanHubFlow *hub, const string &hostname, int port)
        : HubPort(hub->service())
        , host_(hostname)
        , port_(port)
        , hub_(hub)
        , gcHub_(hub_->service())
    {

        wifi_set_opmode_current(STATION_MODE);
        memcpy(&stationConfig_.ssid, ssid.c_str(), ssid.size() + 1);
        memcpy(&stationConfig_.password, password.c_str(), password.size() + 1);
        wifi_station_set_config(&stationConfig_);
        wifi_set_event_handler_cb(&static_wifi_callback);
        wifi_station_connect(); // may be avoided if the constructor is still called in
                        // the user_init.
    }

    static void static_wifi_callback(System_Event_t *evt)
    {
        os_printf("%s: %d\n", __FUNCTION__, evt->event);

        switch (evt->event)
        {
            case EVENT_STAMODE_CONNECTED:
            {
                os_printf("connect to ssid %s, channel %d\n",
                    evt->event_info.connected.ssid,
                    evt->event_info.connected.channel);
                break;
            }

            case EVENT_STAMODE_DISCONNECTED:
            {
                os_printf("disconnect from ssid %s, reason %d\n",
                    evt->event_info.disconnected.ssid,
                    evt->event_info.disconnected.reason);

                system_deep_sleep_set_option(0);
                system_deep_sleep(60 * 1000 * 1000); // 60 seconds
                break;
            }

            case EVENT_STAMODE_GOT_IP:
            {
                /*
                os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                    IP2STR(evt->event_info.got_ip.ip),
                    IP2STR(evt->event_info.got_ip.mask),
                    IP2STR(evt->event_info.got_ip.gw));
                    os_printf("\n");*/

                Singleton<ESPWifiClient>::instance()->do_dns_lookup();
                break;
            }

            default:
            {
                break;
            }
        }
    }

    void do_dns_lookup()
    {
        espconn_gethostbyname(&conn_, host_.c_str(), &targetIp_, dns_done);
    }

    static void dns_done(const char *name, ip_addr_t *ipaddr, void *arg)
    {
        if (ipaddr == NULL)
        {
            os_printf("DNS lookup failed\n");
            wifi_station_disconnect();
            return;
        }
        Singleton<ESPWifiClient>::instance()->do_connect(ipaddr);
    }

    void do_connect(ip_addr_t *ipaddr)
    {
        os_printf("Connecting...\n");

        conn_.type = ESPCONN_TCP;
        conn_.state = ESPCONN_NONE;
        conn_.proto.tcp = &tcp_;
        conn_.proto.tcp->local_port = espconn_port();
        conn_.proto.tcp->remote_port = port_;
        memcpy(conn_.proto.tcp->remote_ip, &ipaddr->addr, 4);

        espconn_regist_connectcb(&conn_, static_tcp_connected);
        espconn_regist_disconcb(&conn_, static_tcp_disconnected);

        espconn_connect(&conn_);
    }

    static void static_tcp_connected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_connected();
    }

    void tcp_connected() {
        gcAdapter_.reset(
            GCAdapterBase::CreateGridConnectAdapter(&gcHub_, hub_, false));
        espconn_regist_recvcb(&conn_, static_data_received);
        gcHub_.register_port(this);
    }

    static void static_tcp_disconnected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_disconnected();
    }

    void tcp_disconnected() {
        gcHub_.unregister_port(this);
        gcAdapter_->shutdown();
        wifi_station_disconnect();
    }

    static void static_data_received(void *arg, char *pdata, unsigned short len)
    {
        Singleton<ESPWifiClient>::instance()->data_received(pdata, len);
    }

    void data_received(char *pdata, unsigned short len)
    {
        auto *b = gcHub_.alloc();
        b->data()->skipMember_ = this;
        b->data()->assign(pdata, len);
        gcHub_.send(b);
    }

private:
    Action entry() override
    {
        espconn_sent(
            &conn_, (uint8*)message()->data()->data(), message()->data()->size());
        return release_and_exit();
    }

    struct station_config stationConfig_;
    struct espconn conn_;
    esp_tcp tcp_;

    ip_addr_t targetIp_;

    string host_;
    int port_;
    CanHubFlow *hub_;
    HubFlow gcHub_;
    std::unique_ptr<GCAdapterBase> gcAdapter_;
};

#endif // _UTILS_ESPWIFICLIENT_HXX_
