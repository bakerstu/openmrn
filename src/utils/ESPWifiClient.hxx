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
#include "ets_sys.h"
#include <ip_addr.h>


#include <espconn.h>
#include <osapi.h>
#include <user_interface.h>
}

#include "utils/GridConnectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/Singleton.hxx"

/// Uses the ESPConn WiFi API on an ESP8266 to set up the module in SoftAP
/// mode.
class ESPWifiAP : public Singleton<ESPWifiAP>
{
public:
    ESPWifiAP(const string &ssid, const string &password, int channel = 9)
    {
        memset(&apConfig_, 0, sizeof(apConfig_));
        wifi_set_opmode_current(SOFTAP_MODE);
        memcpy(&apConfig_.ssid, ssid.c_str(), ssid.size() + 1);
        apConfig_.ssid_len = ssid.size();
        memcpy(&apConfig_.password, password.c_str(), password.size() + 1);
        apConfig_.channel = channel;
        if (password.empty())
        {
            apConfig_.authmode = AUTH_OPEN;
        }
        else
        {
            apConfig_.authmode = AUTH_WPA_WPA2_PSK;
        }
        apConfig_.max_connection = 4;
        apConfig_.beacon_interval = 100;

        wifi_softap_set_config(&apConfig_);
        wifi_set_event_handler_cb(&static_wifi_callback);
        // wifi_softap_dhcps_start();
    }

    /// Callback when an event happens on the wifi port. @param evt describes
    /// event that happened.
    static void static_wifi_callback(System_Event_t *evt)
    {
        switch (evt->event)
        {
            case EVENT_SOFTAPMODE_STACONNECTED:
            {
                os_printf("SOFTAP: station connected %d: " MACSTR "\n",
                    evt->event_info.sta_connected.aid,
                    MAC2STR(evt->event_info.sta_connected.mac));
                break;
            }

            case EVENT_SOFTAPMODE_STADISCONNECTED:
            {
                os_printf("SOFTAP: station disconnected %d: " MACSTR "\n",
                    evt->event_info.sta_disconnected.aid,
                    MAC2STR(evt->event_info.sta_connected.mac));
                break;
            }

            case EVENT_SOFTAPMODE_PROBEREQRECVED:
            {
                // We don't care about probe received events, ignore.
                break;
            }
            default:
            {
                os_printf("SOFTAP: unhandled event %d\n", evt->event);
                break;
            }
        }
    }

    /// Configuration of the access point we are creating.
    struct softap_config apConfig_;
};

/// Starts a listening socket for gridconnect TCP connections using the EspConn
/// API.
class ESPGcTcpServer : public Singleton<ESPGcTcpServer>
{
public:
    /// Constructor
    /// @param hub CAN hub to connect to the server
    /// @param send_buf_size in bytes, how much to buffer befone handing over to
    /// the TCP stack.
    ESPGcTcpServer(CanHubFlow *hub, unsigned send_buf_size)
        : hub_(hub)
        , bufSize_(send_buf_size)
    { }

    /// OpenLCB gridconnect TCP port number.
    static constexpr unsigned LISTEN_PORT = 12021;

    /// Starts up the server.
    void start()
    {
        start_listen();
    }

    void start_mdns()
    {
        struct ip_info ipconfig;
        wifi_get_ip_info(SOFTAP_IF, &ipconfig);

        struct mdns_info *info =
            (struct mdns_info *)malloc(sizeof(struct mdns_info));
        memset(info, 0, sizeof(struct mdns_info));
        info->host_name = (char*)"esp_host";
        info->ipAddr = ipconfig.ip.addr; // sation ip
        info->server_name = (char*)"openlcb-can";
        info->server_port = LISTEN_PORT;
        espconn_mdns_init(info);
    }

private:
    /// Starts the listening socket for the TCP server.
    void start_listen()
    {
        memset(&conn_, 0, sizeof(conn_));
        memset(&tcp_, 0, sizeof(tcp_));

        conn_.type = ESPCONN_TCP;
        conn_.state = ESPCONN_NONE;
        conn_.proto.tcp = &tcp_;
        conn_.proto.tcp->local_port = LISTEN_PORT;
        espconn_regist_connectcb(&conn_, on_connect);
        espconn_regist_disconcb(&conn_, on_discon);
        espconn_regist_reconcb(&conn_, on_recon);

        espconn_accept(&conn_);
        os_printf("Server: Listening on port %d / %p\n",
            conn_.proto.tcp->local_port, &conn_);
    }

    /// Callback when a client connected to the server socket.
    static void on_connect(void *arg)
    {
        struct espconn *pesp_conn = static_cast<struct espconn *>(arg);

        os_printf("Server: connection from %d.%d.%d.%d:%d / %p\n",
            pesp_conn->proto.tcp->remote_ip[0],
            pesp_conn->proto.tcp->remote_ip[1],
            pesp_conn->proto.tcp->remote_ip[2],
            pesp_conn->proto.tcp->remote_ip[3],
            pesp_conn->proto.tcp->remote_port, pesp_conn);

        espconn_regist_recvcb(pesp_conn, on_recv);
        espconn_regist_disconcb(pesp_conn, on_discon);
        espconn_regist_reconcb(pesp_conn, on_recon);
    }

    /// Invoked by the system when a connected TCP socket disconnects.
    /// @param arg the espconn pointer.
    static void on_discon(void *arg)
    {
        struct espconn *pesp_conn = static_cast<struct espconn *>(arg);

        os_printf("Server: disconnected %d.%d.%d.%d:%d / %p\n",
            pesp_conn->proto.tcp->remote_ip[0],
            pesp_conn->proto.tcp->remote_ip[1],
            pesp_conn->proto.tcp->remote_ip[2],
            pesp_conn->proto.tcp->remote_ip[3],
            pesp_conn->proto.tcp->remote_port, pesp_conn);

        /// @TODO delete connection.
    }

    /// Invoked by the system when a connected TCP socket disconnects.
    /// @param arg the espconn pointer.
    static void on_recon(void *arg, sint8 err)
    {
        struct espconn *pesp_conn = static_cast<struct espconn *>(arg);

        os_printf("Server: reconnected %d.%d.%d.%d:%d err %d / %p\n",
            pesp_conn->proto.tcp->remote_ip[0],
            pesp_conn->proto.tcp->remote_ip[1],
            pesp_conn->proto.tcp->remote_ip[2],
            pesp_conn->proto.tcp->remote_ip[3],
            pesp_conn->proto.tcp->remote_port, err, pesp_conn);

        /// @TODO delete connection.
    }

    /// Invoked by the system when there is incoming data coming on a TCP
    /// socket that is already connected.
    /// @param arg the espconn pointer.
    /// @param pdata pointer to data received.
    /// @param len number of bytes received.
    static void on_recv(void *arg, char *pusrdata, unsigned short length)
    { }

    /// Converts an espconn pointer to a key representing the TCP connection
    /// with remote ip, remote port, local port tuple.
    /// @param arg espconn callback, typically a pointer to an espconn
    /// structure.
    /// @return a key composed of the identifying attributes of the TCP
    /// connection.
    static uint64_t key_from_espconn(void *arg)
    {
        struct Key
        {
            uint16_t remote_port;
            uint16_t local_port;
            uint8 remote_ip[4];
        };
        struct KU
        {
            union
            {
                Key k;
                uint64_t u;
            };
        } ku;
        static_assert(sizeof(ku) == 8, "incorrect alignment or something");
        struct espconn *pespconn = static_cast<struct espconn *>(arg);
        if (!pespconn || pespconn->type != ESPCONN_TCP)
        {
            return 0;
        }
        auto *tcp = pespconn->proto.tcp;
        memcpy(ku.k.remote_ip, tcp->remote_ip, 4);
        ku.k.remote_port = tcp->remote_port;
        ku.k.local_port = tcp->local_port;
        return ku.u;
    }

    /// Stores the values we need to know about a single client connection.
    struct Conn
    {
        uint64_t key_;
        Conn(void *arg)
            : key_(key_from_espconn(arg))
        { }
    };

    /// Finds the index of a known connection.
    /// @param arg callback argument from an espconn callback.
    /// @return the index at which the matching Conn is stored in the conns_
    /// array, or -1 if not found.
    static int idx_by_espconn(void *arg)
    {
        uint64_t key = key_from_espconn(arg);
        auto &v = instance()->conns_;
        for (unsigned i = 0; i < v.size(); ++i)
        {
            if (v[i]->key_ == key)
            {
                return i;
            }
        }
        return -1;
    }

    /// Finds a connection indexed by espconn*.
    /// @param arg callback argument from an espconn callback.
    /// @return the matchin Conn instance, or nullptr if not known.
    static Conn *lookup_by_espconn(void *arg)
    {
        int id = idx_by_espconn(arg);
        if (id < 0)
        {
            return nullptr;
        }
        return instance()->conns_[id].get();
    }

    /// Removes a connection indexed by espconn*.
    static void erase_by_espconn(void *arg)
    {
        int id = idx_by_espconn(arg);
        if (id < 0)
        {
            os_printf("Server: failed to delete entry\n");
            return;
        }
        auto &v = instance()->conns_;
        v.erase(v.begin() + id);
    }

    /// Listening server socket connection handle.
    struct espconn conn_;
    /// TCP connection handle for server socket.
    esp_tcp tcp_;

    /// The list of connected TCP sockets.
    std::vector<std::unique_ptr<Conn>> conns_;
    /// CAN hub to send incoming packets to and to receive outgoing packets
    /// from.
    CanHubFlow *hub_;
    /// How many bytes are there in the send buffer.
    unsigned bufSize_;
};

/// Uses the ESPConn API on the ESP8266 wifi-enabled MCU to connect to a wifi
/// base station, perform a DNS lookup to a given target, and connect to a
/// given port via TCP. Acts as a HubPort, meaning the data coming from the Hub
/// (usually gridconnect packets) are sent to the TCP connection.
class ESPWifiClient : public Singleton<ESPWifiClient>, private HubPort
{
public:
    /// Creates a wifi+TCP client connection via the ESPConn API.
    ///
    /// @param ssid wifi access point name
    /// @param password passphrase for the wifi access point, or empty string
    /// for open (unencrypted) connection.
    /// @param hub CAN hub to connect to the server
    /// @param hostname hostname of the gridconnect TCP hub server. IP address
    /// in dot format is not supported.
    /// @param port port number of the TCP hub server.
    /// @param send_buf_size in bytes, how much to buffer befone handing over to
    /// the TCP stack.
    /// @param connect_callback will be called after the wifi and the TCP
    /// connection is established. Usually used for starting the OpenLCB stack.
    ESPWifiClient(const string &ssid, const string &password, CanHubFlow *hub,
        const string &hostname, int port, unsigned send_buf_size,
        std::function<void()> connect_callback)
        : HubPort(hub->service())
        , host_(hostname)
        , port_(port)
        , sendPending_(0)
        , sendBlocked_(0)
        , timerPending_(0)
        , sendBuf_((uint8_t *)malloc(send_buf_size))
        , bufSize_(send_buf_size)
        , hub_(hub)
        , gcHub_(hub_->service())
        , connectCallback_(std::move(connect_callback))
    {
        wifi_set_opmode_current(STATION_MODE);
        memcpy(&stationConfig_.ssid, ssid.c_str(), ssid.size() + 1);
        memcpy(&stationConfig_.password, password.c_str(), password.size() + 1);
        wifi_station_set_config(&stationConfig_);
        wifi_set_event_handler_cb(&static_wifi_callback);
        wifi_station_connect(); // may be avoided if the constructor is still
                                // called in the user_init.
    }

    /// Callback when an event happens on the wifi port. @param evt describes
    /// event that happened.
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
                os_printf("ip: %08x\n", evt->event_info.got_ip.ip);

                Singleton<ESPWifiClient>::instance()->do_dns_lookup();
                break;
            }

            default:
            {
                break;
            }
        }
    }

    /// Initiates the DNS lookup of the target host.
    void do_dns_lookup()
    {
        os_printf("DNS lookup start: %s\n", host_.c_str());
        espconn_gethostbyname(&conn_, host_.c_str(), &targetIp_, dns_done);
    }

    /// Callback when the DNS lookup is completed.
    ///
    /// @param name what we were looking up (ignored)
    /// @param ipaddr ip address of the host
    /// @param arg ignored.
    ///
    static void dns_done(const char *name, ip_addr_t *ipaddr, void *arg)
    {
        os_printf("dns_done\n");
        if (ipaddr == NULL)
        {
            os_printf("DNS lookup failed\n");
            wifi_station_disconnect();
            return;
        }
        Singleton<ESPWifiClient>::instance()->do_connect(ipaddr);
    }

    /// Connects to the specific IP address. @param ipaddr is the address of
    /// the host we wanted to connect to.
    void do_connect(ip_addr_t *ipaddr)
    {
        os_printf("Connecting to %s:%d...\n", host_.c_str(), port_);

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

    /// Callback when the TCP connection is established. @param arg ignored.
    static void static_tcp_connected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_connected();
    }

    /// Callback when the TCP connection is established.
    void tcp_connected()
    {
        os_printf("%s\n", __FUNCTION__);
        gcAdapter_.reset(
            GCAdapterBase::CreateGridConnectAdapter(&gcHub_, hub_, false));
        espconn_regist_recvcb(&conn_, static_data_received);
        espconn_regist_sentcb(&conn_, static_data_sent);
        gcHub_.register_port(this);
        connectCallback_();
    }

    /// Callback when the TCP connection is lost. @param arg ignored.
    static void static_tcp_disconnected(void *arg)
    {
        os_printf("%s\n", __FUNCTION__);
        Singleton<ESPWifiClient>::instance()->tcp_disconnected();
    }

    /// Callback when the TCP connection is lost.
    void tcp_disconnected()
    {
        gcHub_.unregister_port(this);
        gcAdapter_->shutdown();
        wifi_station_disconnect();
    }

    /// Callback for incoming data.
    ///
    /// @param arg ignored
    /// @param pdata pointer to data received.
    /// @param len number of bytes received.
    static void static_data_received(void *arg, char *pdata, unsigned short len)
    {
        Singleton<ESPWifiClient>::instance()->data_received(pdata, len);
    }

    /// Callback when data that was requested to be sent has completed
    /// sending. @param arg ignored.
    static void static_data_sent(void *arg)
    {
        Singleton<ESPWifiClient>::instance()->data_sent();
    }

    /// Callback when incoming data is received.
    ///
    /// @param pdata incoming data
    /// @param len number of bytes received.
    ///
    void data_received(char *pdata, unsigned short len)
    {
        auto *b = gcHub_.alloc();
        b->data()->skipMember_ = this;
        b->data()->assign(pdata, len);
        gcHub_.send(b);
    }

private:
    /// Sending base state. @return next action.
    Action entry() override
    {
        if (sendPending_)
        {
            sendBlocked_ = 1;
            // Will call again once the wifi notification comes back.
            return wait();
        }
        if (message()->data()->size() > bufSize_)
        {
            if (bufEnd_ > 0)
            {
                // Cannot copy the data to the buffer. Must send separately.
                send_buffer();
                return again();
            }
            else
            {
                sendPending_ = 1;
                sendBlocked_ = 1; // will cause notify.
                espconn_sent(&conn_, (uint8 *)message()->data()->data(),
                    message()->data()->size());
                return wait_and_call(STATE(send_done));
            }
        }
        if (message()->data()->size() > bufSize_ - bufEnd_)
        {
            // Doesn't fit into the current buffer.
            send_buffer();
            return again();
        }
        // Copies the data into the buffer.
        memcpy(sendBuf_ + bufEnd_, message()->data()->data(),
            message()->data()->size());
        bufEnd_ += message()->data()->size();
        release();
        // Decides whether to send off the buffer now.
        if (!queue_empty())
        {
            // let's process more of the queue
            return exit();
        }
        if (!timerPending_)
        {
            timerPending_ = 1;
            bufferTimer_.start(MSEC_TO_NSEC(3));
        }
        return exit();
    }

    /// Called from the timer to signal sending off the buffer's contents.
    void timeout()
    {
        timerPending_ = 0;
        if (!sendPending_)
        {
            send_buffer();
        }
    }

    /// Callback state when we are sending directly off of the input buffer
    /// becuase the data payload is too much to fit into the send assembly
    /// buffer. Called when the send is completed and the input buffer can be
    /// releases. @return next action
    Action send_done()
    {
        return release_and_exit();
    }

    /// Writes all bytes that are in the send buffer to the TCP socket.
    void send_buffer()
    {
        espconn_sent(&conn_, sendBuf_, bufEnd_);
        sendPending_ = 1;
    }

    /// Callback from the TCP stack when the data send has been completed.
    void data_sent()
    {
        sendPending_ = 0;
        bufEnd_ = 0;
        if (sendBlocked_)
        {
            sendBlocked_ = 0;
            notify();
        }
    }

    /// Timer that triggers the parent flow when expiring. Used to flush the
    /// accumulated gridconnect bytes to the TCP socket.
    class BufferTimer : public ::Timer
    {
    public:
        /// Constructor. @param parent who owns *this.
        BufferTimer(ESPWifiClient *parent)
            : Timer(parent->service()->executor()->active_timers())
            , parent_(parent)
        { }

        /// callback when the timer expires. @return do not restart timer.
        long long timeout() override
        {
            parent_->timeout();
            return NONE;
        }

    private:
        ESPWifiClient *parent_; ///< parent who owns *this.
    } bufferTimer_ {this};      ///< Instance of the timer we own.

    /// Configuration of the access pint we are connecting to.
    struct station_config stationConfig_;
    /// IP (including DNS) connection handle.
    struct espconn conn_;
    /// TCP connection handle.
    esp_tcp tcp_;

    /// IP address of the target host.
    ip_addr_t targetIp_;
    /// Target host we are trying to connect to.
    string host_;
    /// Port numer we are connecting to on the target host.
    int port_;
    /// True when the TCP stack is busy.
    uint16_t sendPending_ : 1;
    /// True when we are waiting for a notification from the TCP stack send done
    /// callback.
    uint16_t sendBlocked_ : 1;
    /// True when there is a send timer running with the assembly buffer being
    /// not full.
    uint16_t timerPending_ : 1;

    /// Offset in sendBuf_ of the first unused byte.
    uint16_t bufEnd_ : 16;
    /// Temporarily stores outgoing data until the TCP stack becomes free.
    uint8_t *sendBuf_;
    /// How many bytes are there in the send buffer.
    unsigned bufSize_;
    /// CAN hub to send incoming packets to and to receive outgoing packets
    /// from.
    CanHubFlow *hub_;
    /// String-typed hub for the gridconnect-rendered packets.
    HubFlow gcHub_;
    /// Transcoder bridge from CAN to GridConnect protocol.
    std::unique_ptr<GCAdapterBase> gcAdapter_;
    /// Application level callback function to call when the connection has
    /// been successfully established.
    std::function<void()> connectCallback_;
};

#endif // _UTILS_ESPWIFICLIENT_HXX_
