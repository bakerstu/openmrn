/** @copyright
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
 * @file SocketClient.hxx
 *
 * Connects to a remote TCP socket.
 *
 * @author Stuart Baker
 * @date 11 March 2017
 */

#ifndef _UTILS_SOCKET_CLIENT_HXX_
#define _UTILS_SOCKET_CLIENT_HXX_

#include <functional>
#include <netdb.h>
#include <arpa/inet.h>

#include "executor/StateFlow.hxx"
#include "executor/Timer.hxx"

class SocketClient : public StateFlowBase, public OSThread , public OSSem
{
public:
    /** Constructor.
     * @param service service that the StateFlowBase will be bound to.
     * @param mdns_name service name to connect to, nullptr to force use
     *                  hostsname and port
     * @param host host to connect to if mdns_name resolution fails,
     *             nullptr to force use mDNS
     * @param port port number to connect to if mdns_name resolution fails
     * @param callback callback method to invoke when a client connection is
     *                 made successfully.  It is the responsibility of the
     *                 callee to register the notifiable (on close) if this
     *                 client shall reattempt the connection if the socket
     *                 is ever closed.
     * @param retry_seconds time in seconds that the client shall wait to retry
     *                      connecting on error.
     */
    SocketClient(Service *service, const char *mdns, const char * host,
                 int port,
                 std::function<void(int, char*, int, Notifiable*)>callback,
                 unsigned retry_seconds = 5)
        : StateFlowBase(service)
        , OSThread()
        , OSSem()
        , mdns_(mdns)
        , host_(host)
        , port_(port)
        , callback_(callback)
        , retrySeconds_(retry_seconds)
        , selectHelper_(this)
        , timer_(this)
        , fd_(-1)
        , addr_(nullptr)
    {
        HASSERT(mdns_ || (host_ && port_));
        start_flow(STATE(spawn_thread));
    }

private:
    /** thread that will handle the blocking address resolution.
     * @return should never return
     */
    virtual void *entry() override
    {
        int ai_ret;
        struct addrinfo hints;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = 0;
        hints.ai_protocol = IPPROTO_TCP;

        for ( ; /* forever */ ; )
        {
            OSSem::wait();
            for ( ; /* forever */ ; )
            {
                if (mdns_)
                {
                    /* mdns address resolution */
                    ai_ret = getaddrinfo(nullptr, mdns_, &hints, &addr_);
                }
                else
                {
                    char port_str[30];
                    integer_to_buffer(port_, port_str);
                    ai_ret = getaddrinfo(host_, port_str, &hints, &addr_);
                }
                if (ai_ret == 0 && addr_)
                {
                    notify();
                    break;
                }
                if (addr_)
                {
                    freeaddrinfo(addr_);
                }
                sleep(retrySeconds_);
            }
        }

        return nullptr;
    }

    /** Entry point into the state flow.
     * @return next state is find_host()
     */
    Action spawn_thread()
    {
        start("socket_client", 0, 1024);
        return call_immediately(STATE(find_host));
    }

    /** Kick off host name resolution.
     * @return next state is do_connect() upon successful host name resolution
     */
    Action find_host()
    {
        OSSem::post();
        return wait_and_call(STATE(do_connect));
    }

    /** Start the connection attempt.
     * @return next stat is connect_active() on success or connect in progress,
     *         else find_host() after timeout on error
     */
    Action do_connect()
    {
        fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd_ < 0)
        {
            /* no available socket */
            return sleep_and_call(&timer_, SEC_TO_NSEC(retrySeconds_),
                                  STATE(find_host));
        }

        /* set to non-blocking */
        ::fcntl(fd_, F_SETFL, O_NONBLOCK);

        int ret = connect(fd_, addr_->ai_addr, addr_->ai_addrlen);
        if (ret == 0)
        {
            /* connect successful */
            return call_immediately(STATE(connect_active));
        }
        else if (ret < 0 && errno == EINPROGRESS)
        {
            /* connection pending */
            return connect_and_call(&selectHelper_, fd_, STATE(connect_active));
        }
        else
        {
            /* connect failed */
            close(fd_);
            return sleep_and_call(&timer_, SEC_TO_NSEC(retrySeconds_),
                                  STATE(find_host));
        }
    }

    /** Start the connection attempt
     * @return next state is find_host() after a timeout upon error, or upon
               success, after the connection has been broken
     */
    Action connect_active()
    {
        /// @todo small delay seems to be required for CC32xx.  Not sure why.
        usleep(10000);
        int ret = connect(fd_, addr_->ai_addr, addr_->ai_addrlen);
        freeaddrinfo(addr_);
        if (ret < 0)
        {
            if (errno != EISCONN)
            {
                /* connection failed or timed out */
                close(fd_);
                return sleep_and_call(&timer_, SEC_TO_NSEC(retrySeconds_),
                                      STATE(find_host));
            }
        }

        /* connect successful */
        callback_(fd_, nullptr, -1, this);
        return wait_and_call(STATE(find_host));
    }

    /** mDNS service name */
    const char *mdns_;

    /** hostname */
    const char *host_;

    /** port number */
    int port_;

    /** callback to call on connection success */
    std::function<void(int, char *, int, Notifiable*)> callback_;

    /** number of seconds between retries */
    unsigned retrySeconds_;

    /** helper for non-blocking connect */
    StateFlowSelectHelper selectHelper_;

    /** retry timer */
    StateFlowTimer timer_;

    /** socket descriptor */
    int fd_;

    /** address info metadata */
    struct addrinfo *addr_;

    DISALLOW_COPY_AND_ASSIGN(SocketClient);
};

#endif /* _UTILS_SOCKET_CLIENT_HXX */

