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
#include <fcntl.h>
#include <ifaddrs.h>

#include "executor/StateFlow.hxx"
#include "executor/Timer.hxx"
#include "os/MDNS.hxx"
#include "utils/Atomic.hxx"
#include "utils/format_utils.hxx"

class SocketClient : public StateFlowBase, private Atomic
{
public:
    /** Connection status that can be sent back to the "owner" of the socket so
    * it can update display or status information while the connection attempts
    * are progressing.
    */
    enum class Status
    {
        MDNS_LOOKUP,
        MDNS_CONNECT,
        STATIC_CONNECT,
        CONNECT_FAILED,
        CONNECT_FAILED_SELF,
    };

    /** Constructor.
     * @param service service that the StateFlowBase will be bound to. This
     * service will never be blocked. Externally owned.
     * @param connect_executor is a thread on which DNS lookups and blocking
     * connect calls will be attempted. Externally owned. May be shared between
     * different SocketClients.
     * @param mdns_executor is a thread on which mdns lookups will be
     * attempted. May be null if mdns is never used (by the parameters) or may
     * be the same executor as connect_executor if connect and mdns attempts
     * ought to be serialized. Externally owned. May be shared between
     * different SocketClients.
     * @param params defines all the different parameters on whom to connect
     * to. Takes ownership.
     * @param connect_callback callback method to invoke when a client
     * connection is made successfully. It is unspecified which thread this
     * callback will be invoked upon.
     * - First param is the file descriptor of the resulting socket
     * - Second param is a notifiable (ownership is not transferred). The
     * callee must invoke this notifiable when the socket has been torn down in
     * order to restart the search with the same parameters.
     */
    SocketClient(Service *service, ExecutorBase *connect_executor,
        ExecutorBase *mdns_executor, std::unique_ptr<SocketClientParams> params,
        std::function<void(int, Notifiable *)> connect_callback)
        : StateFlowBase(service)
        , params_(std::move(params))
        , callback_(callback)
        , connectExecutor_(connect_executor)
        , mdnsExecutor_(mdns_executor)
        , state_(STATE_CREATED)
        , fd_(-1)
        , addr_(nullptr)
    {
        HASSERT(mdns_ || (host_ && port_));
        start_flow(STATE(spawn_thread));
    }

    /** Destructor.
     */
    ~SocketClient()
    {
        shutdown();
        if (addr_)
        {
            freeaddrinfo(addr_);
        }
    }

    /// Updates the parameter structure for this socket client.
    /// @param params is the parameter structure; ownership will be
    /// transferred.
    void reset_params(std::unique_ptr<SocketClientParams> params)
    {
        params_.reset(std::move(params));
        /// @todo (balazs.racz): do we need to somehow wake up the flow and
        /// make it attempt to reconnect?
    }

    /** Shutdown the client so that it can be deleted.
     */
    void shutdown()
    {
        start_shutdown();
        while (state_ != STATE_SHUTDOWN)
        {
            usleep(1000);
        }
    }

    /** Reports if this instance has finished shutting down
     */
    bool is_shutdown()
    {
        return state_ == STATE_SHUTDOWN;
    }

    /** Request that this client shutdown and exit the other thread.
     */
    void start_shutdown()
    {
        {
            AtomicHolder h(this);
            if (state_ != STATE_SHUTDOWN)
            {
                state_ = STATE_SHUTDOWN_REQUESTED;
            }
        }
        sem_.post();
    }

    /** Connects a tcp socket to the specified remote host:port. Returns -1 if
     *  unsuccessful; returns the fd if successful.
     *
     *  @param host hostname to connect to
     *  @param port TCP port number to connect to
     *
     *  @return fd of the connected socket.
     */
    static int connect(const char *host, int port);

    /** Connects a tcp socket to the specified remote host:port. Returns -1 if
     *  unsuccessful; returns the fd if successful.
     *
     *  @param host hostname to connect to. Shall be null for mDNS target.
     *  @param port_str TCP port number or mDNS hostname to connect to.
     *
     *  @return fd of the connected socket.
     */
    static int connect(const char *host, const char* port_str);

private:
    /** Execution state.
     */
    enum State
    {
        STATE_CREATED = 0, /**< constructed */
        STATE_STARTED,     /**< thread started */
        STATE_SHUTDOWN_REQUESTED, /**< shutdown requested */
        STATE_SHUTDOWN, /**< shutdown */
    };


    /// Main entry point of the connection process.
    Action start_connection() {
        try_last_connect();
        
    }

    /// Helper function to attempt to use the reconnect slot. If succeeds, then
    /// enqueues an asynchronous connection.
    void try_last_connect() {
        if (!params_->enable_last()) {
            return;
        }
        int port = params_->last_port();
        if (port <= 0) {
            return;
        }
        const char* host = params_->last_host_name();
        if (!host || !*host) {
            return;
        }
        schedule_connect(host, port);
    }

    /// Helper function to schedule asynchronous work on the connect
    /// thread. Never blocks.
    /// @param host hostname (or IP address in text form) to connect to
    /// @param port port number to connect to.
    void schedule_connect(const char* host, int port) {
        connectExecutor_->add(new ConnectExecutable(this, host, port));
    }

    /// Helper class to schedule blocking work on a different executor. This
    /// class calls connect_blocking. Owns itself.
    class ConnectExecutable : public Executable {
    public:
        /// @param parent the SocketClient that created us
        /// @param host hostname (or IP address in text form) to connect to
        /// @param port port number to connect to.
        ConnectExecutable(SocketClient* parent, string host, int port)
            : parent_(parent)
            , host_(std::move(host))
            , port_(port) {
        }

        void run() override {
            parent_->connect_blocking(host_, port_);
            delete this;
        }
    };

    /// Called on the connect executor.
    /// @param host hostname (or IP address in text form) to connect to
    /// @param port port number to connect to.
    void connect_blocking(const string& host, int port) {
        struct addrinfo hints;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = 0;
        hints.ai_protocol = IPPROTO_TCP;

        struct addrinfo* addr;
        char port_str[30];
        integer_to_buffer(port, port_str);
        int ai_ret = getaddrinfo(host.c_str(), port_str, &hints, &addr);
        std::unique_ptr<struct addrinfo, freeaddrinfo> addr_holder(addr);

        // @todo check request shutdown here
        
    }
    
    /** Entry point to the thread; this thread performs the synchronous network
     * calls (address resolution and connect). The function returns only when
     * the thread needs to be terminated (i.e. after shutdown() is invoked).
     *
     * When this method is first called, the state should be STATE_CREATED. This
     * will then switch to STATE_STARTED and remain there for most of the
     * lifetime of this method. It will switch to STATE_SHUTDOWN_REQUESTED after
     * the shutdown() method is called, and then to STATE_SHUTDOWN once it
     * finishes shutting down.
     *
     * @return does not return a value, but exits after shutdown
     */
    void *entry() override
    {
        int ai_ret = -1;
        struct addrinfo hints;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = 0;
        hints.ai_protocol = IPPROTO_TCP;

        sem_.wait();

        for ( ; /* forever */ ; )
        {
            {
                AtomicHolder h(this);
                switch (state_)
                {
                    case STATE_CREATED:
                        state_ = STATE_STARTED;
                    case STATE_STARTED:
                        break;
                    case STATE_SHUTDOWN_REQUESTED:
                        state_ = STATE_SHUTDOWN;
                    case STATE_SHUTDOWN:
                        return nullptr;
                }
            }
            long long start_time = OSTime::get_monotonic();

            if (mdns_)
            {
                LOG(INFO, "mdns lookup for %s", mdns_);
                /* try mDNS address resolution */
                update_status(Status::MDNS_LOOKUP);
                ai_ret = MDNS::lookup(mdns_, &hints, &addr_);
                if (ai_ret != 0 || addr_ == nullptr)
                {
                    LOG(INFO, "mdns lookup for %s failed.", mdns_);
                }
                else
                {
                    update_status(Status::MDNS_CONNECT);
                }
                
            }
            if ((ai_ret != 0 || addr_ == nullptr) && host_)
            {
                /* try address resolution without mDNS */
                update_status(Status::STATIC_CONNECT);
                char port_str[30];
                integer_to_buffer(port_, port_str);
                ai_ret = getaddrinfo(host_, port_str, &hints, &addr_);
            }

            if (ai_ret == 0 && addr_)
            {
                /* able to resolve the hostname to an address */
                bool addr_okay = true;
                if (addr_->ai_addr->sa_family != AF_INET)
                {
                    /* we only support IPv4 addresses */
                    addr_okay = false;
                }
                if (addr_okay && disallowLocal_)
                {
                    /* test for trying to connect to self */
                    addr_okay = !local_test(addr_);
                    if (!addr_okay)
                    {
                        update_status(Status::CONNECT_FAILED_SELF);
                    }
                }
                if (addr_okay)
                {
                    /* we have a valid IPv4 address that is not ourselves */
                    fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
                    if (fd_ >= 0)
                    {
                        {
                            struct timeval tm;
                            tm.tv_sec = timeoutSeconds_;
                            tm.tv_usec = 0;
                            ERRNOCHECK("setsockopt_timeout",
                                       setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO,
                                                  &tm, sizeof(tm)
                                                 )
                                      );
                        }
                        /* socket available */
                        int ret = ::connect(fd_, addr_->ai_addr,
                                            addr_->ai_addrlen);
                        if (ret == 0)
                        {
                            /* test for possible connection to self, again */
                            if (disallowLocal_ && local_test(addr_))
                            {
                                /* connected to self */
                                update_status(Status::CONNECT_FAILED_SELF);
                                /* connect failed */
                                close(fd_);
                            }
                            else
                            {
                                /* connect successful */
                                notify();
                                sem_.wait();
                            }
                        }
                        else
                        {
                            update_status(Status::CONNECT_FAILED);
                            /* connect failed */
                            close(fd_);
                        }
                    }
                }
            }

            if (addr_)
            {
                freeaddrinfo(addr_);
                addr_ = nullptr;
            }

            long long diff_time = OSTime::get_monotonic() - start_time;
            if (NSEC_TO_SEC(diff_time) < retrySeconds_)
            {
                sleep(retrySeconds_ - NSEC_TO_SEC(diff_time));
            }
        }

        /* should never get here */
        return nullptr;
    }

    void update_status(Status status)
    {
        if (statusCallback_ != nullptr)
        {
            statusCallback_(status);
        }
    }

    /** Entry point into the state flow. Create a new thread, which will then
     * call the entry() method of this class.
     * @return next state is do_connect()
     */
    Action spawn_thread()
    {
        start("socket_client", 0, 1536);
        return call_immediately(STATE(do_connect));
    }

    /** Kick off connection attempt.
     * @return next state is connected() upon successful connection
     */
    Action do_connect()
    {
        sem_.post();
        return wait_and_call(STATE(connected));
    }

    /** Connected successfully, notify user through a callback.
     * @return next state is do_connect() after the connection has been broken
     */
    Action connected()
    {
        /* connect successful */
        callback_(fd_, addr_, this);
        return wait_and_call(STATE(do_connect));
    }

    /** Test if a given address is local.
     * @param addr address info to test
     * @return true if local, else false if not local
     */
    bool local_test(struct addrinfo *addr);

    /// Stores the parameter structure.
    std::unique_ptr<SocketClientParams> params_;
    
    /// callback to call on connection success
    std::function<void(int, Notifiable*)> callback_ = nullptr;

    /// Executor for synchronous (blocking) connect calls. Externally owned.
    ExecutorBase* connectExecutor_ = nullptr;
    /// Executor for synchronous (blocking) mDNS lookup calls. Externally
    /// owned, may be null.
    ExecutorBase* mdnsExecutor_ = nullptr;
    
    /** current state in the objects lifecycle */
    volatile State state_;

    /** socket descriptor */
    int fd_;

    /** address info metadata */
    struct addrinfo *addr_;

    DISALLOW_COPY_AND_ASSIGN(SocketClient);
};

#endif /* _UTILS_SOCKET_CLIENT_HXX */

