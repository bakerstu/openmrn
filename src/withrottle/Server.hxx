/** \copyright
 * Copyright (c) 2016, Stuart W Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * \file withrottle/Server.hxx
 *
 * This file provides the WiThrottle server objects.
 *
 * @author Stuart Baker
 * @date 17 December 2016
 */

#ifndef _WITHROTTLE_SERVER_HXX_
#define _WITHROTTLE_SERVER_HXX_

#include <string>
#include <memory>

#include "executor/Dispatcher.hxx"
#include "executor/Service.hxx"
#include "openlcb/TractionThrottle.hxx"
#include "utils/socket_listener.hxx"
#include "withrottle/Defs.hxx"
#include "withrottle/ServerCommand.hxx"
#include "withrottle/ServerCommandLoco.hxx"

namespace withrottle
{
/* forward declaration */
class ThrottleFlow;

/** WiThrottle server object.
 */
class Server : public Service
{
public:
    /** Constructor.
     * @param name name of the bus
     * @param port TCP port to listen for connections on, -1 for default.
     * @param node reference to the OpenLCB Node that proxies our bus
     */
    Server(const char *name, int port, openlcb::Node *node)
        : Service(&executor)
        , executor(name, 0, 2048)
        , node(node)
        , listener((port >= 0 || port <= UINT16_MAX) ? port : Defs::DEFAULT_PORT,
                   std::bind(&Server::on_new_connection, this,
                   std::placeholders::_1))
    {
    }

    /** Destructor.
     */
    ~Server()
    {
        listener.shutdown();
    }

    /** Start the server.
     */
    void start()
    {
    }

private:
    /** A new throttle connection is made.
     * @param fd socket descriptor
     */
    void on_new_connection(int fd);

    /** The executor that will run the WiThrottle flows. */
    Executor<1> executor;

    /** node reference */
    openlcb::Node* node;

    /** listen socket for new connections */
    SocketListener listener;

    /** allow access from ThrottleFlow */
    friend class ThrottleFlow;

    DISALLOW_COPY_AND_ASSIGN(Server);
};

/** State flow for handling a throttle instance.
 */
class ThrottleFlow : public StateFlowBase
{
public:
    /** Constructor.
     * @param service service this flow belongs to
     * @param fd socket descriptor of throttle connection.
     * @param node OpenLCB node that proxies our throttles
     */
    ThrottleFlow(Server *server, int fd, openlcb::Node *node);

    /** Destructor.
     */
    ~ThrottleFlow()
    {
        close(fd);
        command->unref();
    }

    /** Start the service.
     */
    void start()
    {
        start_flow(STATE(entry));
    }

private:
    /** Parse the incoming data.
     * @return true if a fully parsed command has been found, else false
     */
    bool parse();

    /** Parse the incoming data. */
    void parse_command();

    /** Parse the incoming data. */
    void parse_multi_type();

    /** Parse the incoming data. */
    void parse_train();

    /** Parse the incoming data.
     * @return true if a fully parsed command has been found, else false
     */
    bool parse_subcommand();

    /** Parse the incoming data. */
    void parse_multi();

    /** Beginning of state flow.
     * @return next state is data_sent()
     */
    StateFlowBase::Action entry();

    /** Data sent successfully.
     * @return next state is data_received()
     */
    StateFlowBase::Action data_sent();

    /** Process read data.
     * @return next state is data_received()
     */
    StateFlowBase::Action data_received();

    /**< OpenLCB throttle instance */
    openlcb::TractionThrottle olcbThrottle;

    /** reference to parent server */
    Server *server;

    string name; /**< name of throttle */
    string id; /**< id of throttle */
    LocoAddress address; /**< primary locomitve addres */
    LocoAddress secondaryAddress; /**< secondary locomotive address */

    /** socket descriptor of throttle connection */
    int fd;

    /** read data buffer */
    char readRaw[128];

    /** agrigate pre-processed stream data */
    string data;

    /** data index for parsing */
    size_t dataIndex;

    /** Current state of parsing the data */
    ServerState state;

    /** Helper for waiting on data from a file descriptor */
    StateFlowSelectHelper selectHelper;

    /** dispatch flow that will handle messages incoming from the cab */
    typedef DispatchFlow<Buffer<ThrottleCommand>, 1> CommandDispatchFlow;

    /** flow responsible for routing incoming messages to handlers. */
    CommandDispatchFlow dispatcher;

    /** throttle command */
    Buffer<ThrottleCommand> *command;

    /** handler for locomotive commands */
    ServerCommandLoco serverCommandLoco;

    /** allow access to private members from class ServerCommandBase */
    friend class ServerCommandBase;

    /** allow access to private members from class ServerCommandLoco */
    friend class ServerCommandLoco;
};

/*
 * Server::on_new_connection()
 */
inline void Server::on_new_connection(int fd)
{
    ThrottleFlow *flow = new ThrottleFlow(this, fd, node);
    flow->start();
}

} /* namespace withrottle */

#endif /* _WITHROTTLE_SERVER_HXX_ */
