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
 * \file withrottle/Server.cxx
 *
 * This file provides the WiThrottle server objects.
 *
 * @author Stuart Baker
 * @date 17 December 2016
 */

#include "withrottle/Server.hxx"

namespace withrottle
{

/** Constructor.
 * @param service service this flow belongs to
 * @param fd socket descriptor of throttle connection.
 * @param node OpenLCB node that proxies our throttles
 */
ThrottleFlow::ThrottleFlow(Server *server, int fd, openlcb::Node *node)
    : StateFlowBase(server)
    , olcbThrottle(node)
    , server(server)
    , fd(fd)
    , data()
    , dataIndex(0)
    , state(STATE_COMMAND)
    , selectHelper(this)
    , dispatcher(server)
    , command(dispatcher.alloc())
    , serverCommandLoco(this)
{
}

/*
 * ThrottleFlow::entry()
 */
StateFlowBase::Action ThrottleFlow::entry()
{
    string init = Defs::get_init_string();

    return write_repeated(&selectHelper, fd, init.data(), init.length(),
                          STATE(data_sent));
}

/*
 * ThrottleFlow::data_sent()
 */
StateFlowBase::Action ThrottleFlow::data_sent()
{
    return read_single(&selectHelper, fd, readRaw, sizeof(readRaw),
                      STATE(data_received));
}

/*
 * ThrottleFlow::data_received()
 */
StateFlowBase::Action ThrottleFlow::data_received()
{
    if (selectHelper.hasError_)
    {
        /* remote throttle has closed the connection */
        printf("WiThrottle connection closed\n");
        return delete_this();
    }

    //printf("WiThrottle strraw: %.*s\n",
    //       sizeof(readRaw) - selectHelper.remaining_, readRaw);

    data.append(readRaw, sizeof(readRaw) - selectHelper.remaining_);
    if (data.find('\n' != string::npos))
    {
        bool result;
        do
        {
            result = parse();
            if (result)
            {
                dispatcher.send(command);
                command = dispatcher.alloc();
            }
        } while (result);
    }

    return read_single(&selectHelper, fd, readRaw, sizeof(readRaw),
                       STATE(data_received));
}

/*
 * ThrottleFlow::parse()
 */
bool ThrottleFlow::parse()
{
    const char *c_str = data.c_str();
    printf("WiThrottle stream: %s\n", c_str + dataIndex);

    do
    {
        switch (state)
        {
            case STATE_COMMAND:
                if (data.length() < 2)
                {
                    /* need more data first */
                    return false;
                }
                parse_command();
                break;
            case STATE_MULTI_TYPE:
                parse_multi_type();
                break;
            case STATE_TRAIN:
                parse_train();
                break;
            case STATE_SUBCOMMAND:
                if (parse_subcommand())
                {
                    return true;
                }
                break;
            case STATE_PAYLOAD:
            default:
            case STATE_NEXT:
                if (data[0] == '\n')
                {
                    dataIndex = 0;
                    state = STATE_COMMAND;
                }
                data.erase(0, 1);
                break;
        }
    } while (data.length());

    return false;
}

/*
 * ThrottleFlow::parse_command()
 */
void ThrottleFlow::parse_command()
{
    command->data()->commandType = (CommandType)data[0];

    switch (data[0])
    {
        default:
        case SECONDARY:
        case HEX_PACKET:
        case PANEL:
        case ROSTER:
        case QUIT:
            state = STATE_NEXT;
            return;
        case MULTI:
            state = data[1] == 'T' ? STATE_MULTI_TYPE : STATE_NEXT;
            data.erase(0, 1);
            break;
        case PRIMARY:
            state = STATE_SUBCOMMAND;
            break;
        case HEARTBEAT:
        case SET_NAME:
            write(fd, "*10\n\n", 5);
        case SET_ID:
            state = STATE_PAYLOAD;
            break;
    }

    data.erase(0, 1);
}

/*
 * ThrottleFlow::parse_multi_type()
 */
void ThrottleFlow::parse_multi_type()
{
    switch (data[0])
    {
        default:
            state = STATE_NEXT;
            return;
        case ACTION:
        case ADD:
        case REMOVE:
            state = STATE_TRAIN;
            command->data()->commandMultiType = (CommandMultiType)data[0];
            break;
    }

    data.erase(0, 1);
}

/*
 * ThrottleFlow::parse_train()
 */
void ThrottleFlow::parse_train()
{
    size_t end = data.find("<;>");

    switch (end)
    {
        case string::npos:
        case 1:
            if (data[0] == '*')
            {
                break;
            }
        case 0:
            /* invalid string */
            state = STATE_NEXT;
            return;
    }

    command->data()->train.assign(data, 0, end);
    data.erase(0, end + 3);
    state = STATE_SUBCOMMAND;
}

/*
 * ThrottleFlow::parse_subcommand()
 */
bool ThrottleFlow::parse_subcommand()
{
    switch (data[0])
    {
        default:
        case VELOCITY:
        case ESTOP:
        case FUNCTION:
        case FORCE:
        case DIRECTION:
        case RELEASE:
        case DISPATCH:
        case ADDR_SHORT:
        case ADDR_ROSTER:
        case CONSIST:
        case CONSIST_LEAD:
        case IDLE:
        case SS_MODE:
        case MOMENTARY:
        case QUERY:
            state = STATE_NEXT;
            return false;
        case ADDR_LONG:
            state = STATE_PAYLOAD;
            break;
    }

    command->data()->commandSubType = (CommandSubType)data[0];
    data.erase(0, 1);

    size_t end = data.find('\n');

    switch (end)
    {
        case string::npos:
            /* invalid payload */
            state = STATE_NEXT;
            return false;
    }

    command->data()->payload.assign(data, 0, end);
    data.erase(0, end);
    state = STATE_COMMAND;
    return true;
}

} /* namespace withrottle */
