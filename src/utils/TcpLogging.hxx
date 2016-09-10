/** \copyright
 * Copyright (c) 2016, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file TcpLogging.hxx
 *
 * Log target that accepts a TCP connection and prints the log information to
 * the clients.
 *
 * @author Balazs Racz
 * @date 3 Sep 2016
 */

#ifndef _UTILS_TCPLOGGING_HXX_
#define _UTILS_TCPLOGGING_HXX_

#include "executor/StateFlow.hxx"
#include "utils/logging.h"
#include "utils/Singleton.hxx"

/// Buffer content structure for sending log entries.
struct LogEntry
{
    // Log output line. Contains a trailing newline.
    string data;
};

using FdLoggingServerBase = StateFlow<Buffer<LogEntry>, QList<1>>;

class FdLoggingServer : public Singleton<FdLoggingServer>,
                        public FdLoggingServerBase
{
public:
    FdLoggingServer(Service *service)
        : FdLoggingServerBase(service)
    {
    }

private:
    Action entry()
    {
        if (fd_ < 0)
        {
            return release_and_exit();
        }
        return write_repeated(&selectHelper_, fd_, input()->data.data(),
            input()->data.size(), STATE(write_done));
    }

    Action write_done()
    {
        if (selectHelper_.hasError_)
        {
            ::close(fd_);
            fd_ = -1;
        }
        return release_and_exit();
    }

    LogEntry *input()
    {
        return message()->data();
    }

protected:
    int fd_{-1};

private:
    StateFlowSelectHelper selectHelper_{this};
};

void log_output(char *buf, int size)
{
    if (size <= 0)
        return;
    if (!Singleton<FdLoggingServer>::exists())
        return;
    auto *b = Singleton<FdLoggingServer>::instance()->alloc();
    b->data()->data.reserve(size + 1);
    b->data()->data.assign(buf, size);
    b->data()->data.push_back('\n');
    Singleton<FdLoggingServer>::instance()->send(b);
}

class TcpLoggingServer : public FdLoggingServer
{
public:
    TcpLoggingServer(Service *service, int port)
        : FdLoggingServer(service)
        , listener_(port, std::bind(&TcpLoggingServer::on_new_connection, this,
                              std::placeholders::_1))
    {
    }

private:
    void on_new_connection(int fd)
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
        fd_ = fd;
    }

    SocketListener listener_;
};

class SerialLoggingServer : public FdLoggingServer
{
public:
    SerialLoggingServer(Service *service, const char *name)
        : FdLoggingServer(service)
    {
        fd_ = ::open(name, O_WRONLY);
        HASSERT(fd_ >= 0);
    }
};

#endif // _UTILS_TCPLOGGING_HXX_
