/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file HubDevice.hxx
 * Components for Hubs to connect to physical devices.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_HUBDEVICE_HXX_
#define _UTILS_HUBDEVICE_HXX_

#include <unistd.h>

#include "utils/Hub.hxx"

template <class Data> class FdHubWriteFlow;

class FdHubPortBase : Atomic
{
public:
    static const int kWriteThreadStackSize = 1000;
    static const int kReadThreadStackSize = 1000;
    FdHubPortBase(int fd, Notifiable *done)
        : fd_(fd)
        , writeThread_(fill_thread_name('W', fd), 0, kWriteThreadStackSize)
        , writeService_(&writeThread_)
        , barrier_(done)
        , hasError_(0)
        , writeExitEnqueued_(0)
    {
        barrier_.new_child();
    }

    virtual ~FdHubPortBase()
    {
        writeThread_.shutdown();
    }

protected:
    const char *fill_thread_name(char mode, int fd)
    {
        sprintf(threadName_, "thread_fd_%c_%d", mode, fd);
        return threadName_;
    }

    /** Removes the write flow from the hub's registration. Triggers the write
     * flow to call barrier_ when all the pending queue entries are
     * released. */
    virtual void unregister_write_port() = 0;

    void report_error()
    {
        {
            AtomicHolder h(this);
            if (hasError_)
            {
                return;
            }
            else
            {
                hasError_ = 1;
                ::close(fd_);
            }
        }
        unregister_write_port();
    }

    class ReadThreadBase : public OSThread
    {
    public:
        ReadThreadBase(FdHubPortBase *port) : port_(port)
        {
        }

        /** This is the minimum number of bytes that we will send. */
        virtual int unit() = 0;
        /** We will allocate this many bytes for read buffer. This is the
         * maximum number of bytes that we'll send. */
        virtual int buf_size() = 0;
        /** Sends off a buffer */
        virtual void send_message(const void *buf, int size) = 0;
        void *entry() OVERRIDE
        {
            AutoNotify bn(&port_->barrier_);
            uint8_t buf[buf_size()];
            while (true)
            {
                int done = 0;
                while (done < unit())
                {
                    {
                        AtomicHolder h(port_);
                        if (port_->hasError_)
                        {
                            return NULL;
                        }
                    }
                    ssize_t ret =
                        ::read(port_->fd_, &buf[done], buf_size() - done);
                    if (ret > 0)
                    {
                        done += ret;
                        continue;
                    }
// Now: we have an error.
#ifdef __linux__
                    if (!ret)
                    {
                        LOG(ERROR, "EOF reading fd %d", port_->fd_);
                    }
                    else
                    {
                        LOG(ERROR, "Error reading fd %d: (%d) %s", port_->fd_,
                            errno, strerror(errno));
                    }
#endif
                    port_->report_error();
                    return NULL;
                }
                send_message(buf, done);
            }
        }

    protected:
        FdHubPortBase *port_;
    };

protected:
    template <class Data> friend class FdHubWriteFlow;

    /** Temporary buffer used for rendering thread names. */
    char threadName_[30];
    /** The device file descriptor. */
    int fd_;
    /** This executor is running the writes. */
    Executor<1> writeThread_;
    /** Service for the write flow. */
    Service writeService_;
    /** This barrier will be notified when both read and write thread has
     * exited. */
    BarrierNotifiable barrier_;
    /** If this is 1, the fd has been closed. */
    unsigned hasError_ : 1;
    /** If this is 1, we have already enqueued the request to exit the write
     * flow. */
    unsigned writeExitEnqueued_ : 1;
};

template <class Data>
class FdHubWriteFlow : public StateFlow<Buffer<Data>, QList<1>>
{
public:
    FdHubWriteFlow(FdHubPortBase *parent)
        : StateFlow<Buffer<Data>, QList<1>>(&parent->writeService_)
        , port_(parent)
    {
    }

    StateFlowBase::Action entry() OVERRIDE
    {
        const uint8_t *buf =
            reinterpret_cast<const uint8_t *>(this->message()->data()->data());
        size_t size = this->message()->data()->size();
        while (size > 0)
        {
            {
                AtomicHolder h(port_);
                if (port_->hasError_)
                {
                    if (port_->writeExitEnqueued_ && this->queue_empty())
                    {
                        StateFlowBase::Action a = this->set_terminated();
                        this->release();
                        return a;
                    }
                    return this->release_and_exit();
                }
            }
            ssize_t ret = ::write(port_->fd_, buf, size);
            if (ret > 0)
            {
                size -= ret;
                buf += ret;
                continue;
            }
// now: we have an error.
#ifdef __linux__
            if (!ret)
            {
                LOG(ERROR, "EOF writing fd %d", port_->fd_);
            }
            else
            {
                LOG(ERROR, "Error reading fd %d: (%d) %s", port_->fd_, errno,
                    strerror(errno));
            }
#endif
            port_->report_error();
            break;
        }
        return this->release_and_exit();
    }

    FdHubPortBase *port_;
};

template <class HFlow> class FdHubPort : public FdHubPortBase
{
public:
    FdHubPort(HFlow *hub, int fd, Notifiable *done)
        : FdHubPortBase(fd, done)
        , hub_(hub)
        , writeFlow_(this)
        , readThread_(this)
    {
        hub_->register_port(&writeFlow_);
    }

    void unregister_write_port() OVERRIDE
    {
        hub_->unregister_port(&writeFlow_);
        auto *b = writeFlow_.alloc();
        b->set_done(&barrier_);
        writeFlow_.send(b);
    }

    class ReadThread : public ReadThreadBase
    {
    public:
        ReadThread(FdHubPort<HFlow> *port) : ReadThreadBase(port)
        {
            start(port->fill_thread_name('R', port->fd_), 0,
                  port->kReadThreadStackSize);
        }

        FdHubPort<HFlow> *port()
        {
            return static_cast<FdHubPort<HFlow> *>(port_);
        }

        /** This is the minimum number of bytes that we will send. */
        int unit() OVERRIDE
        {
            return kUnit;
        }
        /** We will allocate this many bytes for read buffer. This is the
         * maximum number of bytes that we'll send. */
        int buf_size() OVERRIDE
        {
            return kBufSize;
        }
        /** Sends off a buffer */
        void send_message(const void *buf, int size) OVERRIDE;

        static const int kUnit;
        static const int kBufSize;
    };

private:
    friend class ReadThread;

    HFlow *hub_;
    FdHubWriteFlow<typename HFlow::value_type> writeFlow_;
    ReadThread readThread_;
};

#endif // _UTILS_HUBDEVICE_HXX_
