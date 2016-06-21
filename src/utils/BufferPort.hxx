/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file BufferPort.hxx
 *
 * Wrapper for a string-valued Hub port. Uses a time delay to buffer string
 * output up to a certain size before sending off to a target port.
 *
 * @author Balazs Racz
 * @date 20 Jun 2016
 */

#ifndef _UTILS_BUFFERPORT_HXX_
#define _UTILS_BUFFERPORT_HXX_

class BufferPort : public HubPort
{
public:
    BufferPort(Service *service, HubPortInterface *downstream,
        unsigned buffer_bytes, long long delay_nsec)
        : HubPort(service)
        , downstream_(downstream)
        , delayNsec_(delay_nsec)
        , sendBuf_(new char[buffer_bytes])
        , bufSize_(buffer_bytes)
        , bufEnd_(0)
        , timerPending_(0)
    {
        HASSERT(sendBuf_);
    }

    ~BufferPort()
    {
        delete sendBuf_;
    }

private:
    Action entry() override
    {
        if (msg().size() < (bufSize_ - bufEnd_))
        {
            // Fits into the buffer.
            memcpy(sendBuf_ + bufEnd_, msg().data(), msg().size());
            bufEnd_ += msg().size();
            if (!timerPending_)
            {
                timerPending_ = 1;
                bufferTimer_.start(delayNsec_);
            }
            if (!tgtBuf_) {
                // Will ensure we keep track of the skipMember_ inside as well.
                tgtBuf_ = transfer_message();
            }
            return release_and_exit();
        }
        else
        {
            flush_buffer();
        }

        if (msg().size() >= bufSize_)
        {
            // Cannot buffer: send off directly.
            downstream_->send(transfer_message(), priority());
            return exit();
        }
        else
        {
            // After flushing the buffers this will fit.
            return again();
        }
    }

    void flush_buffer()
    {
        if (!bufEnd_) return; // nothing to do
        auto *b = tgtBuf_;
        tgtBuf_ = nullptr;
        b->data()->assign(sendBuf_, bufEnd_);
        bufEnd_ = 0;
        downstream_->send(b);
    }

    void timeout()
    {
        timerPending_ = 0;
        flush_buffer();
    }

    const string &msg()
    {
        return *message()->data();
    }

    class BufferTimer : public ::Timer
    {
    public:
        BufferTimer(BufferPort *parent)
            : Timer(parent->service()->executor()->active_timers())
            , parent_(parent)
        {
        }

        long long timeout() override
        {
            parent_->timeout();
            return NONE;
        }

    private:
        BufferPort *parent_;
    } bufferTimer_{this};

    /// Caches one output buffer to fill in the buffer flush method.
    Buffer<HubData> *tgtBuf_{nullptr};
    /// Where to send output data to.
    HubPortInterface* downstream_;
    /// How long maximum we should buffer the input data.
    long long delayNsec_;
    /// Temporarily stores outgoing data.
    char *sendBuf_;
    /// How many bytes are there in the send buffer.
    unsigned bufSize_;
    /// Offset in sendBuf_ of the first unused byte.
    unsigned bufEnd_ : 24;
    unsigned timerPending_ : 1;
};

#endif // _UTILS_BUFFERPORT_HXX_
