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
 * \file main.cxx
 *
 * A simple application to demonstrate asynchronous interfaces.
 *
 * @author Balazs Racz
 * @date 13 Sep 2015
 */

#ifndef _UTILS_JSHUBPORT_HXX_
#define _UTILS_JSHUBPORT_HXX_

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "utils/Hub.hxx"

class JSHubPort : public HubPortInterface
{
public:
    JSHubPort(unsigned long parent, emscripten::val send_fn)
        : parent_(reinterpret_cast<CanHubFlow *>(parent))
        , sendFn_(send_fn)
        , gcHub_(parent_->service())
        , gcAdapter_(
              GCAdapterBase::CreateGridConnectAdapter(&gcHub_, parent_, false))
    {
        HASSERT(sendFn_.typeof().as<std::string>() == "function");
        gcHub_.register_port(this);
    }

    ~JSHubPort()
    {
        gcHub_.unregister_port(this);
    }

    void send(HubPortInterface::message_type *buffer,
        unsigned priority = UINT_MAX) OVERRIDE
    {
        sendFn_((string &)*buffer->data());
        buffer->unref();
    }

    void recv(string s)
    {
        auto *b = gcHub_.alloc();
        b->data()->assign(s);
        b->data()->skipMember_ = this;
        gcHub_.send(b);
    }

private:
    CanHubFlow *parent_;
    emscripten::val sendFn_;
    HubFlow gcHub_;
    std::unique_ptr<GCAdapterBase> gcAdapter_;
};

EMSCRIPTEN_BINDINGS(js_hub_module)
{
    emscripten::class_<JSHubPort>("JSHubPort")
        .constructor<unsigned long, emscripten::val>()
        .function("recv", &JSHubPort::recv);
}

#endif // __EMSCRIPTEN__
#endif // _UTILS_JSHUBPORT_HXX_
