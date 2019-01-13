/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file OpenMRN.h
 * 
 * Main include file for the OpenMRN library to be used in an Arduino
 * compilation environment.
 *
 * @author Balazs Racz
 * @date 24 July 2018
 */

#ifndef _ARDUINO_OPENMRN_H_
#define _ARDUINO_OPENMRN_H_

#include "freertos_drivers/common/Can.hxx"
#include "openlcb/SimpleStack.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/Uninitialized.hxx"

/// Bridge class that connects an Arduino API style serial port to the OpenMRN
/// core stack.
template <class SerialType> class SerialBridge : public Executable
{
public:
    SerialBridge(SerialType *port, CanHubFlow *can_hub)
        : service_(can_hub->service())
        , port_(port)
    {
        GCAdapterBase::CreateGridConnectAdapter(&txtHub_, can_hub, false);
        txtHub_.register_port(&writePort_);
    }

    /// Called by the loop.
    void run() override
    {
        loop_for_write();
        loop_for_read();
    }

private:
    /// Access to the stack's executor.
    Service *service_;

    void loop_for_write()
    {
        if (!writeBuffer_)
            return;
        size_t len = port_->availableForWrite();
        if (!len)
            return;
        size_t to_write = writeBuffer_->data()->size() - writeOfs_;
        if (len > to_write)
            len = to_write;
        port_->write(writeBuffer_->data()->data() + writeOfs_, len);
        writeOfs_ += len;
        if (writeOfs_ >= writeBuffer_->data()->size())
        {
            writeBuffer_ = nullptr;
            // wakes up state flow to release buffer and take next from the
            // queue.
            writePort_.notify();
        }
    }

    void loop_for_read()
    {
        int av = port_->available();
        if (av <= 0)
            return;
        // @todo build a buffer here and send to the hub
    }

    friend class WritePort;
    class WritePort : public HubPort
    {
    public:
        WritePort(SerialBridge *parent, Service *service)
            : HubPort(service)
            , parent_(parent)
        {
        }

        Action entry() override
        {
            parent_->writeBuffer_ = message();
            parent_->writeOfs_ = 0;
            return wait_and_call(STATE(write_done));
        }

        Action write_done()
        {
            return release_and_exit();
        }

    private:
        SerialBridge *parent_;
    } writePort_{this, service_};

    /// Arduino device instance.
    SerialType *port_;
    /// Buffer we are writing the output from right now.
    Buffer<HubData> *writeBuffer_{nullptr};
    /// Offset in the output string of the next byte to write.
    size_t writeOfs_;
    /// Hub for the textual data.
    HubFlow txtHub_{service_};
};

class CanBridge : public Executable
{
public:
    CanBridge(Can *port, CanHubFlow *can_hub)
        : canHub_(can_hub)
        , port_(port)
    {
        port_->enable();
        can_hub->register_port(&writePort_);
    }

    ~CanBridge()
    {
        port_->disable();
    }

    /// Called by the loop.
    void run() override
    {
        loop_for_write();
        loop_for_read();
    }

private:
    void loop_for_write()
    {
        if (!writeBuffer_)
            return;
        if (port_->availableForWrite() <= 0)
            return;
        port_->write(writeBuffer_->data());
        writeBuffer_ = nullptr;
        writePort_.notify();
    }

    void loop_for_read()
    {
        if (!port_->available())
            return;
        auto *b = canHub_->alloc();
        port_->read(b->data());
        b->data()->skipMember_ = &writePort_;
        canHub_->send(b);
    }

    friend class WritePort;
    class WritePort : public CanHubPort
    {
    public:
        WritePort(CanBridge *parent)
            : CanHubPort(parent->canHub_->service())
            , parent_(parent)
        {
        }

        Action entry() override
        {
            parent_->writeBuffer_ = message();
            return wait_and_call(STATE(write_done));
        }

        Action write_done()
        {
            return release_and_exit();
        }

    private:
        CanBridge *parent_;
    };

    /// Hardware driver.
    Can *port_;
    /// Next buffer we are trying to write into the driver's FIFO.
    Buffer<CanHubData> *writeBuffer_{nullptr};
    /// Connection to the stack.
    CanHubFlow *canHub_;
    /// State flow with queues for output frames generated by the stack.
    WritePort writePort_{this};
};

class OpenMRN : private Executable
{
public:
    OpenMRN()
    {
    }
    /// Use this constructor if stack() needs to be accessed during the time of
    /// the static construction. Then use init() with no arguments.
    OpenMRN(openlcb::NodeID node_id)
    {
        init(node_id);
    }

    /// Call this function once if the empty constructor was used.
    void init(openlcb::NodeID node_id)
    {
        stack_.emplace(node_id);
        stack_->start_stack(false);
    }

    /// Call this function once if the constructor with Node ID was used.
    void init()
    {
        stack_->start_stack(false);
    }

    /// @return pointer to the stack. Do not call before init().
    openlcb::SimpleCanStack *stack()
    {
        return stack_.operator->();
    }

    /// Call this function from the loop() function of the Arduino sketch.
    void loop()
    {
        for (auto *e : loopMembers_)
        {
            e->run();
        }
    }

    template <class SerialType> void add_gridconnect_port(SerialType *port)
    {
        loopMembers_.push_back(
            new SerialBridge<SerialType>(port, stack()->can_hub()));
    }

    void add_can_port(Can *port)
    {
        loopMembers_.push_back(new CanBridge(port, stack()->can_hub()));
    }

private:
    void run() override
    {
        stack_->executor()->loop_once();
    }

    /// Storage space for the OpenLCB stack. Will be constructed in init().
    uninitialized<openlcb::SimpleCanStack> stack_;
    /// List of objects we need to call in each loop iteration.
    vector<Executable *> loopMembers_{{this}};
};

#endif // _ARDUINO_OPENMRN_H_
