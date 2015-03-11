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
 * \file SimpleStack.hxx
 *
 * A complete OpenLCB stack for use in straightforward OpenLCB nodes.
 *
 * @author Balazs Racz
 * @date 10 Mar 2015
 */

#ifndef _NMRANET_SIMPLESTACK_HXX_
#define _NMRANET_SIMPLESTACK_HXX_

#include "executor/Executor.hxx"
#include "nmranet/AliasAllocator.hxx"
#include "nmranet/DefaultNode.hxx"
#include "nmranet/EventService.hxx"
#include "nmranet/IfCan.hxx"
#include "nmranet/ProtocolIdentification.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/HubDevice.hxx"
#include "utils/HubDeviceNonBlock.hxx"
#include "utils/GridConnectHub.hxx"

namespace nmranet
{

class SimpleCanStack
{
public:
    static const unsigned EXECUTOR_PRIORITIES = 5;

    SimpleCanStack(const nmranet::NodeID node_id) : node_(&ifCan_, node_id)
    {
        AddAliasAllocator(node_id, &ifCan_);
    }

    Executor<EXECUTOR_PRIORITIES> *executor()
    {
        return &executor_;
    }

    Service *service()
    {
        return &service_;
    }

    IfCan *interface()
    {
        return &ifCan_;
    }

    Node *node()
    {
        return &node_;
    }

    CanHubFlow *can_hub()
    {
        return &canHub0_;
    }

    /** Adds a CAN bus port with synchronous driver API. */
    void add_can_port_blocking(const char *device)
    {
        int can_fd = ::open(device, O_RDWR);
        HASSERT(can_fd >= 0);
        auto *port = new FdHubPort<CanHubFlow>(
            &canHub0_, can_fd, EmptyNotifiable::DefaultInstance());
        additionalComponents_.emplace_back(port);
    }

#ifdef __FreeRTOS__
    /** Adds a CAN bus port with asynchronous driver API. */
    void add_can_port_async(const char *device)
    {
        auto* port = new HubDeviceNonBlock<CanHubFlow>(&canHub0_, device);
        additionalComponents_.emplace_back(port);
    }
#endif

    /** Adds a gridconnect port to the CAN bus. */
    void add_gridconnect_port(const char* device) {
        int fd = ::open(device, O_RDWR);
        HASSERT(fd >= 0);
        create_gc_port_for_can_hub(&canHub0_, fd);
    }

    /** Starts a TCP server on the specified port in listening mode. Each
     * incoming connection will be assumed to be in gridconnect protocol and
     * will be added to the gridconnect hub. */
    void start_tcp_hub_server(int port)
    {
        /// @TODO (balazs.racz) make this more efficient by rendering to string
        /// only once for all connections.
        /// @TODO (balazs.racz) do not leak this.
        new GcTcpHub(&canHub0_, port);
    }

    /** Causes all CAN packets to be printed to stdout. */
    void print_all_packets()
    {
        auto *port = new DisplayPort(&service_);
        gridconnect_hub()->register_port(port);
        additionalComponents_.emplace_back(port);
    }

    /** Returns the hub to be used for gridconnect-format CANbus. You can
     * inject text CAN packets to this hub, add printers and in general connect
     * devices and sockets using the gridconnect protocol to talk CANbus.
     *
     * The actual gridconnect parser / renderer objects will be created upon
     * the first call to this function. */
    HubFlow *gridconnect_hub()
    {
        if (!gcHub_)
        {
            gcHub_.reset(new HubFlow(&service_));
            gcAdapter_.reset(GCAdapterBase::CreateGridConnectAdapter(
                gcHub_.get(), &canHub0_, false));
        }
        return gcHub_.get();
    }

    /** Donates the current thread to the executor. Never returns. */
    void loop_executor()
    {
        start_stack();
        executor_.thread_body();
    }

    /** Instructs the executor to create a new thread and run in there. */
    void start_executor_thread(const char *name, int priority,
                               size_t stack_size)
    {
        start_stack();
        executor_.start_thread(name, priority, stack_size);
    }

private:
    static const auto PIP_RESPONSE = Defs::EVENT_EXCHANGE;

    /** Call this function once after the actual IO ports are set up. Calling
     * before the executor starts looping is okay. */
    void start_stack() {
        // Bootstraps the alias allocation process.
        ifCan_.alias_allocator()->send(ifCan_.alias_allocator()->alloc());
    }

    /// This executor's threads will be handled
    Executor<EXECUTOR_PRIORITIES> executor_{NO_THREAD()};
    /// Default service on the particular executor.
    Service service_{&executor_};
    /// Abstract CAN bus in-memory.
    CanHubFlow canHub0_{&service_};
    /** NMRAnet interface for sending and receiving messages, formatting them
     * to the CAN bus port and maintaining the conversion flows, caches etc. */
    IfCan ifCan_{
        &executor_,                      &canHub0_,
        config_local_alias_cache_size(), config_remote_alias_cache_size(),
        config_local_nodes_count()};
    /// The actual node.
    DefaultNode node_;
    /// Dispatches event protocol requests to the event handlers.
    EventService eventService_{&ifCan_};
    /// Handles PIP requests.
    ProtocolIdentificationHandler pipHandler_{&node_, PIP_RESPONSE};
    /** All packets are forwarded to this hub in gridconnect format, if
     * needed. Will be initialized upon first use. */
    std::unique_ptr<HubFlow> gcHub_;
    /** Bridge between canHub_ and gcHub_. Lazily initialized. */
    std::unique_ptr<GCAdapterBase> gcAdapter_;

    /// Stores and keeps ownership of optional components.
    std::vector<std::unique_ptr<Destructable>> additionalComponents_;
};

} // namespace nmranet

#endif //  _NMRANET_SIMPLESTACK_HXX_
