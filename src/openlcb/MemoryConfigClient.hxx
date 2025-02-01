/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file MemoryConfigClient.hxx
 *
 * Utility flow to talk to a remote device using the memconfig protocol.
 *
 * @author Balazs Racz
 * @date 4 Feb 2017
 */

#ifndef _OPENLCB_MEMORYCONFIGCLIENT_HXX_
#define _OPENLCB_MEMORYCONFIGCLIENT_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/DatagramHandlerDefault.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/StreamReceiver.hxx"
#include "openlcb/StreamTransport.hxx"

namespace openlcb
{

struct MemoryConfigClientRequest : public CallableFlowRequestBase
{
    enum ReadCmd
    {
        READ
    };

    enum ReadStreamCmd
    {
        READ_STREAM
    };

    enum ReadPartCmd
    {
        READ_PART
    };

    enum ReadPartStreamCmd
    {
        READ_PART_STREAM
    };

    enum WriteCmd
    {
        WRITE
    };

    enum UpdateCompleteCmd
    {
        UPDATE_COMPLETE
    };

    enum RebootCmd
    {
        REBOOT
    };

    enum FactoryResetCmd
    {
        FACTORY_RESET
    };

    enum FreezeCmd
    {
        FREEZE
    };

    enum UnfreezeCmd
    {
        UNFREEZE
    };

    /// Sets up a command to read an entire memory space.
    /// @param ReadCmd polymorphic matching arg; always set to READ.
    /// @param d is the destination node to query
    /// @param space is the memory space to read out
    /// @param cb if specified, will be called inline multiple times during the
    /// processing as more data arrives.
    void reset(ReadCmd, NodeHandle d, uint8_t space,
        std::function<void(MemoryConfigClientRequest *)> cb = nullptr)
    {
        reset_base();
        cmd = CMD_READ;
        memory_space = space;
        dst = d;
        address = 0;
        size = 0xffffffffu;
        payload.clear();
        progressCb = std::move(cb);
    }

    /// Sets up a command to read an entire memory space using stream transport.
    /// @param ReadStreamCmd polymorphic matching arg; always set to READ.
    /// @param d is the destination node to query
    /// @param space is the memory space to read out
    /// @param cb if specified, will be called inline multiple times during the
    /// processing as more data arrives.
    void reset(ReadStreamCmd, NodeHandle d, uint8_t space,
        std::function<void(MemoryConfigClientRequest *)> cb = nullptr)
    {
        reset(READ, d, space, std::move(cb));
        use_stream = true;
    }

    /// Sets up a command to read a part of a memory space.
    /// @param ReadPartCmd polymorphic matching arg; always set to READ_PART.
    /// @param d is the destination node to query
    /// @param space is the memory space to read out
    /// @param offset if the address of the first byte to read
    /// @param size is the number of bytes to read
    void reset(ReadPartCmd, NodeHandle d, uint8_t space, unsigned offset,
        unsigned size)
    {
        reset_base();
        cmd = CMD_READ_PART;
        memory_space = space;
        dst = d;
        this->address = offset;
        this->size = size;
        payload.clear();
    }

    /// Sets up a command to read a part of a memory space using stream
    /// transport.
    /// @param ReadPartStreamCmd polymorphic matching arg; always set to
    /// READ_PART.
    /// @param d is the destination node to query
    /// @param space is the memory space to read out
    /// @param offset if the address of the first byte to read
    /// @param size is the number of bytes to read
    void reset(ReadPartStreamCmd, NodeHandle d, uint8_t space, unsigned offset,
        unsigned size)
    {
        reset(READ_PART, d, space, offset, size);
        use_stream = true;
    }

    /// Sets up a command to write a part of a memory space.
    /// @param WriteCmd polymorphic matching arg; always set to WRITE.
    /// @param d is the destination node to write to
    /// @param space is the memory space to write to
    /// @param offset if the address of the first byte to write
    /// @param data is the data to write
    void reset(
        WriteCmd, NodeHandle d, uint8_t space, unsigned offset, string data)
    {
        reset_base();
        cmd = CMD_WRITE;
        memory_space = space;
        dst = d;
        this->address = offset;
        this->size = data.size();
        payload = std::move(data);
    }

    /// Sets up a command to send an Update Complete request to a remote node.
    /// @param UpdateCompleteCmd polymorphic matching arg; always set to
    /// UPDATE_COMPLETE.
    /// @param d is the destination node
    void reset(UpdateCompleteCmd, NodeHandle d)
    {
        reset_base();
        cmd = CMD_META_REQUEST;
        dst = d;
        payload.clear();
        payload.reserve(2);
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_UPDATE_COMPLETE);
    }

    /// Sets up a command to send a Reboot request to a remote node.
    /// @param RebootCmd polymorphic matching arg; always set to
    /// REBOOT.
    /// @param d is the destination node
    void reset(RebootCmd, NodeHandle d)
    {
        reset_base();
        cmd = CMD_META_REQUEST;
        dst = d;
        payload.clear();
        payload.reserve(2);
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_RESET);
    }

    /// Sets up a command to send a Factory Reset request to a remote node.
    /// @param FactoryResetCmd polymorphic matching arg; always set to
    /// FACTORY_RESET.
    /// @param d is the destination node
    void reset(FactoryResetCmd, NodeHandle d)
    {
        reset_base();
        cmd = CMD_FACTORY_RESET;
        dst = d;
        payload.clear();
        payload.reserve(8);
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_FACTORY_RESET);
    }

    /// Sets up a command to send a Freeze request to a remote node.
    /// @param FreezeCmd polymorphic matching arg; always set to
    /// FREEZE.
    /// @param d is the destination node
    /// @param space is the memry space to freeze.
    void reset(FreezeCmd, NodeHandle d, uint8_t space)
    {
        reset_base();
        cmd = CMD_META_REQUEST;
        dst = d;
        payload.clear();
        payload.reserve(3);
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_FREEZE);
        payload.push_back(space);
    }

    /// Sets up a command to send a Unfreeze request to a remote node.
    /// @param UnfreezeCmd polymorphic matching arg; always set to
    /// UNFREEZE.
    /// @param d is the destination node
    /// @param space is the memry space to unfreeze.
    void reset(UnfreezeCmd, NodeHandle d, uint8_t space)
    {
        reset_base();
        cmd = CMD_META_REQUEST;
        dst = d;
        payload.clear();
        payload.reserve(3);
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_UNFREEZE);
        payload.push_back(space);
    }

    enum Command : uint8_t
    {
        CMD_READ,
        CMD_READ_PART,
        CMD_WRITE,
        CMD_META_REQUEST,
        CMD_FACTORY_RESET
    };

    /// Helper function invoked at every other reset call.
    void reset_base()
    {
        CallableFlowRequestBase::reset_base();
        progressCb = nullptr;
        payload.clear();
        size = 0;
        address = 0;
        use_stream = false;
    }

    Command cmd;
    uint8_t memory_space;
    bool use_stream;
    unsigned address;
    unsigned size;
    /// Node to send the request to.
    NodeHandle dst;
    string payload;
    /// Callback to execute as progress is being made.
    std::function<void(MemoryConfigClientRequest *)> progressCb;
};

class MemoryConfigClient : public CallableFlow<MemoryConfigClientRequest>
{
public:
    MemoryConfigClient(Node *node, MemoryConfigHandler *memcfg)
        : CallableFlow<MemoryConfigClientRequest>(memcfg->dg_service())
        , node_(node)
        , memoryConfigHandler_(memcfg)
    { }

    /// These result codes are written into request()->resultCode during and as
    /// a return from the flow.
    enum ResultCodes
    {
        // Internal error codes generated by the send flow
        OPERATION_PENDING =
            DatagramClient::OPERATION_PENDING, ///< cleared when done is called.
        TIMEOUT = Defs::ERROR_PERMANENT |
            Defs::OPENMRN_TIMEOUT, ///< Timeout waiting for ack/nack.
    };

    /// @return OpenLCB node (ourselves) for outgoing communications.
    Node *node()
    {
        return node_;
    }

    /// @return OpenLCB memory config handler (which was given in the
    /// constructor).
    MemoryConfigHandler *mem_cfg()
    {
        return memoryConfigHandler_;
    }

protected:
    Action entry() override
    {
        request()->resultCode = OPERATION_PENDING;
        switch (request()->cmd)
        {
            case MemoryConfigClientRequest::CMD_READ:
            case MemoryConfigClientRequest::CMD_READ_PART:
                return allocate_and_call(
                    STATE(do_read), dg_service()->client_allocator());
            case MemoryConfigClientRequest::CMD_WRITE:
                return allocate_and_call(
                    STATE(do_write), dg_service()->client_allocator());
            case MemoryConfigClientRequest::CMD_META_REQUEST:
                return allocate_and_call(
                    STATE(do_meta_request), dg_service()->client_allocator());
            case MemoryConfigClientRequest::CMD_FACTORY_RESET:
                return call_immediately(STATE(prepare_factory_reset));
            default:
                break;
        }
        return return_with_error(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
    }

private:
    Action do_read()
    {
        dgClient_ = full_allocation_result(dg_service()->client_allocator());
        offset_ = request()->address;
        memoryConfigHandler_->set_client(&responseFlow_);
        return call_immediately(STATE(send_next_read));
    }

    Action send_next_read()
    {
        return allocate_and_call(
            dg_service()->iface()->dispatcher(), STATE(send_read_datagram));
    }

    Action send_read_datagram()
    {
        auto *b = get_allocation_result(dg_service()->iface()->dispatcher());
        b->set_done(bn_.reset(this));
        unsigned sz = request()->size > 64 ? 64 : request()->size;
        b->data()->reset(Defs::MTI_DATAGRAM, node_->node_id(), request()->dst,
            MemoryConfigDefs::read_datagram(
                request()->memory_space, offset_, sz));
        if (request()->size < 0xffffffffu)
        {
            request()->size -= sz;
        }
        isWaitingForTimer_ = 0;
        responseCode_ = DatagramClient::OPERATION_PENDING;
        dgClient_->write_datagram(b);
        return wait_and_call(STATE(read_complete));
    }

    Action read_complete()
    {
        if (!(dgClient_->result() & DatagramClient::OPERATION_SUCCESS))
        {
            // some error occurred.
            return handle_read_error(dgClient_->result());
        }
        if (responseCode_ & DatagramClient::OPERATION_PENDING)
        {
            isWaitingForTimer_ = 1;
            // Extract the timeout.
            long long timeout = DatagramDefs::timeout_from_flags_nsec(
                dgClient_->result() >> DatagramClient::RESPONSE_FLAGS_SHIFT);
            return sleep_and_call(
                &timer_, timeout, STATE(read_response_timeout));
        }
        else
        {
            return call_immediately(STATE(read_response_timeout));
        }
    }

    Action read_response_timeout()
    {
        if (responseCode_ & DatagramClient::OPERATION_PENDING ||
            (isWaitingForTimer_ && !timer_.is_triggered()))
        {
            return handle_read_error(Defs::OPENMRN_TIMEOUT);
        }
        size_t len = responsePayload_.size();
        const uint8_t *bytes =
            MemoryConfigDefs::payload_bytes(responsePayload_);
        if (!MemoryConfigDefs::payload_min_length_check(responsePayload_, 0))
        {
            LOG(INFO,
                "Memory Config client: response datagram payload not "
                "long enough");
            return handle_read_error(
                Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        unsigned ofs = MemoryConfigDefs::get_payload_offset(responsePayload_);
        unsigned address = MemoryConfigDefs::get_address(responsePayload_);
        uint8_t space = MemoryConfigDefs::get_space(responsePayload_);
        uint8_t cmd = bytes[1] & MemoryConfigDefs::COMMAND_MASK;
        if (address != offset_)
        {
            return handle_read_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (space != request()->memory_space)
        {
            return handle_read_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (cmd == MemoryConfigDefs::COMMAND_READ_FAILED)
        {
            if (len < ofs + 2)
            {
                return handle_read_error(
                    Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
            }
            uint16_t error = bytes[ofs++];
            error <<= 8;
            error |= bytes[ofs];
            return handle_read_error(error);
        }
        if (cmd != MemoryConfigDefs::COMMAND_READ_REPLY)
        {
            return handle_read_error(Defs::ERROR_UNIMPLEMENTED);
        }
        unsigned dlen = len - ofs;
        request()->payload.append((char *)(bytes + ofs), dlen);
        offset_ += dlen;
        if (request()->progressCb)
        {
            request()->progressCb(request());
        }
        if ((dlen < 64) || (request()->size == 0))
        {
            return call_immediately(STATE(finish_read));
        }
        return call_immediately(STATE(send_next_read));
    }

protected:
    Action handle_read_error(int error)
    {
        if (error == MemoryConfigDefs::ERROR_OUT_OF_BOUNDS)
        {
            return finish_read();
        }
        cleanup_read();
        return return_with_error(error);
    }

    void cleanup_read()
    {
        responsePayload_.clear();
        dg_service()->client_allocator()->typed_insert(dgClient_);
        memoryConfigHandler_->clear_client(&responseFlow_);
        dgClient_ = nullptr;
    }

    Action finish_read()
    {
        cleanup_read();
        return return_ok();
    }

private:    
    Action do_write()
    {
        dgClient_ = full_allocation_result(dg_service()->client_allocator());
        offset_ = request()->address;
        payloadOffset_ = 0;
        memoryConfigHandler_->set_client(&responseFlow_);
        return call_immediately(STATE(send_next_write));
    }

    Action send_next_write()
    {
        return allocate_and_call(
            dg_service()->iface()->dispatcher(), STATE(send_write_datagram));
    }

    Action send_write_datagram()
    {
        auto *b = get_allocation_result(dg_service()->iface()->dispatcher());
        b->set_done(bn_.reset(this));
        unsigned sz = request()->payload.size() - payloadOffset_;
        if (sz > MemoryConfigDefs::MAX_DATAGRAM_RW_BYTES)
        {
            sz = MemoryConfigDefs::MAX_DATAGRAM_RW_BYTES;
        }
        writeLength_ = sz;
        b->data()->reset(Defs::MTI_DATAGRAM, node_->node_id(), request()->dst,
            MemoryConfigDefs::write_datagram(request()->memory_space, offset_,
                request()->payload.substr(payloadOffset_, sz)));
        isWaitingForTimer_ = 0;
        responseCode_ = DatagramClient::OPERATION_PENDING;
        dgClient_->write_datagram(b);
        return wait_and_call(STATE(write_complete));
    }

    Action write_complete()
    {
        if (!(dgClient_->result() & DatagramClient::OPERATION_SUCCESS))
        {
            // some error occurred.
            return handle_write_error(dgClient_->result());
        }
        if (!(dgClient_->result() & DatagramClient::OK_REPLY_PENDING))
        {
            // Received an immediate received okay with no pending reply.
            return call_immediately(STATE(finish_write));
        }
        else if (responseCode_ & DatagramClient::OPERATION_PENDING)
        {
            isWaitingForTimer_ = 1;
            // Extract the timeout.
            long long timeout = DatagramDefs::timeout_from_flags_nsec(
                dgClient_->result() >> DatagramClient::RESPONSE_FLAGS_SHIFT);
            return sleep_and_call(
                &timer_, timeout, STATE(write_response_timeout));
        }
        else
        {
            return call_immediately(STATE(write_response_timeout));
        }
    }

    Action write_response_timeout()
    {
        if (responseCode_ & DatagramClient::OPERATION_PENDING ||
            (isWaitingForTimer_ && !timer_.is_triggered()))
        {
            return handle_write_error(Defs::OPENMRN_TIMEOUT);
        }
        size_t len = responsePayload_.size();
        const uint8_t *bytes =
            MemoryConfigDefs::payload_bytes(responsePayload_);
        if (!MemoryConfigDefs::payload_min_length_check(responsePayload_, 0))
        {
            LOG(INFO,
                "Memory Config client: response datagram payload not "
                "long enough");
            return handle_write_error(
                Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        unsigned ofs = MemoryConfigDefs::get_payload_offset(responsePayload_);
        unsigned address = MemoryConfigDefs::get_address(responsePayload_);
        uint8_t space = MemoryConfigDefs::get_space(responsePayload_);
        uint8_t cmd = bytes[1] & MemoryConfigDefs::COMMAND_MASK;
        if (address != offset_)
        {
            return handle_write_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (space != request()->memory_space)
        {
            return handle_write_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (cmd == MemoryConfigDefs::COMMAND_WRITE_FAILED)
        {
            if (len < ofs + 2)
            {
                return handle_write_error(
                    Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
            }
            uint16_t error = bytes[ofs++];
            error <<= 8;
            error |= bytes[ofs];
            return handle_write_error(error);
        }
        if (cmd != MemoryConfigDefs::COMMAND_WRITE_REPLY)
        {
            return handle_write_error(Defs::ERROR_UNIMPLEMENTED);
        }
        // write success.
        offset_ += writeLength_;
        payloadOffset_ += writeLength_;
        if (payloadOffset_ >= request()->payload.size())
        {
            return call_immediately(STATE(finish_write));
        }
        return call_immediately(STATE(send_next_write));
    }

    Action handle_write_error(int error)
    {
        if (error == MemoryConfigDefs::ERROR_OUT_OF_BOUNDS)
        {
            return finish_write();
        }
        cleanup_write();
        return return_with_error(error);
    }

    void cleanup_write()
    {
        responsePayload_.clear();
        dg_service()->client_allocator()->typed_insert(dgClient_);
        memoryConfigHandler_->clear_client(&responseFlow_);
        dgClient_ = nullptr;
    }

    Action finish_write()
    {
        cleanup_write();
        return return_ok();
    }

    Action do_meta_request()
    {
        dgClient_ = full_allocation_result(dg_service()->client_allocator());
        // Meta requests do not have a response, so we are not registering the
        // response flow here.
        return allocate_and_call(
            dg_service()->iface()->dispatcher(), STATE(send_meta_datagram));
    }

    Action send_meta_datagram()
    {
        auto *b = get_allocation_result(dg_service()->iface()->dispatcher());
        b->set_done(bn_.reset(this));
        b->data()->reset(Defs::MTI_DATAGRAM, node_->node_id(), request()->dst,
            std::move(request()->payload));
        isWaitingForTimer_ = 0;
        dgClient_->write_datagram(b);
        return wait_and_call(STATE(meta_complete));
    }

    Action meta_complete()
    {
        auto result = dgClient_->result();
        dg_service()->client_allocator()->typed_insert(dgClient_);
        dgClient_ = nullptr;
        result &= DatagramClient::RESPONSE_CODE_MASK;
        if (result == DatagramClient::DST_REBOOT ||
            result == DatagramClient::OPERATION_SUCCESS)
        {
            return return_ok();
        }
        else
        {
            return return_with_error(result);
        }
    }

    /// Before we send out a factory reset command, we have to ensure that we
    /// know the target node's node ID, not just the alias.
    Action prepare_factory_reset()
    {
        node()->iface()->canonicalize_handle(&request()->dst);
        if (request()->dst.id)
        {
            return call_immediately(STATE(factory_reset_have_id));
        }
        // Now: we have a dst with alias only, so we must be running on CAN-bus.
        IfCan *iface = (IfCan *)node()->iface();
        if (!nodeIdlookupFlow_)
        {
            // This is so rarely used that we rather allocate it dynamically.
            nodeIdlookupFlow_.reset(new NodeIdLookupFlow(iface));
        }
        return invoke_subflow_and_wait(nodeIdlookupFlow_.get(),
            STATE(dst_id_complete), node(), request()->dst);
    }

    /// Completed the ID lookup flow.
    Action dst_id_complete()
    {
        auto rb =
            get_buffer_deleter(full_allocation_result(nodeIdlookupFlow_.get()));
        request()->dst = rb->data()->handle;
        // Object not needed anymore.
        nodeIdlookupFlow_.reset();
        if (request()->dst.id)
        {
            return call_immediately(STATE(factory_reset_have_id));
        }
        return return_with_error(Defs::ERROR_OPENMRN_NOT_FOUND);
    }

    /// Called from different places to do the factory reset request once we
    /// have the node ID filled in the dst handle.
    Action factory_reset_have_id()
    {
        request()->payload.resize(8);
        node_id_to_data(request()->dst.id, &request()->payload[2]);
        return allocate_and_call(
            STATE(do_meta_request), dg_service()->client_allocator());
    }

    class ResponseFlow : public DefaultDatagramHandler
    {
    public:
        ResponseFlow(MemoryConfigClient *parent)
            : DefaultDatagramHandler(parent->memoryConfigHandler_->dg_service())
            , parent_(parent)
        { }

    private:
        Action entry() override
        {
            if (!parent_->has_request())
            {
                return respond_reject(Defs::ERROR_OUT_OF_ORDER);
            }
            if (!parent_->node_->iface()->matching_node(
                    parent_->request()->dst, message()->data()->src))
            {
                return respond_reject(Defs::ERROR_OUT_OF_ORDER);
            }
            if (size() < 2)
            {
                return respond_reject(
                    Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
            }
            auto *bytes = payload();
            uint8_t cmd = bytes[1] & ~3;
            switch (cmd)
            {
                case MemoryConfigDefs::COMMAND_READ_REPLY:
                case MemoryConfigDefs::COMMAND_READ_FAILED:
                {
                    if (parent_->request()->cmd !=
                            MemoryConfigClientRequest::CMD_READ &&
                        parent_->request()->cmd !=
                            MemoryConfigClientRequest::CMD_READ_PART)
                    {
                        break;
                    }
                    parent_->responseCode_ = 0;
                    message()->data()->payload.swap(parent_->responsePayload_);
                    if (parent_->isWaitingForTimer_)
                    {
                        parent_->timer_.trigger();
                    }
                    return respond_ok(0);
                }
                case MemoryConfigDefs::COMMAND_READ_STREAM_REPLY:
                case MemoryConfigDefs::COMMAND_READ_STREAM_FAILED:
                {
                    if (parent_->request()->cmd !=
                            MemoryConfigClientRequest::CMD_READ &&
                        parent_->request()->cmd !=
                            MemoryConfigClientRequest::CMD_READ_PART)
                    {
                        break;
                    }
                    if (!parent_->request()->use_stream)
                    {
                        break;
                    }
                    parent_->responseCode_ = 0;
                    message()->data()->payload.swap(parent_->responsePayload_);
                    if (parent_->isWaitingForTimer_)
                    {
                        parent_->timer_.trigger();
                    }
                    return respond_ok(0);
                }
                case MemoryConfigDefs::COMMAND_WRITE_REPLY:
                case MemoryConfigDefs::COMMAND_WRITE_FAILED:
                    if (parent_->request()->cmd !=
                        MemoryConfigClientRequest::CMD_WRITE)
                    {
                        break;
                    }
                    parent_->responseCode_ = 0;
                    message()->data()->payload.swap(parent_->responsePayload_);
                    if (parent_->isWaitingForTimer_)
                    {
                        parent_->timer_.trigger();
                    }
                    return respond_ok(0);
            }
            return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
        }

    private:
        MemoryConfigClient *parent_;
    };

protected:
    DatagramService *dg_service()
    {
        return static_cast<DatagramService *>(service());
    }

    /// Node from which to send the requests out.
    Node *node_;
    /// Hook into the parent node's memory config handler service.
    MemoryConfigHandler *memoryConfigHandler_;
    /// The allocated datagram client which we hold on for the time that we are
    /// querying.
    DatagramClient *dgClient_ {nullptr};
    /// Handler for the incoming reply datagrams.
    ResponseFlow responseFlow_ {this};
    /// Notify helper.
    BarrierNotifiable bn_;
    /// Rarely used helper flow to look up full node IDs from aliases.
    std::unique_ptr<openlcb::NodeIdLookupFlow> nodeIdlookupFlow_;
    /// Next byte to read from the memory space.
    uint32_t offset_;
    /// Next byte in the payload to write.
    uint32_t payloadOffset_;
    /// How many bytes we wrote in this datagram.
    uint16_t writeLength_;
    /// timing helper
    StateFlowTimer timer_ {this};
    /// The data that came back from reading.
    string responsePayload_;
    /// error code that came with the response. 0 for success.
    int responseCode_;
    /// 1 if we are pending on the timer.
    uint8_t isWaitingForTimer_ : 1;
}; // class MemoryConfigClient

class MemoryConfigClientWithStream : public MemoryConfigClient
{
public:
    MemoryConfigClientWithStream(
        Node *node, MemoryConfigHandler *memcfg, uint8_t local_stream_id)
        : MemoryConfigClient(node, memcfg)
    {
        /// @todo make this not specific to the CAN-buf interface but somehow
        /// portable.
        IfCan *iface = static_cast<IfCan *>(node_->iface());
        HASSERT(iface);
        HASSERT(iface->stream_transport());
        dstStreamId_ = iface->stream_transport()->get_next_stream_receive_id();
        receiver_.reset(new StreamReceiverCan(iface, dstStreamId_));
    }

protected:
    Action entry() override
    {
        if (!request()->use_stream)
        {
            return MemoryConfigClient::entry();
        }
        request()->resultCode = OPERATION_PENDING;
        switch (request()->cmd)
        {
            case MemoryConfigClientRequest::CMD_READ:
            case MemoryConfigClientRequest::CMD_READ_PART:
                return allocate_and_call(
                    STATE(do_stream_read), dg_service()->client_allocator());
            default:
                return return_with_error(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
        }
    }

    Action do_stream_read()
    {
        dgClient_ = full_allocation_result(dg_service()->client_allocator());
        memoryConfigHandler_->set_client(&responseFlow_);
        {
            // Opens the stream receiver.
            receiver_->pool()->alloc(&streamRecvRequest_);
            /// @todo add option to specify byte sink directly.
            streamRecvRequest_->data()->reset(&defaultSink_, node_,
                request()->dst, StreamDefs::INVALID_STREAM_ID, dstStreamId_);
            streamRecvRequest_->data()->done.reset(this);
            // We keep an extra reference.
            streamRecvRequest_->data()->done.new_child();
            receiver_->send(streamRecvRequest_->ref());
        }
        return allocate_and_call(dg_service()->iface()->dispatcher(),
            STATE(send_stream_read_datagram));
    }

    Action send_stream_read_datagram()
    {
        auto *b = get_allocation_result(dg_service()->iface()->dispatcher());
        b->set_done(bn_.reset(this));
        b->data()->reset(Defs::MTI_DATAGRAM, node_->node_id(), request()->dst,
            MemoryConfigDefs::read_stream_datagram(request()->memory_space,
                request()->address, dstStreamId_, request()->size));

        isWaitingForTimer_ = 0;
        responseCode_ = DatagramClient::OPERATION_PENDING;
        dgClient_->write_datagram(b);
        return wait_and_call(STATE(stream_read_dg_complete));
    }

    Action stream_read_dg_complete()
    {
        if (!(dgClient_->result() & DatagramClient::OPERATION_SUCCESS))
        {
            // some error occurred.
            return handle_read_error(dgClient_->result());
        }
        if (responseCode_ & DatagramClient::OPERATION_PENDING)
        {
            isWaitingForTimer_ = 1;
            return sleep_and_call(
                &timer_, SEC_TO_NSEC(3), STATE(stream_read_response_timeout));
        }
        else
        {
            return call_immediately(STATE(stream_read_response_timeout));
        }
    }

    Action stream_read_response_timeout()
    {
        if (responseCode_ & DatagramClient::OPERATION_PENDING)
        {
            return handle_read_error(Defs::OPENMRN_TIMEOUT);
        }
        const uint8_t *bytes =
            MemoryConfigDefs::payload_bytes(responsePayload_);
        if (!MemoryConfigDefs::payload_min_length_check(responsePayload_, 2))
        {
            LOG(INFO,
                "Memory Config client: response datagram payload not "
                "long enough");
            return handle_read_error(
                Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        unsigned ofs = MemoryConfigDefs::get_payload_offset(responsePayload_);
        unsigned address = MemoryConfigDefs::get_address(responsePayload_);
        uint8_t space = MemoryConfigDefs::get_space(responsePayload_);
        uint8_t cmd = bytes[1] & MemoryConfigDefs::COMMAND_MASK;
        if (address != request()->address)
        {
            LOG(VERBOSE, "mismatched address a %u o %u", (unsigned)address,
                (unsigned)request()->address);
            return handle_read_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (space != request()->memory_space)
        {
            LOG(VERBOSE, "mismatched space");
            return handle_read_error(Defs::ERROR_OUT_OF_ORDER);
        }
        if (cmd == MemoryConfigDefs::COMMAND_READ_STREAM_FAILED)
        {
            uint16_t error = bytes[ofs++];
            error <<= 8;
            error |= bytes[ofs];
            return handle_read_error(error);
        }
        if (cmd != MemoryConfigDefs::COMMAND_READ_STREAM_REPLY)
        {
            return handle_read_error(Defs::ERROR_UNIMPLEMENTED);
        }
        // Now: we have a read success, and the stream is probably happening.
        return call_immediately(STATE(wait_for_stream_complete));
    }

    Action wait_for_stream_complete()
    {
        if (streamRecvRequest_->data()->done.abort_if_almost_done()) {
            // There was already a close request.
            return call_immediately(STATE(recv_stream_closed));
        } else {
            // We don't need to hold on to our extra ref anymore.
            streamRecvRequest_->data()->done.notify();
            return wait_and_call(STATE(recv_stream_closed));
        }
    }

    Action recv_stream_closed()
    {
        return finish_read();
    }

    /// Called upon various error conditions, typically before opening the
    /// stream.
    Action handle_read_error(int error)
    {
        // We don't need to hold on to our extra ref anymore.
        streamRecvRequest_->data()->done.notify();
        receiver_->cancel_request();
        request()->resultCode = error;
        return wait_and_call(STATE(cleanup_after_error));
    }

    Action cleanup_after_error()
    {
        cleanup_read();
        return return_with_error(request()->resultCode);
    }

    /// Stores incoming stream data into the request()->payload object
    /// (which is a string).
    struct DefaultSink : public ByteSink
    {
        DefaultSink(MemoryConfigClientWithStream *parent)
            : parent_(parent)
        { }

        void send(ByteBuffer *msg, unsigned prio) override
        {
            auto rb = get_buffer_deleter(msg);
            parent_->request()->payload.append(
                (char *)msg->data()->data_, msg->data()->size());
            if (parent_->request()->progressCb)
            {
                parent_->request()->progressCb(parent_->request());
            }
        }

        MemoryConfigClientWithStream *parent_;
    } defaultSink_{this};

    std::unique_ptr<StreamReceiverInterface> receiver_;
    /// stream ID on the local device.
    uint8_t dstStreamId_;
    /// Holds a ref to the stream receiver request.
    BufferPtr<StreamReceiveRequest> streamRecvRequest_;
}; // class MemoryConfigClientWithStream

} // namespace openlcb

#endif
