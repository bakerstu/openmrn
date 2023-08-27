/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file MemoryConfig.hxx
 *
 * Implementation of the Memory Config Protocol server
 *
 * @author Balazs Racz
 * @date 23 Feb 2014
 */

#ifndef _OPENLCB_MEMORYCONFIG_HXX_
#define _OPENLCB_MEMORYCONFIG_HXX_

#include "openlcb/DatagramDefs.hxx"
#include "openlcb/DatagramHandlerDefault.hxx"
#include "openlcb/MemoryConfigDefs.hxx"
#include "openmrn_features.h"
#include "utils/ConfigUpdateService.hxx"
#include "utils/Destructable.hxx"

class Notifiable;

extern "C" {
extern void enter_bootloader();
/** @todo need to find an alternative for reboot() for MacOS */
#if OPENMRN_FEATURE_REBOOT
extern void reboot();
#endif
}

namespace openlcb
{

/// Abstract base class for the address spaces exported via the Memory Config
/// Protocol.
///
/// Usage: Instantiate the specific child class of this interface. Register for
/// the address space number needed via @ref
/// MemoryConfigHandler::registry()->insert()
class MemorySpace : public Destructable
{
public:
    typedef uint32_t address_t;
    typedef uint16_t errorcode_t;

    /** This error code signals that the operation was only partially
     * completed, the again notify was used and will be notified when the
     * operation can be re-tried). */
    static const errorcode_t ERROR_AGAIN = 0x3FFF;

    /// Specifies which node the next operation pertains. If it returns false,
    /// the operation will be rejected by "unknown memory space ID".
    virtual bool set_node(Node* node)
    {
        return true;
    }

    /// @returns whether the memory space does not accept writes.
    virtual bool read_only()
    {
        return true;
    }
    /// @returns the lowest address that's valid for this block.
    virtual address_t min_address()
    {
        return 0;
    }
    /** @returns the largest valid address for this block.  A read of 1 from
     *  this address should succeed in returning the last byte.
     */
    virtual address_t max_address() = 0;

    /** @returns the number of bytes successfully written (before hitting end
     * of space). If *error is set to non-null, then the operation has
     * failed. If the operation needs to be continued, then sets error to
     * MemorySpace::ERROR_AGAIN, and calls the Notifiable @param again when a
     * re-try makes sense. The caller should call write once more, with the
     * offset adjusted with the previously returned bytes. */
    virtual size_t write(address_t destination, const uint8_t *data, size_t len,
                         errorcode_t *error, Notifiable *again)
    {
        DIE("Unimplemented");
    }
    /** @returns the number of bytes successfully read (before hitting end of
     * space). If *error is set to non-null, then the operation has failed. If
     * the operation needs to be continued, then sets error to ERROR_AGAIN, and
     * calls the Notifiable @param again when a re-try makes sense. The caller
     * should call read once more, with the offset adjusted with the previously
     * returned bytes. */
    virtual size_t read(address_t source, uint8_t *dst, size_t len,
                        errorcode_t *error, Notifiable *again) = 0;

    /** Handles space freeze command. Returns an error code, or 0 for
     * success. */
    virtual errorcode_t freeze() {
        return Defs::ERROR_INVALID_ARGS;
    }

    /** Handles space unfreeze command. Returns an error code, or 0 for
     * success. */
    virtual errorcode_t unfreeze() {
        return Defs::ERROR_INVALID_ARGS;
    }
};

/// Memory space implementation that exports a some memory-mapped data as a
/// read-only memory space. The data must be given as a const void* pointer,
/// which can point both to RAM or flash (either rodata or specific flash
/// addresses).
class ReadOnlyMemoryBlock : public MemorySpace
{
public:
    /** Creates a memory block for a given pointer of data. The pointer must
     * stay alive so long as this object is alive.  @param data is a
     * null-terminated string, which may point into read-only memory. */
    ReadOnlyMemoryBlock(const void *data)
        : data_(reinterpret_cast<const uint8_t *>(data))
        , len_(strlen((const char*)data_))
    {
    }

    /** Initializes a memory block with a given block of memory. The address
     * range [data, data+len) must be dereferenceable for read so long as this
     * object is alive. It may point into read-only memory. */
    ReadOnlyMemoryBlock(const void *data, address_t len)
        : data_(reinterpret_cast<const uint8_t*>(data))
        , len_(len)
    {
    }

    address_t max_address() OVERRIDE
    {
        return len_ - 1;
    }

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                Notifiable *again) OVERRIDE
    {
        if (source >= len_) {
            *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        size_t count = len;
        if (source + count > len_)
        {
            count = len_ - source;
        }
        memcpy(dst, data_ + source, count);
        return count;
    }

private:
    const uint8_t *data_; //< Data bytes to serve.
    const address_t len_; //< Length of block to serve.
};

/// Memory space implementation that exports a some memory-mapped data as a
/// read-write memory space. The data must be given as a void* pointer pointing
/// to RAM (or other memory-mapped structures).
class ReadWriteMemoryBlock : public MemorySpace
{
public:
    /** Initializes a memory block with a given block of memory. The address
     * range [data, data+len) must be dereferenceable for read and write so
     * long as this object is alive. */
    ReadWriteMemoryBlock(void *data, address_t len)
        : data_(reinterpret_cast<uint8_t*>(data))
        , len_(len)
    {
    }

    /// @returns whether the memory space does not accept writes.
    bool read_only() OVERRIDE
    {
        return false;
    }

    address_t max_address() OVERRIDE
    {
        return len_ - 1;
    }

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                Notifiable *again) OVERRIDE
    {
        if (source >= len_) {
            *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        size_t count = len;
        if (source + count > len_)
        {
            count = len_ - source;
        }
        memcpy(dst, data_ + source, count);
        return count;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) OVERRIDE {
        if (destination >= len_) {
            *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        if (destination + len > len_) {
            len = len_ - destination;
        }
        memcpy(data_ + destination, data, len);
        return len;
    }

private:
    uint8_t *data_; //< Data bytes to serve.
    const address_t len_; //< Length of block to serve.
};

/// Memory space implementation that exports the contents of a file as a memory
/// space. The file can be specified either as a path or an fd. By default
/// writes are also allowed.
class FileMemorySpace : public MemorySpace
{
public:
    static const address_t AUTO_LEN = (address_t) - 1;

    /** Creates a memory space based on an fd.
     *
     * @param fd is an open file descriptor with the data.
     * @param len tells how many bytes there are in the memory space. If
     * specified as AUTO_LEN, then uses fstat to figure out the size of the
     * file.
     */
    FileMemorySpace(int fd, address_t len = AUTO_LEN);

    /** Creates a memory space based on a file name. Opens the file at the
     * first use, and never closes it.
     *
     * @param name is the file name to open. The pointer must stay alive so
     * long as *this is around.
     * @param len tells how many bytes there are in the memory space. If
     * specified as AUTO_LEN, then uses fstat to figure out the size of the
     * file.
     */
    FileMemorySpace(const char *name, address_t len = AUTO_LEN);

    bool read_only() OVERRIDE
    {
        return false;
    }

    address_t max_address() OVERRIDE
    {
        ensure_file_open();
        return fileSize_;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) OVERRIDE;

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                Notifiable *again) OVERRIDE;

private:
    /** Makes fd a valid parameter, and ensures fileSize is filled in. */
    void ensure_file_open();

    address_t fileSize_;
    const char *name_;
    int fd_;
};

/// Memory space implementation that exports the contents of a file as a memory
/// space. The file can be specified either as a path or an fd. By default
/// writes are also allowed.
class ROFileMemorySpace : public FileMemorySpace {
public:
    /** Creates a memory space based on an fd.
     *
     * @param fd is an open file descriptor with the data.
     * @param len tells how many bytes there are in the memory space. If
     * specified as AUTO_LEN, then uses fstat to figure out the size of the
     * file.
     */
    ROFileMemorySpace(int fd, address_t len = AUTO_LEN)
        : FileMemorySpace(fd, len) {}

    /** Creates a memory space based on a file name. Opens the file at the
     * first use, and never closes it.
     *
     * @param name is the file name to open. The pointer must stay alive so
     * long as *this is around.
     * @param len tells how many bytes there are in the memory space. If
     * specified as AUTO_LEN, then uses fstat to figure out the size of the
     * file.
     */
    ROFileMemorySpace(const char *name, address_t len = AUTO_LEN)
        : FileMemorySpace(name, len) {}

    bool read_only() OVERRIDE
    {
        return true;
    }
};

class MemoryConfigHandlerBase : public DefaultDatagramHandler
{
protected:
    enum
    {
        DATAGRAM_ID = DatagramDefs::CONFIGURATION,
    };

    MemoryConfigHandlerBase(DatagramService *if_dg)
        : DefaultDatagramHandler(if_dg)
        , responseFlow_(nullptr)
    {
    }

    typedef MemorySpace::address_t address_t;
    typedef MemorySpace::errorcode_t errorcode_t;
    typedef TypedNodeHandlerMap<Node, MemorySpace> Registry;

    Action ok_response_sent() OVERRIDE
    {
        if (!response_.empty())
        {
            return allocate_and_call(
                STATE(client_allocated), dg_service()->client_allocator());
        }
        else
        {
            release();
            return call_immediately(STATE(cleanup));
        }
    }

    Action cleanup()
    {
        HASSERT(!message());
        return exit();
    }

    Action client_allocated()
    {
        responseFlow_ =
            full_allocation_result(dg_service()->client_allocator());
        return allocate_and_call(
            dg_service()->iface()->dispatcher(), STATE(send_response_datagram));
    }

    Action send_response_datagram()
    {
        auto *b = get_allocation_result(dg_service()->iface()->dispatcher());
        b->set_done(b_.reset(this));
        b->data()->reset(Defs::MTI_DATAGRAM, message()->data()->dst->node_id(),
            message()->data()->src, EMPTY_PAYLOAD);
        b->data()->payload.swap(response_);
        release(); /// @TODO(balazs.racz) Should this be here or elsewhere?
        responseFlow_->write_datagram(b);
        return wait_and_call(STATE(response_flow_complete));
    }

    Action response_flow_complete()
    {
        if (!(responseFlow_->result() & DatagramClient::OPERATION_SUCCESS))
        {
            LOG(WARNING,
                "MemoryConfig: Failed to send response datagram. error code %x",
                (unsigned)responseFlow_->result());
        }
        dg_service()->client_allocator()->typed_insert(responseFlow_);
        return call_immediately(STATE(cleanup));
    }

    /// @return true iff we have a custom space
    bool has_custom_space()
    {
        return !(in_bytes()[1] & ~MemoryConfigDefs::COMMAND_MASK);
    }

    /** Returns the memory space number, or -1 if the incoming datagram is of
     * incorrect format. Assumes that the incoming datagram length is at least
     * 2 (i.e., there is a command byte).*/
    int get_space_number()
    {
        const uint8_t *bytes = in_bytes();
        int len = message()->data()->payload.size();
        uint8_t cmd = bytes[1];
        // Handles special memory spaces FD, FE, FF.
        if (!has_custom_space())
        {
            return MemoryConfigDefs::COMMAND_MASK +
                (cmd & ~MemoryConfigDefs::COMMAND_MASK);
        }
        if (len <= 6)
        {
            LOG(WARNING,
                "MemoryConfig: Incoming datagram asked for custom "
                "space but datagram not long enough. command=0x%02x, "
                "length=%d. Source {0x%012" PRIx64 ", %03x}",
                cmd, len, message()->data()->src.id,
                message()->data()->src.alias);
            return -1;
        }
        return bytes[6];
    }

    /** Returns the read/write length from byte 6 or 7 of the incoming
     * datagram, or -1 if the incoming datagram is of incorrect format. */
    int get_read_length()
    {
        const uint8_t *bytes = in_bytes();
        int len = message()->data()->payload.size();
        int ofs;
        uint8_t cmd = bytes[1];
        // Handles special memory spaces FD, FE, FF.
        if (!has_custom_space())
        {
            ofs = 6;
        }
        else
        {
            ofs = 7;
        }
        if (len <= ofs)
        {
            LOG(WARNING,
                "MemoryConfig::read_len: Incoming datagram not long "
                "enough. command=0x%02x, length=%d. Source "
                "{0x%012" PRIx64 ", %03x}",
                cmd, len, message()->data()->src.id,
                message()->data()->src.alias);
            return -1;
        }
        return bytes[ofs] & 0x7F; // highest bit is reserved.
    }

    int get_write_length()
    {
        int len = message()->data()->payload.size();
        return len - (has_custom_space() ? 7 : 6);
    }

    /** Returns the address from the incoming datagram. Assumes that length >=
     * 6*/
    address_t get_address()
    {
        const uint8_t *bytes = in_bytes();
        address_t a = bytes[2];
        a <<= 8;
        a |= bytes[3];
        a <<= 8;
        a |= bytes[4];
        a <<= 8;
        a |= bytes[5];
        return a;
    }

    /** Copies the address and memory space information from the incoming
     * datagram to the outgoing datagram payload. Modifies the low bits of the
     * response command byte, if needed. Callers are advised to set the base
     * command value before calling this function. */
    void set_address_and_space()
    {
        uint8_t *resp_bytes = out_bytes();
        const uint8_t *bytes = in_bytes();
        memcpy(resp_bytes + 2, bytes + 2, 4);
        if (has_custom_space())
        {
            resp_bytes[6] = bytes[6];
        }
        else
        {
            resp_bytes[1] |= (bytes[1] & ~MemoryConfigDefs::COMMAND_MASK);
        }
    }

    /// Sets the address in the response payload buffer.
    void set_address(address_t address)
    {
        uint8_t *bytes = out_bytes();
        bytes[5] = address & 0xff;
        address >>= 8;
        bytes[4] = address & 0xff;
        address >>= 8;
        bytes[3] = address & 0xff;
        address >>= 8;
        bytes[2] = address & 0xff;
    }

    /// @returns the response datagram payload buffer.
    uint8_t *out_bytes()
    {
        return reinterpret_cast<uint8_t *>(&response_[0]);
    }

    /// @returns the request datagram payload buffer.
    const uint8_t *in_bytes()
    {
        return reinterpret_cast<const uint8_t *>(
            message()->data()->payload.data());
    }

    DatagramPayload response_; //< reply payload to send back.
    DatagramClient *responseFlow_;
    BarrierNotifiable b_;
}; // class MemoryConfigHandlerBase

/// Implementation of the Memory Access Configuration Protocol for OpenLCB.
///
/// Usage: Create an instance of this object either for the specific virtual
/// node, or for an entire interface. Create your memory spaces using various
/// children of the class @ref MemorySpace. Register the memory spaces using
/// @ref registry().
class MemoryConfigHandler : public MemoryConfigHandlerBase
{
public:
    /// node can be nullptr, and then the handler will be registered globally.
    MemoryConfigHandler(DatagramService *if_dg, Node *node, int registry_size)
        : MemoryConfigHandlerBase(if_dg)
        , registry_(registry_size)
    {
        dg_service()->registry()->insert(node, DATAGRAM_ID, this);
    }

    ~MemoryConfigHandler()
    {
        /// @TODO(balazs.racz): unregister *this!
    }

    Registry *registry()
    {
        return &registry_;
    }

    /// Overrides the default send method in orderto decide whether the queue
    /// the incoming datagram in the server queue or the client queue.
    void send(DefaultDatagramHandler::message_type *message, unsigned priority = UINT_MAX) override
    {
        size_t len = message->data()->payload.size();
        const uint8_t *bytes = (const uint8_t *)message->data()->payload.data();
        uint8_t cmd = ((len >= 2) && (client_ != nullptr)) ? bytes[1] : 0;
        bool is_client_command = false;
        // To recognize replies for read & write commands, we need to look at a
        // bit.
        if ((cmd < MemoryConfigDefs::COMMAND_MAX_FOR_RW) &&
            (cmd & MemoryConfigDefs::COMMAND_REPLY_BIT_FOR_RW))
        {
            is_client_command = true;
        }
        // For the rest of the commands we can enumerate which are replies.
        switch (cmd)
        {
            case MemoryConfigDefs::COMMAND_OPTIONS_REPLY:
            case MemoryConfigDefs::COMMAND_INFORMATION_REPLY:
            case MemoryConfigDefs::COMMAND_LOCK_REPLY:
            case MemoryConfigDefs::COMMAND_UNIQUE_ID_REPLY:
            {
                is_client_command = true;
            }
            default:
                break;
        }
        LOG(VERBOSE, "MemoryConfig incoming cmd 0x%02x is_client %d", cmd,
            is_client_command);
        if (is_client_command)
        {
            client_->send(message, priority);
            return;
        }
        DatagramHandlerFlow::send(message, priority);
    }

    /// Registers a second handler to forward all the client interactions,
    /// i.e. everythingthat comes back with the RESPONSE bit set.
    void set_client(DatagramHandlerFlow* client) {
        HASSERT(client_ == nullptr || client_ == client);
        client_ = client;
    }

    /// Unregisters the previously registered second handler.
    void clear_client(DatagramHandlerFlow* client) {
        HASSERT(client_ == client);
        client_ = nullptr;
    }

    /// This will be called by the constructor of the stream handler plugin.
    void set_stream_handler(DatagramHandlerFlow *stream_handler)
    {
        streamHandler_ = stream_handler;
    }

private:
    Action entry() OVERRIDE
    {
        response_.clear();
        const uint8_t *bytes = in_bytes();
        size_t len = message()->data()->payload.size();
        HASSERT(len >= 1);
        HASSERT(bytes[0] == DATAGRAM_ID);
        if (len < 2)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }

        uint8_t cmd = bytes[1];

        if ((cmd & MemoryConfigDefs::COMMAND_MASK) ==
            MemoryConfigDefs::COMMAND_READ)
        {
            return call_immediately(STATE(handle_read));
        }
        else if ((cmd & MemoryConfigDefs::COMMAND_MASK) ==
                 MemoryConfigDefs::COMMAND_WRITE)
        {
            return call_immediately(STATE(handle_write));
        }
        else if ((((cmd & MemoryConfigDefs::COMMAND_MASK) ==
                      MemoryConfigDefs::COMMAND_READ_STREAM) ||
                     ((cmd & MemoryConfigDefs::COMMAND_MASK) ==
                         MemoryConfigDefs::COMMAND_WRITE_STREAM)) &&
            streamHandler_)
        {
            streamHandler_->send(transfer_message());
            return exit();
        }
        switch (cmd)
        {
            case MemoryConfigDefs::COMMAND_LOCK:
            {
                // Unknown/unsupported command, reject datagram.
                return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);

                // if (
                break;
            }
            case MemoryConfigDefs::COMMAND_FREEZE:
            {
                if (len < 3)
                {
                    return respond_reject(
                        Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
                }
                uint8_t space = bytes[2];
                if (space != MemoryConfigDefs::SPACE_FIRMWARE)
                {
                    // Custom spaces cannot do free yet.
                    return respond_reject(Defs::ERROR_INVALID_ARGS);
                }
                // Fall through.
            }
            case MemoryConfigDefs::COMMAND_ENTER_BOOTLOADER:
            {
                enter_bootloader();
                return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
            }
            case MemoryConfigDefs::COMMAND_UPDATE_COMPLETE:
            {
                Singleton<ConfigUpdateService>::instance()->trigger_update();
                return respond_ok(0);
            }
            case MemoryConfigDefs::COMMAND_RESET:
            {
#if OPENMRN_FEATURE_REBOOT
                reboot();
#endif
                return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
            }
            case MemoryConfigDefs::COMMAND_FACTORY_RESET:
            {
                NodeID id = message()->data()->dst->node_id();
                if (len < 8 || (data_to_node_id(&bytes[2]) != id))
                {
                    return respond_reject(Defs::ERROR_INVALID_ARGS);
                }
                uint16_t ret = handle_factory_reset(id);
                if (!ret)
                {
                    return respond_ok(0);
                }
                else
                {
                    return respond_reject(ret);
                }
            }
            case MemoryConfigDefs::COMMAND_OPTIONS:
            {
                return call_immediately(STATE(handle_options));
            }
            case MemoryConfigDefs::COMMAND_INFORMATION:
            {
                return call_immediately(STATE(handle_get_space_info));
            }
            case MemoryConfigDefs::COMMAND_WRITE_REPLY:
            case MemoryConfigDefs::COMMAND_WRITE_FAILED:
            case MemoryConfigDefs::COMMAND_WRITE_STREAM_REPLY:
            case MemoryConfigDefs::COMMAND_WRITE_STREAM_FAILED:
            case MemoryConfigDefs::COMMAND_READ_REPLY:
            case MemoryConfigDefs::COMMAND_READ_FAILED:
            case MemoryConfigDefs::COMMAND_READ_STREAM_REPLY:
            case MemoryConfigDefs::COMMAND_READ_STREAM_FAILED:
            case MemoryConfigDefs::COMMAND_OPTIONS_REPLY:
            case MemoryConfigDefs::COMMAND_INFORMATION_REPLY:
            case MemoryConfigDefs::COMMAND_LOCK_REPLY:
            case MemoryConfigDefs::COMMAND_UNIQUE_ID_REPLY:
            {
                if (client_)
                {
                    client_->send(transfer_message());
                    return exit();
                }
                LOG(VERBOSE, "memcfg handler reply: no client registered");
            } // fall through to unsupported.
            default:
                // Unknown/unsupported command, reject datagram.
                return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
        }
    }

    /// Used internally by the factory_reset implementation to reboot the
    /// binary asynchronously.
    class RebootTimer : public ::Timer
    {
    public:
        RebootTimer(Service *s)
            : ::Timer(s->executor()->active_timers())
        { }

        long long timeout() override
        {
#if OPENMRN_FEATURE_REBOOT         
            reboot();
#endif            
            return DELETE;
        }
    };

    /// Invokes the openlcb config handler to do a factory reset. Starts a
    /// timer to reboot the device after a little time.
    /// @param target the node ID for which factory reset was invoked.
    /// @return openlcb error code, 0 on success
    uint16_t handle_factory_reset(NodeID target);

    /// Weak definition for invoking a factory reset on virtual nodes. The
    /// default implementation does nothing and return an unimplemented
    /// error. Applications that use virtual nodes and need to support factory
    /// reset should reimplement this function.
    /// @param target the node ID for which factory reset was invoked.
    /// @return openlcb error code, 0 on success
    uint16_t __attribute__((noinline)) app_handle_factory_reset(NodeID target);


    Action handle_options()
    {
        response_.reserve(7);
        response_.push_back(DATAGRAM_ID);
        response_.push_back(MemoryConfigDefs::COMMAND_OPTIONS_REPLY);
        uint16_t available_commands =
            MemoryConfigDefs::AVAIL_UR | MemoryConfigDefs::AVAIL_UW;
        if (streamHandler_)
        {
            available_commands |= MemoryConfigDefs::AVAIL_SR;
        }
        // Figure out about ACDI spaces
        MemorySpace* memspace = registry_.lookup(message()->data()->dst, 0xFC);
        if (memspace) {
            available_commands |= MemoryConfigDefs::AVAIL_R0xFC;
        }
        memspace = registry_.lookup(message()->data()->dst, 0xFB);
        if (memspace) {
            available_commands |= MemoryConfigDefs::AVAIL_R0xFB;
            if (!memspace->read_only()) {
                available_commands |= MemoryConfigDefs::AVAIL_W0xFB;
            }
        }
        response_.push_back(available_commands >> 8);
        response_.push_back(available_commands & 0xff);
        // Write lengths
        response_.push_back(static_cast<char> (
            MemoryConfigDefs::LENGTH_1 | MemoryConfigDefs::LENGTH_2 |
            MemoryConfigDefs::LENGTH_4 | MemoryConfigDefs::LENGTH_ARBITRARY));

        uint8_t min_space = 0xFF;
        uint8_t max_space = 0;
        // Walks the spaces.
        for (auto it = registry_.begin(); it != registry_.end(); ++it) {
            auto h = *it;
            uint8_t space = h.first.second;
            Node* node = h.first.first;
            //MemorySpace* space_impl = h.second;
            if (node && node != message()->data()->dst) {
                continue;
            }
            if (space >= 0xFD) continue;
            if (space > max_space) max_space = space;
            if (space < min_space) min_space = space;
        }
        if (max_space < min_space) {
            max_space = 0xff;
            min_space = 0xfd;
        }
        response_.push_back(max_space);
        response_.push_back(min_space);
        return respond_ok(DatagramClient::REPLY_PENDING);
    }

    Action handle_get_space_info()
    {
        if (message()->data()->payload.size() < 3) {
            // Incoming message too short.
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
        uint8_t space_number = in_bytes()[2];
        MemorySpace* space = registry_.lookup(message()->data()->dst, space_number);
        response_.reserve(8);
        response_.push_back(DATAGRAM_ID);
        response_.push_back(MemoryConfigDefs::COMMAND_INFORMATION_REPLY);
        response_.push_back(space_number);
        if (!space) {
            return respond_ok(DatagramDefs::REPLY_PENDING);
        } else {
            response_[1]++;
        }
        MemorySpace::address_t address = space->max_address();
        response_.push_back((address >> 24) & 0xff);
        response_.push_back((address >> 16) & 0xff);
        response_.push_back((address >> 8) & 0xff);
        response_.push_back(address & 0xff);
        uint8_t flags = 0;
        address = space->min_address();
        if (address) {
            flags |= MemoryConfigDefs::FLAG_NZLA;
        }
        if (space->read_only()) {
            flags |= MemoryConfigDefs::FLAG_RO;
        }
        response_.push_back(flags);
        if (address) {
            response_.push_back((address >> 24) & 0xff);
            response_.push_back((address >> 16) & 0xff);
            response_.push_back((address >> 8) & 0xff);
            response_.push_back(address & 0xff);
        }
        return respond_ok(DatagramDefs::REPLY_PENDING);
    }

    Action handle_read()
    {
        size_t len = message()->data()->payload.size();
        if (len <= 6)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS);
        }
        MemorySpace *space = get_space();
        if (!space)
        {
            return respond_reject(MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN);
        }
        int read_len = get_read_length();
        if (read_len < 0)
        {
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
        size_t response_data_offset = 6;
        if (has_custom_space())
        {
            ++response_data_offset;
        }
        size_t response_len = response_data_offset + read_len;
        currentOffset_ = 0;
        char c = 0;
        response_.assign(response_len, c);
        return call_immediately(STATE(try_read));
    }

    Action try_read() {
        MemorySpace *space = get_space();
        int read_len = get_read_length();
        address_t address = get_address();
        size_t response_data_offset = 6;
        if (has_custom_space())
        {
            ++response_data_offset;
        }
        address += currentOffset_;
        response_data_offset += currentOffset_;
        read_len -= currentOffset_;
        errorcode_t error = 0;
        uint8_t *response_bytes = out_bytes();
        if (read_len > 0)
        {
            int byte_read = space->read(address,
                response_bytes + response_data_offset, read_len, &error, this);
            currentOffset_ += byte_read;
            read_len -= byte_read;
            if (error == MemorySpace::ERROR_AGAIN)
            {
                return wait();
            }
            else if (error == 0 && read_len)
            {
                return again();
            }
        }
        if (error == MemoryConfigDefs::ERROR_OUT_OF_BOUNDS && currentOffset_)
        {
            // We can return a partial response.
            error = 0;
        }
        response_bytes[0] = DATAGRAM_ID;
        response_bytes[1] = error ? MemoryConfigDefs::COMMAND_READ_FAILED
                                  : MemoryConfigDefs::COMMAND_READ_REPLY;
        set_address_and_space();
        response_data_offset = 6;
        if (has_custom_space())
        {
            ++response_data_offset;
        }
        if (error)
        {
            response_.resize(response_data_offset + 2);
            out_bytes()[response_data_offset] = error >> 8;
            out_bytes()[response_data_offset + 1] = error & 0xff;
        }
        else
        {
            response_.resize(response_data_offset + currentOffset_);
        }
        return respond_ok(DatagramClient::REPLY_PENDING);
    }

    Action handle_write()
    {
        size_t len = message()->data()->payload.size();
        if (len <= 6)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        MemorySpace *space = get_space();
        if (!space)
        {
            return respond_reject(MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN);
        }
        if (space->read_only())
        {
            return respond_reject(MemoryConfigDefs::ERROR_WRITE_TO_RO);
        }
        int write_len = get_write_length();
        if (write_len <= 0)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS);
        }
        currentOffset_ = 0;
        inline_respond_ok(DatagramClient::REPLY_PENDING);
        return call_immediately(STATE(try_write));
    }

    /// Internal loop performing the writes. This state will be invoked
    /// multiple times if the writes are asynchronous. The OK datagram response
    /// has been already sent. Eventually when all writes are completed, the
    /// ok_response_sent state will be invoked. This is correct even if there
    /// is an error; the error will be returned in the response datagram.
    Action try_write()
    {
        MemorySpace *space = get_space();
        int write_len = get_write_length();
        address_t address = get_address();
        size_t data_offset = 6;
        if (has_custom_space())
        {
            ++data_offset;
        }
        address += currentOffset_;
        data_offset += currentOffset_;
        write_len -= currentOffset_;
        errorcode_t error = 0;
        if (write_len > 0)
        {
            size_t written = space->write(address, in_bytes() + data_offset,
                                          write_len, &error, this);
            currentOffset_ += written;
            write_len -= written;
            if (error == MemorySpace::ERROR_AGAIN)
            {
                return wait();
            }
            else if (error == 0 && write_len)
            {
                return again();
            }
        }
        char c = 0;
        int response_len = 6;
        if (has_custom_space())
        {
            response_len++;
        }
        if (error == 0)
        {
            response_.assign(response_len, c);
            out_bytes()[1] = MemoryConfigDefs::COMMAND_WRITE_REPLY;
        }
        else
        {
            response_.assign(response_len + 2, c);
            out_bytes()[1] = MemoryConfigDefs::COMMAND_WRITE_FAILED;
            out_bytes()[response_len] = error >> 8;
            out_bytes()[response_len + 1] = error & 0xff;
        }
        out_bytes()[0] = DATAGRAM_ID;
        set_address_and_space();
        return call_immediately(STATE(ok_response_sent));
    }

    /** Looks up the memory space for the current datagram. Returns NULL if no
     * space was registered (for neither the current node, nor global). */
    MemorySpace *get_space()
    {
        int space_number = get_space_number();
        if (space_number < 0)
            return nullptr;
        MemorySpace *space =
            registry_.lookup(message()->data()->dst, space_number);
        if (!space)
        {
            LOG(WARNING, "MemoryConfig: asked node 0x%012" PRIx64 " for unknown space "
                         "%d. Source {0x%012" PRIx64 ", %03x}",
                message()->data()->dst->node_id(), space_number,
                message()->data()->src.id, message()->data()->src.alias);
            return nullptr;
        }
        if (!space->set_node(message()->data()->dst))
        {
            LOG(WARNING, "MemoryConfig: Global space %d rejected node.",
                space_number);
            return nullptr;
        }
        return space;
    }

    ///@todo (balazs.racz) implement lock/unlock.
    //NodeID lockNode_; //< Holds the node ID that locked us.

    Registry registry_;         //< holds the known memory spaces
    /// If there is a memory config client, we will forward response traffic to
    /// it.
    DatagramHandlerFlow* client_{nullptr};
    /// If there is a handler for stream requests, we will forward the
    /// respective traffic to it.
    DatagramHandlerFlow *streamHandler_ {nullptr};

    /** Offset withing the current write/read datagram. This does not include
     * the offset from the incoming datagram. */
    uint8_t currentOffset_;
};

} // namespace openlcb

#endif // _OPENLCB_MEMORYCONFIG_HXX_
