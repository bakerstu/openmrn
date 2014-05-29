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

#ifndef _NMRANET_MEMORYCONFIG_HXX_
#define _NMRANET_MEMORYCONFIG_HXX_

#include "nmranet/DatagramHandlerDefault.hxx"
#include "nmranet/NMRAnetMemoryConfig.hxx"

namespace nmranet
{

struct MemoryConfigDefs {
    /** Possible Commands for a configuration datagram.
     */
    enum commands
    {
        COMMAND_MASK              = 0xFC,
        COMMAND_FLAG_MASK         = 0x03, /**< mask for special memory space flags */
        COMMAND_PRESENT_MASK      = 0x01, /**< mask for address space present bit */
        COMMAND_WRITE             = 0x00, /**< command to write data to address space */
        COMMAND_WRITE_UNDER_MASK  = 0x08, /**< command to write data under mask */
        COMMAND_WRITE_REPLY       = 0x10, /**< reply to write data to address space */
        COMMAND_WRITE_FAILED      = 0x18, /**< failed to write data to address space */
        COMMAND_WRITE_STREAM      = 0x20, /**< command to write data using a stream */
        COMMAND_READ              = 0x40, /**< command to read data from address space */
        COMMAND_READ_REPLY        = 0x50, /**< reply to read data from address space */
        COMMAND_READ_FAILED       = 0x58, /**< failed to read data from address space */
        COMMAND_READ_STREAM       = 0x60, /**< command to read data using a stream */
        COMMAND_OPTIONS           = 0x80,
        COMMAND_OPTIONS_REPLY     = 0x82,
        COMMAND_INFORMATION       = 0x84,
        COMMAND_INFORMATION_REPLY = 0x86,
        COMMAND_LOCK              = 0x88, /**< lock the configuration space */
        COMMAND_LOCK_REPLY        = 0x8A, /**< unlock the configuration space */
        COMMAND_UNIQUE_ID         = 0x8C, /**< ask for a node unique id */
        COMMAND_UNIQUE_ID_REPLY   = 0x8D, /**< node unique id */
        COMMAND_UPDATE_COMPLETE   = 0xA8, /**< indicate that a sequence of commands is complete */
        COMMAND_RESET             = 0xA9, /**< reset node to its power on state */
        COMMAND_FACTORY_RESET     = 0xAA, /**< reset node to factory defaults */
        COMMAND_FREEZE            = 0xA1, /**< freeze operation of node */
        COMMAND_UNFREEZE          = 0xA0, /**< unfreeze operation of node */

        COMMAND_PRESENT    = 0x01, /**< address space is present */

        COMMAND_CDI        = 0x03, /**< flags for a CDI space */
        COMMAND_ALL_MEMORY = 0x02, /**< flags for an all memory space */
        COMMAND_CONFIG     = 0x01, /**< flags for a config memory space */
    };

    /** Possible memory spaces.
     */
    enum spaces
    {
        SPACE_SPECIAL    = 0xFC, /**< offset for the special memory spaces */
        SPACE_CDI        = 0xFF, /**< CDI space */
        SPACE_ALL_MEMORY = 0xFE, /**< all memory space */
        SPACE_CONFIG     = 0xFD, /**< config memory space */
    };

    /** Possible available options.
     */
    enum available
    {
        AVAIL_WUM   = 0x8000, /**< write under mask supported */
        AVAIL_UR    = 0x4000, /**< unaligned reads supported */
        AVAIL_UW    = 0x2000, /**< unaligned writes supported */
        AVAIL_R0xFC = 0x0800, /**< read from adddress space 0xFC available */
        AVAIL_R0xFB = 0x0400, /**< read from adddress space 0xFB available */
        AVAIL_W0xFB = 0x0200, /**< write from adddress space 0xFB available */
    };

    /** Possible supported write lengths.
     */
    enum lengths
    {
        LENGTH_1         = 0x80, /**< write length of 1 supported */
        LENGTH_2         = 0x40, /**< write length of 2 supported */
        LENGTH_4         = 0x20, /**< write length of 4 supported */
        LENGTH_63        = 0x10, /**< write length of 64 supported */
        LENGTH_ARBITRARY = 0x02, /**< arbitrary write of any length supported */
        LENGTH_STREAM    = 0x01, /**< stream writes supported */
    };

    /** Possible address space information flags.
     */
    enum flags
    {
        FLAG_RO   = 0x01, /**< space is read only */
        FLAG_NZLA = 0x02, /**< space has a nonzero low address */
    };

private:
    /** Do not instantiate this class. */
    MemoryConfigDefs();
};

class MemorySpace
{
public:
    typedef uint32_t address_t;
    typedef uint16_t errorcode_t;
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

    virtual void write(address_t destination, const uint8_t* data, size_t len,
                       errorcode_t* error)
    {
        HASSERT(0);
    }
    /** @returns the number of bytes successfully read (before hitting end of
     * space). If *error is set to non-null, then the operation has failed. */
    virtual size_t read(address_t source, uint8_t* dst, size_t len,
                        errorcode_t* error) = 0;
};

class ReadOnlyMemoryBlock : public MemorySpace
{
public:
    /** Creates a memory block for a given pointer of data. The pointer must
     * stay alive so long as this object is alive.  @param data is a
     * null-terminated string, which may point into read-only memory. */
    ReadOnlyMemoryBlock(const char* data)
        : data_(reinterpret_cast<const uint8_t*>(data)), len_(strlen(data))
    {
    }

    /** Initializes a memory block with a given block of memory. The address
     * range [data, data+len) must be dereferenceable for read so long as this
     * object is alive. It may point into read-only memory. */
    ReadOnlyMemoryBlock(const uint8_t* data, address_t len)
        : data_(data), len_(len)
    {
    }

    virtual address_t max_address()
    {
        return len_ - 1;
    }

    virtual size_t read(address_t source, uint8_t* dst, size_t len,
                        errorcode_t* error)
    {
        if (source >= len_)
            return 0;
        size_t count = len;
        if (source + count > len_)
        {
            count = len_ - source;
        }
        memcpy(dst, data_ + source, count);
        return count;
    }

private:
    const uint8_t* data_; //< Data bytes to serve.
    const address_t len_; //< Length of block to serve.
};

class MemoryConfigHandler : public DefaultDatagramHandler
{
public:
    enum
    {
        DATAGRAM_ID = 0x20,
    };

    MemoryConfigHandler(DatagramSupport* if_dg, Node* node,
                        size_t registry_size)
        : DefaultDatagramHandler(if_dg),
          response_(nullptr),
          responseFlow_(nullptr),
          registry_(registry_size)
    {
        ifDatagram_->registry()->insert(node, DATAGRAM_ID, this);
    }

    ~MemoryConfigHandler()
    {
        /// @TODO(balazs.racz): unregister *this!
    }

    typedef TypedNodeHandlerMap<Node, MemorySpace> Registry;

    Registry* registry()
    {
        return &registry_;
    }

private:
    typedef MemorySpace::address_t address_t;
    typedef MemorySpace::errorcode_t errorcode_t;

    // override
    virtual Action datagram_arrived()
    {
        HASSERT(!response_);
        const uint8_t* bytes = in_bytes();
        size_t len = datagram_->payload->used();
        HASSERT(len >= 1);
        HASSERT(bytes[0] == DATAGRAM_ID);
        if (len < 2)
        {
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }

        uint8_t cmd = bytes[1];

        if ((cmd & MemoryConfig::COMMAND_MASK) == MemoryConfig::COMMAND_READ)
        {
            return call_immediately(STATE(handle_read));
        }
        switch (cmd)
        {
            case MemoryConfig::COMMAND_LOCK:
            {
                // Unknown/unsupported command, reject datagram.
                return respond_reject(DatagramClient::PERMANENT_ERROR);

                // if (
                break;
            }
            default:
                // Unknown/unsupported command, reject datagram.
                return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
    }

    virtual Action ok_response_sent()
    {
        if (response_)
        {
            return Allocate(ifDatagram_->client_allocator(),
                            ST(send_response_datagram));
        }
        else
        {
            datagram_->free();
            return call_immediately(STATE(cleanup));
        }
    }

    Action cleanup()
    {
        return call_immediately(STATE(wait_for_datagram));
    }

    Action send_response_datagram()
    {
        responseFlow_ =
            GetTypedAllocationResult(ifDatagram_->client_allocator());
        responseFlow_->write_datagram(datagram_->dst->node_id(), datagram_->src,
                                      response_, this);
        datagram_->free();
        return WaitAndCall(STATE(response_flow_complete));
    }

    Action response_flow_complete()
    {
        if (!responseFlow_->result() & DatagramClient::OPERATION_SUCCESS)
        {
            LOG(WARNING,
                "MemoryConfig: Failed to send response datagram. error code %x",
                (unsigned)responseFlow_->result());
        }
        ifDatagram_->client_allocator()->TypedRelease(responseFlow_);
        response_ = nullptr;
        return call_immediately(STATE(cleanup));
    }

    Action handle_read()
    {
        size_t len = datagram_->payload->used();
        if (len <= 6)
        {
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
        MemorySpace* space = get_space();
        if (!space)
        {
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
        int read_len = get_length();
        if (read_len < 0)
        {
            return respond_reject(DatagramClient::PERMANENT_ERROR);
        }
        address_t address = get_address();
        size_t response_data_offset = 6;
        if (has_custom_space())
        {
            ++response_data_offset;
        }
        size_t response_len = response_data_offset + read_len;
        response_ = buffer_alloc(response_len);
        uint8_t* response_bytes = out_bytes();
        errorcode_t error = 0;
        int byte_read = space->read(
            address, response_bytes + response_data_offset, read_len, &error);
        response_bytes[0] = DATAGRAM_ID;
        response_bytes[1] = error ? MemoryConfig::COMMAND_READ_FAILED
                                  : MemoryConfig::COMMAND_READ_REPLY;
        set_address_and_space();
        if (error)
        {
            response_bytes[response_data_offset] = error >> 8;
            response_bytes[response_data_offset + 1] = error & 0xff;
            response_->advance(response_data_offset + 2);
        }
        else
        {
            response_->advance(response_data_offset + byte_read);
        }
        return respond_ok(DatagramClient::REPLY_PENDING);
    }

    /// @return true iff we have a custom space
    bool has_custom_space()
    {
        return !(in_bytes()[1] & ~MemoryConfig::COMMAND_MASK);
    }

    /** Returns the memory space number, or -1 if the incoming datagram is of
     * incorrect format. Assumes that the incoming datagram length is at least
     * 2 (i.e., there is a command byte).*/
    int get_space_number()
    {
        const uint8_t* bytes = in_bytes();
        int len = datagram_->payload->used();
        uint8_t cmd = bytes[1];
        // Handles special memory spaces FD, FE, FF.
        if (!has_custom_space())
        {
            return MemoryConfig::COMMAND_MASK +
                   (cmd & ~MemoryConfig::COMMAND_MASK);
        }
        if (len <= 6)
        {
            LOG(WARNING, "MemoryConfig: Incoming datagram asked for custom "
                         "space but datagram not long enough. command=0x%02x, "
                         "length=%d. Source {0x%012llx, %03x}",
                cmd, len, datagram_->src.id, datagram_->src.alias);
            return -1;
        }
        return bytes[6];
    }

    /** Looks up the memory space for the current datagram. Returns NULL if no
     * space was registered (for neither the current node, nor global). */
    MemorySpace* get_space()
    {
        int space_number = get_space_number();
        if (space_number < 0)
            return nullptr;
        MemorySpace* space = registry_.lookup(datagram_->dst, space_number);
        if (!space)
        {
            LOG(WARNING, "MemoryConfig: asked node 0x%012llx for unknown space "
                         "%d. Source {0x%012llx, %03x}",
                datagram_->dst->node_id(), space_number, datagram_->src.id,
                datagram_->src.alias);
            return nullptr;
        }
        return space;
    }

    /** Returns the read/write length from byte 6 or 7 of the incoming
     * datagram, or -1 if the incoming datagram is of incorrect format. */
    int get_length()
    {
        const uint8_t* bytes = in_bytes();
        int len = datagram_->payload->used();
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
            LOG(WARNING, "MemoryConfig::read_len: Incoming datagram not long "
                         "enough. command=0x%02x, length=%d. Source "
                         "{0x%012llx, %03x}",
                cmd, len, datagram_->src.id, datagram_->src.alias);
            return -1;
        }
        return bytes[ofs];
    }

    /** Returns the address from the incoming datagram. Assumes that length >=
     * 6*/
    address_t get_address()
    {
        const uint8_t* bytes = in_bytes();
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
        uint8_t* resp_bytes = out_bytes();
        const uint8_t* bytes = in_bytes();
        memcpy(resp_bytes + 2, bytes + 2, 4);
        if (has_custom_space())
        {
            resp_bytes[6] = bytes[6];
        }
        else
        {
            resp_bytes[1] |= (bytes[1] & ~MemoryConfig::COMMAND_MASK);
        }
    }

    /// Sets the address in the response payload buffer.
    void set_address(address_t address)
    {
        uint8_t* bytes = out_bytes();
        bytes[5] = address & 0xff;
        address >>= 8;
        bytes[4] = address & 0xff;
        address >>= 8;
        bytes[3] = address & 0xff;
        address >>= 8;
        bytes[2] = address & 0xff;
    }

    /// @Returns the response datagram payload buffer.
    uint8_t* out_bytes()
    {
        return static_cast<uint8_t*>(response_->start());
    }

    /// @Returns the request datagram payload buffer.
    const uint8_t* in_bytes()
    {
        return static_cast<const uint8_t*>(datagram_->payload->start());
    }

    Buffer* response_; //< reply payload to send back.
    DatagramClient* responseFlow_;

    NodeID lockNode_; //< Holds the node ID that locked us.

    Registry registry_;         //< holds the known memory spaces
    Node* registeredNode_; //< we registered as a datagram handler for
                                // this node. May be null.
};

} // namespace nmranet

#endif // _NMRANET_MEMORYCONFIG_HXX_
