/** @copyright
 * Copyright (c) 2025, Stuart Baker
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
 * @file MemorySpace.hxx
 *
 * Logic for interacting with memory spaces.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_MEMORYSPACE_HXX_
#define _TRACTION_MODEM_MEMORYSPACE_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "traction_modem/Link.hxx"
#include "utils/Singleton.hxx"

namespace traction_modem
{

/// Shared base class for the implementation proxies memory space read and write
/// requests over the modem interface to the decoder using the matching
/// messages. The address space is 1:1 mapping, while the memory space number
/// is specified by a virtual function and thus can be translated.
class MemorySpace : public openlcb::MemorySpace
                  , public PacketFlowInterface
                  , public LinkStatusInterface
{
public:
    /// Write data to the address space. Called by the memory config service for
    /// incoming write requests.
    /// @param destination memory space offset address to write to
    /// @param data data to write. The callee may capture and begin processing
    ///        this data but should only acount for what it has captured in the
    ///        return value once it has been successfully written, and not just
    ///        "in process".
    /// @param len length of write data in bytes
    /// @param error The output argument for the error code. If the operation
    ///        succeeded, sets *error to zero. If the operation failed, sets
    ///        *error to non-zero. If the operation needs to be continued, then
    ///        sets error to ERROR_AGAIN, and later calls the Notifiable when
    ///        ready to continue. The caller preset *error to zero, such that
    ///        the callee is not required to also set it to zero upon "success".
    /// @param again Used only when *error was set to ERROR_AGAIN. Calls
    ///        again->notify() when the operation is ready to be continued. The
    ///        caller should then call the write again, with the offset (source)
    ///        adjusted with the previously returned bytes.
    /// @return the number of bytes successfully written (before hitting the end
    ///         of the space)
    size_t write(address_t destination, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) override
    {
        if (destination < min_address() || destination > max_address())
        {
            // The write is starting outside the bounds of the memory space.
            *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            evaluate_error(*error);
            return 0;
        }
        if (state_ == DONE)
        {
            // The write is completed, return the results.
            state_ = IDLE;
            *error = error_;
            evaluate_error(*error);
            return size_;
        }
        if (!link_->is_link_up())
        {
            // Link is down, return error.
            LOG(INFO, "traction_modem::MemorySpace: write() link is not up.");
            *error = openlcb::Defs::ERROR_TEMPORARY;
            return 0;
        }
        HASSERT(state_ == IDLE);
        // Clamp the write length to the max supported by the modem.
        if (len > Defs::MAX_WRITE_DATA_LEN)
        {
            len = Defs::MAX_WRITE_DATA_LEN;
        }
        // Register for a write response and send the read request.
        link_->get_rx_iface()->register_handler(this, Defs::RESP_MEM_W);
        // Link is up, we can send the write request.
        link_->get_tx_iface()->send_packet(
            Defs::get_memw_payload(get_space_id(), destination, data, len));
        // Start the supervisor timer.
        timer_.start(openlcb::DatagramDefs::timeout_from_flags_nsec(
            get_write_timeout()));
        // Indicate to the caller we must be called again.
        *error = ERROR_AGAIN;
        done_ = again;
        state_ = PENDING;
        return 0;
    }

    /// Read data from the address space. Called by the memory config service
    /// for incoming read requests.
    /// @param source memory space offset address to read from
    /// @param dst location to fill in with read data
    /// @param len length of requested read data in bytes
    /// @param error The output argument for the error code. If the operation
    ///        succeeded, sets *error to zero. If the operation failed, sets
    ///        *error to non-zero. If the operation needs to be continued, then
    ///        sets error to ERROR_AGAIN, and later calls the Notifiable when
    ///        ready to continue. The caller preset *error to zero, such that
    ///        the callee is not required to also set it to zero upon "success".
    /// @param again Used only when *error was set to ERROR_AGAIN. Calls
    ///        again->notify() when the operation is ready to be continued. The
    ///        caller should then call the read again, with the offset (source)
    ///        adjusted with the previously returned bytes.
    /// @return the number of bytes successfully read (before hitting the end of
    ///         the space)
    size_t read(address_t source, uint8_t *dst, size_t len,
                errorcode_t *error, Notifiable *again) override
    {
        if (source < min_address() || source > max_address())
        {
            // The read is starting outside the bounds of the memory space.
            *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            evaluate_error(*error);
            return 0;
        }
        if (state_ == DONE)
        {
            // The read is completed, return the results.
            state_ = IDLE;
            *error = error_;
            evaluate_error(*error);
            return size_;
        }
        if (!link_->is_link_up())
        {
            // Link is down, return error.
            LOG(INFO, "traction_modem::MemorySpace: read() link is not up.");
            *error = openlcb::Defs::ERROR_TEMPORARY;
            return 0;
        }
        HASSERT(state_ == IDLE);
        // Clamp the read length to the max supported by the modem.
        if (len > Defs::MAX_READ_DATA_LEN)
        {
            len = Defs::MAX_READ_DATA_LEN;
        }
        // Register for a read response and send the read request.
        link_->get_rx_iface()->register_handler(this, Defs::RESP_MEM_R);
        // Send the read request.
        link_->get_tx_iface()->send_packet(
            Defs::get_memr_payload(get_space_id(), source, len));
        // Start the supervisor timer.
        timer_.start(openlcb::DatagramDefs::timeout_from_flags_nsec(
            get_read_timeout()));
        // Indicate to the caller we must be called again.
        *error = ERROR_AGAIN;
        done_ = again;
        rdData_ = dst;
        // The size is saved so that we don't overrun the provided buffer in
        // case we get back an unexpected size of data from our read. This is
        // for defensive coding purposes.
        size_ = len;
        state_ = PENDING;
        return 0;
    }

protected:
    /// Constructor.
    /// @param service Service instance to bind this flow to.
    /// @param link reference to the link object.
    MemorySpace(Service *service, Link *link)
        : link_(link)
        , timer_(this, service)
        , state_(IDLE)
    {
        link_->register_link_status(this);
    }

    ~MemorySpace()
    {
        link_->unregister_link_status(this);
    }

    /// Pass error results down to derived objects. This gives a derived object
    /// an opportunity to track the error state over a series of operations.
    /// @param error error code to pass down for evaluation
    virtual void evaluate_error(errorcode_t error)
    {
    }

    Link *link_; ///< reference to the link object

private:
    /// Internal state.
    enum State
    {
        IDLE, ///< waiting for a new request
        PENDING, ///< request in progress
        DONE ///< previous request is finished
    };

    /// Get the space ID that will be used over the modem interface. This is
    /// the space number that the server in the decoder will see.
    /// @return space id
    virtual uint8_t get_space_id() = 0;

    /// Receive for read and write responses.
    /// @param buf incoming message
    /// @param prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        auto rb = get_buffer_deleter(buf);
        switch (rb->data()->command())
        {
            case Defs::RESP_MEM_W:
            {
                // This is a write, decode the response, save the error code,
                // and set the size_ that will get returned appropriately.
                Defs::WriteResponse *wr =
                    (Defs::WriteResponse*)rb->data()->payload.data();
                error_ = rb->data()->response_status();
                switch (error_)
                {
                    default:
                        // Fall through.
                    case openlcb::MemoryConfigDefs::ERROR_WRITE_TO_RO:
                        // Fall through.
                    case openlcb::MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN:
                        size_ = 0;
                        break;
                    case openlcb::Defs::ErrorCodes::ERROR_CODE_OK:
                        // Fall through.
                    case openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS:
                        size_ = be16toh(wr->bytesWritten_);
                        break;
                }
                // We got the expected response, unregister so that unexpected
                // responses fall on the floor.
                break;
            }
            case Defs::RESP_MEM_R:
            {
                // This is a read, decode the response, save the error code,
                // and set the size_ that will get returned appropriately.
                Defs::ReadResponse *rr =
                    (Defs::ReadResponse*)rb->data()->payload.data();
                error_ = rb->data()->response_status();
                switch (error_)
                {
                    default:
                        // Fall through.
                    case openlcb::MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN:
                        size_ = 0;
                        break;
                    case openlcb::Defs::ErrorCodes::ERROR_CODE_OK:
                        // Fall through.
                    case openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS:
                    {
                        size_t read_size =
                            rb->data()->length() - sizeof(rr->error_);
                        // Defensive coding to prevent memory corruption.
                        size_ = std::min(size_, read_size);
                        memcpy(rdData_, rr->data_, size_);
                        break;
                    }
                }
                // We got the expected response, unregister so that unexpected
                // responses fall on the floor.
                break;
            }
        }
        // Trigger the supervising timer to expire early because we have the
        // result.
        link_->get_rx_iface()->unregister_handler_all(this);
        timer_.ensure_triggered();
    }

    /// Timeout supervisor for the memory transaction.
    class Timeout : public Timer
    {
    public:
        /// Constructor.
        /// @param parent parent MemorySpace object
        /// @param service Service instance to bind this flow to
        Timeout(MemorySpace *parent, Service *service)
            : Timer(service->executor()->active_timers())
            , parent_(parent)
        {
        }

    private:
        /// Timer expiration callback. We will get here if either there is a
        /// timeout waiting for a response or if a valid response has come in
        /// and we can move on.
        /// @return NONE
        long long timeout() override
        {
            if (!is_triggered())
            {
                // Timed out. Unregister the handlers so successive responses
                // fall on the floor.
                LOG(VERBOSE, "Timer expired");
                parent_->link_->get_rx_iface()->unregister_handler_all(parent_);
                parent_->size_ = 0;
                parent_->error_ = openlcb::Defs::ERROR_OPENLCB_TIMEOUT;
            }
            // Notify our caller so that the results can be provided.
            parent_->state_ = DONE;
            parent_->done_->notify();
            parent_->done_ = nullptr; // Defensive coding.
            return NONE;
        };

        MemorySpace *parent_; ///< parent object
    } timer_;

    /// If this is not empty, it means the link is down when we tried to
    /// transmit the message. If the link comes up and this is not empty, it
    /// should be transmitted. If a timeout occurs, it shall be cleared.
    Notifiable *done_; ///< Notifiable for the memory operation to continue.
    uint8_t *rdData_; ///< read data pointer
    size_t size_; ///< requested size
    State state_; ///< current request state
    errorcode_t error_; ///< error code returned by the decoder

    /// Allow access from child timer object.
    friend class timeout;
};

/// Memory space for DCC CVs.
class CvSpace : public MemorySpace
{
public:
    /// Memory space number for CVs
    static constexpr uint8_t SPACE_ID = openlcb::MemoryConfigDefs::SPACE_DCC_CV;

    /// Constructor.
    /// @param service Service instance to bind this flow to
    /// @param link reference to the link object.
    CvSpace(
        Service *service, Link *link)
        : MemorySpace(service, link)
    {
    }

private:
    /// Test if the memory space is read only.
    /// @return false
    bool read_only() override
    {
        return false;
    }

    /// Get the largest supported address.
    /// @return 1023 (max CV address)
    address_t max_address() override
    {
        return (0x1 << 24) - 1;
    };

    /// Get the space ID that will be used over the modem interface.
    /// @return space id
    uint8_t get_space_id() override
    {
        return SPACE_ID;
    }
};

/// Memory space for firmware updates.
class FirmwareSpace : public MemorySpace
{
public:
    /// Memory space number for CVs
    static constexpr uint8_t SPACE_ID =
        openlcb::MemoryConfigDefs::SPACE_FIRMWARE;

    /// Constructor.
    /// @param service Service instance to bind this flow to
    /// @param link reference to the link object.
    FirmwareSpace(
        Service *service, Link *link)
        : MemorySpace(service, link)
    {
    }

    /// Test if the memory space is read only.
    /// @return false
    bool read_only() override
    {
        return false;
    }

    /// Get the largest supported address.
    /// @return 1023 (max CV address)
    address_t max_address() override
    {
        return UINT32_MAX;
    };

    /// Get the space ID that will be used over the modem interface.
    /// @return space id
    uint8_t get_space_id() override
    {
        return SPACE_ID;
    }

    /// Reboot into bootloader request.
    /// @return openlcb::Defs::ErrorCodes::ERROR_CODE_OK
    errorcode_t freeze() override
    {
        trackedError_ = openlcb::Defs::ErrorCodes::ERROR_CODE_OK;
        link_->get_tx_iface()->send_packet(
            Defs::get_reboot_payload(Defs::RebootArg::BOOT));
        return openlcb::Defs::ErrorCodes::ERROR_CODE_OK;
    }

    /// Reboot into application with full validation request.
    /// @return openlcb::Defs::ErrorCodes::ERROR_CODE_OK
    errorcode_t unfreeze() override
    {
        link_->get_tx_iface()->send_packet(
            Defs::get_reboot_payload(Defs::RebootArg::APP_VALIDATE));
        return trackedError_;
    }

private:
    /// Pass error results down to derived objects. This gives a derived object
    /// an opportunity to track the error state over a series of operations.
    /// @param error error code to pass down for evaluation
    void evaluate_error(errorcode_t error) override
    {
        if (trackedError_ == openlcb::Defs::ErrorCodes::ERROR_CODE_OK)
        {
            // Capture the first error occurrence.
            trackedError_ = error;
        }
    }

    errorcode_t trackedError_; ///< tracks errors during an update sequence
};

} // namespace traction_modem

#endif // _TRACTIONMODEM_MEMORYSPACE_HXX_
