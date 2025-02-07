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
#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"
#include "utils/Singleton.hxx"

namespace traction_modem
{

/// Abstract interface for traction modem memory spaces.
class MemorySpace : public openlcb::MemorySpace, public RxFlowBase
{
protected:
    /// Constructor.
    /// @param service Service instance to bind this flow to.
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    MemorySpace(
        Service *service, TxFlowInterface *tx_flow, RxFlowInterface *rx_flow)
        : RxFlowBase(service)
        , timer_(this, service)
        , txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , state_(IDLE)
    {
    }

private:
    /// Internal state.
    enum State
    {
        IDLE, ///< waiting for a new request
        PENDING, ///< request in progress
        DONE ///< previous request is finished
    };

    /// Get the space ID that will be used over the modem interface.
    /// @return space id
    virtual uint8_t get_space_id() = 0;

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
            *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        if (state_ == DONE)
        {
            state_ = IDLE;
            if (size_ < len)
            {
                *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            }
            return size_;
        }
        HASSERT(state_ == IDLE);
        HASSERT(is_terminated());
        rxFlow_->register_handler(this, Defs::RESP_MEM_W);
        txFlow_->send_packet(Defs::get_memw_payload(
            get_space_id(), destination, data, len));

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
            *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            return 0;
        }
        if (state_ == DONE)
        {
            state_ = IDLE;
            if (size_ < len)
            {
                *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
            }
            return size_;
        }
        HASSERT(state_ == IDLE);
        HASSERT(is_terminated());
        rxFlow_->register_handler(this, Defs::RESP_MEM_R);
        txFlow_->send_packet(Defs::get_memr_payload(
            get_space_id(), source, len));
        *error = ERROR_AGAIN;
        done_ = again;
        rdData_ = dst;
        size_ = len;
        state_ = PENDING;
        return 0;
    }

    Action entry()
    {
        Defs::Message *m =
            (Defs::Message*)message()->data()->payload.data();
        switch (be16toh(m->header_.command_))
        {
            case Defs::RESP_MEM_W:
            {
                Defs::WriteResponse *wr =
                    (Defs::WriteResponse*)message()->data()->payload.data();
                if (be16toh(wr->error_) !=
                    openlcb::Defs::ErrorCodes::ERROR_CODE_OK)
                {
                    size_ = be16toh(wr->length_);
                }
                break;
            }
            case Defs::RESP_MEM_R:
            {
                Defs::ReadResponse *rr =
                    (Defs::ReadResponse*)message()->data()->payload.data();
                size_t read_size =
                    be16toh(rr->header_.length_) - sizeof(rr->error_);
                switch (be16toh(rr->error_))
                {
                    default:
                    case openlcb::Defs::ErrorCodes::ERROR_TEMPORARY + 1:
                        read_size = 0;
                        break;
                    case openlcb::Defs::ErrorCodes::ERROR_TEMPORARY:
                        break;
                    case openlcb::Defs::ErrorCodes::ERROR_CODE_OK:
                        break;
                }
                size_ = std::min(size_, read_size);
                memcpy(rdData_, rr->data_, size_);
                break;
            }
        }
        timer_.ensure_triggered();
        return release_and_exit();
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
        /// Timer expiration callback.
        /// @return NONE
        long long timeout() override
        {
            if (!is_triggered())
            {
                // Expired early, unregister the handlers.
                parent_->rxFlow_->unregister_handler(parent_, Defs::RESP_MEM_W);
                parent_->rxFlow_->unregister_handler(parent_, Defs::RESP_MEM_R);
                parent_->size_ = 0;
            }
            parent_->state_ = DONE;
            parent_->done_->notify();
           return NONE;
        };

        /// parent object
        MemorySpace *parent_;
    } timer_;

    TxFlowInterface *txFlow_; ///< reference to the transmit flow
    RxFlowInterface *rxFlow_; ///< reference to the receive flow

    Notifiable *done_; ///< Notifiable for the memory operation to continue.
    uint8_t *rdData_; ///< read data pointer
    size_t size_; ///< requested size
    State state_; ///< current request state

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
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    CvSpace(
        Service *service, TxFlowInterface *tx_flow, RxFlowInterface *rx_flow)
        : MemorySpace(service, tx_flow, rx_flow)
    {
    }

private:
    /// Test if the memory space is read only.
    /// @return true
    bool read_only() override
    {
        return true;
    }

    /// Get the largest supported address.
    /// @return 1023 (max CV address)
    address_t max_address() override
    {
        return 1023;
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
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    FirmwareSpace(
        Service *service, TxFlowInterface *tx_flow, RxFlowInterface *rx_flow)
        : MemorySpace(service, tx_flow, rx_flow)
    {
    }

private:
    /// Test if the memory space is read only.
    /// @return true
    bool read_only() override
    {
        return true;
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
};

} // namespace traction_modem

#endif // _TRACTIONMODEM_MEMORYSPACE_HXX_