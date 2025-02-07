/** @copyright
 * Copyright (c) 2025, Stuart Baker
 * All rights reserved
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
#include "traction_modem/ModemTrain.hxx"
#include "utils/Singleton.hxx"

namespace traction_modem
{

/// MemorySpace operation request.
struct MemorySpaceClientRequest : public CallableFlowRequestBase
{
    enum Write
    {
        WRITE,
    };

    enum Read
    {
        READ,
    };

    void reset(Write, uint32_t address, const uint8_t *data, size_t size)
    {
        reset_base();
        cmd_ = Type::WRITE;
        address_ = address;
        wrData_ = data;
        size_ = size;
    }

    void reset(Read, uint32_t address, uint8_t *data, size_t size)
    {
        reset_base();
        cmd_ = Type::WRITE;
        address_ = address;
        rdData_ = data;
        size_ = size;
    }

    /// Type of request.
    enum class Type
    {
        WRITE, ///< write request
        READ, ///< read request
    };

    Type cmd_; ///< request type
    uint32_t address_; ///< start address offset
    union
    {
        const uint8_t *wrData_; ///< write data pointer
        uint8_t *rdData_; ///< read data pointer
    };
    size_t size_; ///< requested size on request, actual size on return
    uint8_t space_; ///< memory space id
};

class MemorySpaceClientFlow : public CallableFlow<MemorySpaceClientRequest>
                            , public Singleton<MemorySpaceClientFlow>
{
public:
    /// Constructor.
    /// @param service Service instance to bind this flow to.
    /// @param train train instance
    MemorySpaceClientFlow(Service *service, ModemTrain *train)
        : CallableFlow(service)
        , timer_(this)
        , train_(train)
    {
        //train->register_handler(this, Defs::RESP_MEM_W);
    }

    /// Error codes.
    enum Error
    {
        ERROR_OK = 0, ///< no error
        ERROR_TIMEOUT ///< timeout occurred
    };

private:

    Action entry() override
    {
        switch (request()->cmd_)
        {
            case MemorySpaceClientRequest::Type::WRITE:
                train_->register_handler(&rxFlow_, Defs::RESP_MEM_W);
                train_->send_packet(Defs::get_memw_payload(request()->space_,
                    request()->address_, request()->wrData_, request()->size_));
                return sleep_and_call(
                    &timer_, SEC_TO_NSEC(1), STATE(write_done));
            case MemorySpaceClientRequest::Type::READ:
                train_->register_handler(&rxFlow_, Defs::RESP_MEM_R);
                train_->send_packet(Defs::get_memr_payload(request()->space_,
                    request()->address_, request()->size_));
                return sleep_and_call(
                    &timer_, SEC_TO_NSEC(1), STATE(read_done));
        }
        return return_with_error(openlcb::Defs::ERROR_UNIMPLEMENTED_CMD);
    }

    Action write_done()
    {
        train_->unregister_handler(&rxFlow_, Defs::RESP_MEM_W);
        int error = timer_.is_triggered() ? ERROR_OK : ERROR_TIMEOUT;
        return return_with_error(error);
    }

    Action read_done()
    {
        train_->unregister_handler(&rxFlow_, Defs::RESP_MEM_R);
        int error = timer_.is_triggered() ? ERROR_OK : ERROR_TIMEOUT;
        return return_with_error(error);
    }

    /// Flow for receiving the reply message.
    class ReplyFlow : public RxFlowBase
    {
    public:
        /// Constructor.
        /// @param service Service instance to bind this flow to.
        ReplyFlow(MemorySpaceClientFlow *parent)
            : RxFlowBase(parent_->service())
            , parent_(parent)
        {
        }

    private:
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
                        parent_->request()->size_ = be16toh(wr->length_);
                    }
                    break;
                }
                case Defs::RESP_MEM_R:
                {
                    Defs::ReadResponse *rr =
                        (Defs::ReadResponse*)message()->data()->payload.data();
                    switch (be16toh(rr->error_))
                    {
                        default:
                        case openlcb::Defs::ErrorCodes::ERROR_TEMPORARY:
                            parent_->request()->size_ =
                                be16toh(rr->header_.length_) -
                                sizeof(rr->error_);
                            break;
                        case openlcb::Defs::ErrorCodes::ERROR_TEMPORARY + 1:
                            parent_->request()->size_ = 0;
                            break;                        
                        case openlcb::Defs::ErrorCodes::ERROR_CODE_OK:
                            break;
                    }
                    memcpy(parent_->request()->rdData_,
                        rr->data_, parent_->request()->size_);
                    break;
                }
            }
            parent_->timer_.ensure_triggered();
            return release_and_exit();
        }

        MemorySpaceClientFlow *parent_; ///< parent to this object
    } rxFlow_{this};

    /// Timer helper to supported timed waits.
    StateFlowTimer timer_;

    /// Parent train object that contains the dispatcher.
    ModemTrain *train_;

    /// Allow access from child reply flow.
    friend class ReplyFlow;
};

/// Abstract interface for traction modem memory spaces.
class MemorySpace : public openlcb::MemorySpace, public StateFlowBase
{
protected:
    /// Constructor.
    /// @param service Service instance to bind this flow to.
    MemorySpace(Service *service)
        : StateFlowBase(service)
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
        start_flow(STATE(do_write));
        *error = ERROR_AGAIN;
        done_ = again;
        address_ = destination;
        wrData_ = data;
        size_ = len;
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
        start_flow(STATE(do_read));
        *error = ERROR_AGAIN;
        done_ = again;
        address_ = source;
        rdData_ = dst;
        size_ = len;
        state_ = PENDING;
        return 0;
    }

    Action do_write()
    {
        return invoke_subflow_and_wait(
            MemorySpaceClientFlow::instance(), STATE(done),
            MemorySpaceClientRequest::WRITE, address_, wrData_, size_);
    }

    Action do_read()
    {
        return invoke_subflow_and_wait(
            MemorySpaceClientFlow::instance(), STATE(done),
            MemorySpaceClientRequest::READ, address_, rdData_, size_);
    }

    Action done()
    {
        auto b = get_buffer_deleter(
            full_allocation_result(MemorySpaceClientFlow::instance()));
        if (b->data()->resultCode == MemorySpaceClientFlow::ERROR_TIMEOUT)
        {
            size_ = 0;
        }
        size_ = b->data()->size_; // Get the actual consumed size.
        state_ = DONE;
        done_->notify();
        return exit();
    }

    /// Get the proxy space ID that will be used as a proxy.
    virtual uint8_t get_proxy_space_id() = 0;

    Notifiable *done_; ///< Notifiable for the memory operation to continue.
    uint32_t address_; ///< start address offset
    union
    {
        const uint8_t *wrData_; ///< write data pointer
        uint8_t *rdData_; ///< read data pointer
    };
    size_t size_; ///< requested size
    State state_; ///< current request state
};

/// Memory space for DCC CVs.
class NewCvSpace : public MemorySpace
{
public:
    /// Memory space number for CVs
    static constexpr uint8_t SPACE_ID = openlcb::MemoryConfigDefs::SPACE_DCC_CV;

    /// Constructor.
    /// @param service Service instance to bind this flow to
    NewCvSpace(Service *service)
        : MemorySpace(service)
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

    uint8_t get_proxy_space_id() override
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
    FirmwareSpace(Service *service)
        : MemorySpace(service)
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

    uint8_t get_proxy_space_id() override
    {
        /// @todo Need to revisit this proxy number
        return 0x59;
    }
};

} // namespace traction_modem

#endif // _TRACTIONMODEM_MEMORYSPACE_HXX_