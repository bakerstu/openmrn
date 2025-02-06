/** @copyright
 * Copyright (c) 2024, Balazs Racz
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
 * @file TractionModem.hxx
 *
 * Class for interacting with the decoder chip via the traction modem protocol.
 *
 * @author Balazs Racz
 * @date 9 Feb 2024
 */

#ifndef _TRACTION_MODEM_TRACTIONMODEM_HXX_
#define _TRACTION_MODEM_TRACTIONMODEM_HXX_

/// @todo Need to prune out the "hardware.hxx" dependencies from this file.
///       These need to be dispatched to hardware specific code somehow.

#include "traction_modem/Defs.hxx"
#if !defined(GTEST)
#include "hardware.hxx"
#endif

#include "executor/Dispatcher.hxx"
#include "executor/StateFlow.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/TrainInterface.hxx"
#include "utils/format_utils.hxx"

#include "traction_modem/ModemTrain.hxx"

namespace traction_modem
{

class CvSpace : public openlcb::MemorySpace, public PacketFlowInterface
{
public:
    CvSpace(PacketFlowInterface *tx)
        : pendingRead_(false)
        , doneRead_(false)
        , pendingWrite_(false)
        , doneWrite_(false)
        , txFlow_(tx)
    { }

    address_t max_address() override
    {
        return 1023;
    }

    /// @returns whether the memory space does not accept writes.
    bool read_only() override
    {
        return false;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
        errorcode_t *error, Notifiable *again) override
    {
        if (doneWrite_)
        {
            doneWrite_ = false;
            *error = errorCode_;
            return actualLen_;
        }

        actualLen_ = len;
        pendingWrite_ = true;
        doneWrite_ = false;
        done_ = again;
        *error = ERROR_AGAIN;

        auto *b = txFlow_->alloc();
        b->data()->payload =
            Defs::get_memw_payload(proxySpace_, destination, data, len);
        txFlow_->send(b);
        return 0;
    }

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
        Notifiable *again) override
    {
        if (doneRead_)
        {
            doneRead_ = false;
            *error = errorCode_;
            return actualLen_;
        }

        readBuf_ = dst;
        actualLen_ = len;
        pendingRead_ = true;
        doneRead_ = false;
        done_ = again;
        *error = ERROR_AGAIN;

        auto *b = txFlow_->alloc();
        b->data()->payload = Defs::get_memr_payload(proxySpace_, source, len);
        txFlow_->send(b);
        return 0;
    }

    /// Handles messages coming back from the decoder via the traction modem
    /// protocol.
    void send(Buffer<Message>* buf, unsigned prio) override {
        auto rb = get_buffer_deleter(buf);
        auto& txm = *buf->data();
        if (!txm.valid()) {
            return;
        }
        switch (txm.command())
        {
            case Defs::RESP_MEM_R:
            {
                if (pendingRead_)
                {
                    handle_read_response(txm);
                }
                break;
            }
            case Defs::RESP_MEM_W:
            {
                if (pendingWrite_)
                {
                    handle_write_response(txm);
                }
                break;
            }
        }
    }

    void handle_read_response(Message &txm)
    {
        doneRead_ = true;
        pendingRead_ = false;
        errorCode_ = txm.response_status();
        unsigned data_bytes = txm.length() - 2;
        if (data_bytes > actualLen_)
        {
            // We should not have received more bytes back than we requested,
            // but we still clip.
            data_bytes = actualLen_;
        }
        memcpy(readBuf_, txm.payload.data() + Defs::OFS_DATA + 2, data_bytes);
        actualLen_ = data_bytes;
        done_->notify();
    }

    void handle_write_response(Message& txm) {
        pendingWrite_ = false;
        doneWrite_ = true;
        errorCode_ = txm.response_status();
        if (errorCode_)
        {
            actualLen_ = Defs::get_uint16(txm.payload, Defs::OFS_DATA + 2);
        }
        done_->notify();
    }

    /// This is the memory space we will be using on the decoder.
    static constexpr uint8_t proxySpace_ = openlcb::MemoryConfigDefs::SPACE_DCC_CV;

    /// true if we are waiting for a read response
    bool pendingRead_ : 1;
    /// true if we the read response arrived
    bool doneRead_ : 1;
    /// true if we are waiting for a write response
    bool pendingWrite_ : 1;
    /// true if we the write response arrived
    bool doneWrite_ : 1;

    /// Returned error code from the backend.
    uint16_t errorCode_ = 0;

    /// Where to put the bytes read.
    uint8_t* readBuf_ = nullptr;
    /// How many bytes to put there. When doneRead_, then the number of bytes
    /// actually read.
    unsigned actualLen_ = 0;

    /// Notifiable to mark when the pending read/write completes.
    Notifiable* done_ = nullptr;

    /// We send outgoing packets to the decoder using this interface.
    PacketFlowInterface* txFlow_;
};


} // namespace traction_modem

#endif // _TRACTION_MODEM_TRACTIONMODEM_HXX_
