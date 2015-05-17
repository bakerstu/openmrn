/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TractionCvSpace.hxx
 *
 * A memory config protocol compatible Memory Space for reading and writing CVs
 * of DCC locomotives.
 *
 * @author Balazs Racz
 * @date 16 May 2015
 */

#ifndef _NMRANET_TRACTIONCVSPACE_HXX_
#define _NMRANET_TRACTIONCVSPACE_HXX_

#include "nmranet/MemoryConfig.hxx"
#include "executor/StateFlow.hxx"
#include "dcc/PacketFlowInterface.hxx"
#include "dcc/RailCom.hxx"
#include "dcc/RailcomHub.hxx"

namespace nmranet
{

class TractionCvSpace : private MemorySpace, private dcc::RailcomHubPortInterface, public StateFlowBase
{
public:
    TractionCvSpace(MemoryConfigHandler *parent,
        dcc::PacketFlowInterface *track, dcc::RailcomHubFlow *railcom_hub);

    ~TractionCvSpace();
private:
    static const unsigned MAX_CV = 255;

    bool set_node(Node* node) OVERRIDE;

    bool read_only() OVERRIDE { return false; }

    address_t max_address() OVERRIDE { return MAX_CV; }

    size_t write(address_t destination, const uint8_t *data, size_t len,
                 errorcode_t *error, Notifiable *again) OVERRIDE;

    size_t read(address_t source, uint8_t *dst, size_t len,
                errorcode_t *error, Notifiable *again) OVERRIDE;

    // State flow states.
    Action try_read1();
    Action fill_read1_packet();
    Action read1_returned();

    // Railcom feedback
    void send(Buffer<dcc::RailcomHubData>* b, unsigned priority) OVERRIDE;
    void record_railcom_status(unsigned code);

    MemoryConfigHandler *parent_;
    dcc::PacketFlowInterface *track_;
    dcc::RailcomHubFlow *railcomHub_;
    uint16_t dccAddress_;
    uint16_t cvNumber_;
    uint8_t cvData_;  //< data to read or write.
    uint8_t errorCode_ : 3;
    enum {
        ERROR_NOOP = 0,
        ERROR_PENDING,
        ERROR_OK,
        ERROR_BUSY,
        ERROR_NO_RAILCOM_CH2_DATA,
        ERROR_GARBAGE,
        ERROR_UNKNOWN_RESPONSE
    };
    Notifiable* done_;  //< notify when transfer is done
    StateFlowTimer timer_;
};

} // namespace nmranet

#endif // _NMRANET_TRACTIONCVSPACE_HXX_
