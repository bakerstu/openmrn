/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file LogonModule.hxx
 * Default implementation of a storage and policy module for automatic logon.
 *
 * @author Balazs Racz
 * @date 14 Aug 2021
 */

#ifndef _DCC_LOGONMODULE_HXX_
#define _DCC_LOGONMODULE_HXX_

#include <map>
#include <vector>

#include "dcc/Defs.hxx"
#include "dcc/Logon.hxx"

namespace dcc
{

/// Default implementation of the storage and policy module for trains.
template<class Base>
class ParameterizedLogonModule : public LogonHandlerModule
{
public:
    /// We store this structure about each locomotive.
    struct LocoInfo : public Base::Storage
    {
        /// State machine flags about this loco.
        uint8_t flags_ {0};

        /// The assigned DCC address. The encoding is in the S-9.2.1.1 format.
        /// The default value is an invalid address causing an error on the
        /// locomotive.
        uint16_t assignedAddress_ {Defs::ADR_INVALID};

        /// 44-bit decoder unique ID.
        uint64_t decoderId_;
    };

    std::vector<LocoInfo> locos_;
    std::map<uint64_t, uint16_t> ids_;

    /// @return the number of locomotives known. The locomotive IDs are
    /// 0..num_locos() - 1.
    unsigned num_locos()
    {
        return locos_.size();
    }

    /// @param loco_id a locomotive identifier
    /// @return true if this is valid and belongs to a loco we know about.
    bool is_valid_loco_id(unsigned loco_id)
    {
        return loco_id < num_locos();
    }

    /// Finds the storage cell for a locomotive and returns the flag byte for
    /// it.
    /// @param loco_id a valid locomotive ID.
    /// @return the flag byte for this loco.
    uint8_t &loco_flags(unsigned loco_id)
    {
        return locos_[loco_id].flags_;
    }

    /// Retrieves the decoder unique ID.
    /// @param loco_id the dense locomotive identifier.
    /// @return the decoder unique ID (44 bit, LSb-aligned).
    uint64_t loco_did(unsigned loco_id)
    {
        return locos_[loco_id].decoderId_;
    }

    /// Creates a new locomotive by decoder ID, or looks up an existing
    /// locomotive by decoder ID.
    /// @param decoder_id 44-bit decoder ID (aligned to LSb).
    /// @return locomotive ID for this cell.
    unsigned create_or_lookup_loco(uint64_t decoder_id)
    {
        auto it = ids_.find(decoder_id);
        if (it == ids_.end())
        {
            // create new.
            uint16_t lid = locos_.size();
            locos_.emplace_back();
            locos_[lid].decoderId_ = decoder_id;
            ids_[decoder_id] = lid;
            return lid;
        }
        else
        {
            return it->second;
        }
    }

    /// Runs the locomotive address policy. After the address policy is run,
    /// the loco should have the ability to answer the assigned_address
    /// question.
    /// @param loco_id which locomotive this is
    /// @param desired_address the S-9.2.1.1 encoded desired address for this
    /// decoder.
    void run_address_policy(unsigned loco_id, uint16_t desired_address)
    {
        /// @todo support accessory decoders.

        // Note: we ignore the desired address and start assigning addresses
        // from 10000 and up.
        locos_[loco_id].assignedAddress_ = nextAddress_++;
    }

    /// @param loco_id
    /// @return the address to be assigned to this locomotive. 14-bit.
    uint16_t assigned_address(unsigned loco_id)
    {
        return locos_[loco_id].assignedAddress_;
    }

    /// Invoked when the address assignment completes for a decoder.
    /// @param loco_id which decoder.
    void assign_complete(unsigned loco_id)
    {
        loco_flags(loco_id) |= LogonHandlerModule::FLAG_COMPLETE;
    }

    uint16_t nextAddress_ {(Defs::ADR_MOBILE_LONG << 8) + 10000};

}; // class ParameterizedLogonModule

class DefaultBase {
public:
    struct Storage {};
};

using DefaultLogonModule = ParameterizedLogonModule<DefaultBase>;

} // namespace dcc

#endif //  _DCC_LOGONMODULE_HXX_
