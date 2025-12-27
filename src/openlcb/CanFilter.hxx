/** \copyright
 * Copyright (c) 2024, Balazs Racz
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
 * \file CanFilter.hxx
 *
 * Filter class for routing CAN frames.
 *
 * @author Balazs Racz
 * @date 27 Jan 2024
 */

#ifndef _OPENLCB_CANFILTER_HXX_
#define _OPENLCB_CANFILTER_HXX_

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>

#include "openlcb/CanDefs.hxx"
#include "os/OS.hxx"
#include "utils/Hub.hxx"

namespace openlcb
{

/**
 * Filter class for routing CAN frames.
 *
 * This class stores a table keyed by the source Alias of a CAN frame, and
 * stores the identifier of the CanHubPort from where this packet came from.
 * This table is used to perform routing decisions on outgoing CAN frames.
 *
 * When there is an Alias to which a CAN frame is targeted, the frame will only
 * be sent to the port(s) registered for this specific alias.
 *
 * The prepare_packet and is_matching methods are NOT thread-safe and must be
 * called from a single StateFlow (single thread).
 * The remove_port method IS thread-safe and can be called from any thread.
 * It schedules the removal which is applied at the beginning of the next
 * prepare_packet call.
 */
class CanFilter
{
public:
    /**
     * Prepares the packet for routing.
     *
     * This method does preprocessing of the packet, potentially updating the
     * internal routing table. It decides whether the frame should be broadcast
     * or unicast, and if unicast, where it should be sent.
     *
     * @param frame The CAN hub data containing the frame and source port info.
     */
    void prepare_packet(CanHubData *frame)
    {
        // Apply pending removals
        if (hasPendingRemovals_)
        {
            apply_pending_removals();
        }

        const struct can_frame &can_frame = frame->frame();

        // Update routing table with source info
        NodeAlias src = get_source_address(can_frame);
        uintptr_t src_port = reinterpret_cast<uintptr_t>(frame->skipMember_);

        auto range = routingTable_.equal_range(src);
        bool found = false;
        for (auto it = range.first; it != range.second; ++it)
        {
            if (it->second == src_port)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            routingTable_.insert(std::make_pair(src, src_port));
        }

        // Store source port for filtering
        sourcePort_ = src_port;
        targetPorts_.clear();

        // Determine destination and routing
        if (is_broadcast(can_frame))
        {
            isBroadcast_ = true;
        }
        else
        {
            NodeAlias dst = get_destination_address(can_frame);
            if (dst == 0)
            {
                isBroadcast_ = true;
                return;
            }

            auto dst_range = routingTable_.equal_range(dst);

            if (dst_range.first != dst_range.second)
            {
                // Unicast to known destination(s)
                isBroadcast_ = false;
                for (auto it = dst_range.first; it != dst_range.second; ++it)
                {
                    targetPorts_.push_back(it->second);
                }
            }
            else
            {
                // Unicast to unknown destination -> Flood
                isBroadcast_ = true;
            }
        }
    }

    /**
     * Checks if the current frame matches the given port.
     *
     * @param port_id The identifier of the port to check.
     * @return true if the frame should be sent to this destination port.
     */
    bool is_matching(uintptr_t port_id)
    {
        // Source filtering: never send back to the source
        if (port_id == sourcePort_)
        {
            return false;
        }

        if (isBroadcast_)
        {
            return true;
        }

        for (uintptr_t target : targetPorts_)
        {
            if (target == port_id)
            {
                return true;
            }
        }

        return false;
    }

    /**
     * Removes all routing entries associated with the given port.
     * This method is thread-safe.
     * @param port_id The identifier of the port to remove.
     */
    void remove_port(uintptr_t port_id)
    {
        OSMutexLock l(&lock_);
        pendingRemovals_.push_back(port_id);
        hasPendingRemovals_ = true;
    }

private:
    /**
     * Applies pending port removals to the routing table.
     */
    void apply_pending_removals()
    {
        std::vector<uintptr_t> removals;
        {
            OSMutexLock l(&lock_);
            removals.swap(pendingRemovals_);
            hasPendingRemovals_ = false;
        }

        for (uintptr_t port_id : removals)
        {
            for (auto it = routingTable_.begin(); it != routingTable_.end();)
            {
                if (it->second == port_id)
                {
                    it = routingTable_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
    }

    /**
     * Gets the source address from the CAN frame.
     * @param frame The CAN frame.
     * @return The source NodeAlias.
     */
    NodeAlias get_source_address(const struct can_frame &frame)
    {
        uint32_t can_id = GET_CAN_FRAME_ID_EFF(frame);
        return CanDefs::get_src(can_id);
    }

    /**
     * Gets the destination address from the CAN frame.
     * @param frame The CAN frame.
     * @return The destination NodeAlias.
     */
    NodeAlias get_destination_address(const struct can_frame &frame)
    {
        uint32_t can_id = GET_CAN_FRAME_ID_EFF(frame);
        CanDefs::CanFrameType can_type = CanDefs::get_can_frame_type(can_id);

        if (can_type == CanDefs::GLOBAL_ADDRESSED)
        {
            // For MTI-based messages, if address bit is set, destination is in
            // payload. We assume this is only called if is_broadcast returned
            // false, which checks the MTI address bit. However, to be safe and
            // complete:
            Defs::MTI mti = static_cast<Defs::MTI>(CanDefs::get_mti(can_id));
            if ((mti & Defs::MTI_ADDRESS_MASK) != 0 && frame.can_dlc >= 2)
            {
                NodeAlias dst = frame.data[0] & 0x0f;
                dst = (dst << 8) | frame.data[1];
                return dst;
            }
            // Should not happen if called correctly on unicast packets, or if
            // packet is malformed.
            return 0;
        }
        else
        {
            // Datagrams and Streams have destination in the CAN ID.
            return CanDefs::get_dst(can_id);
        }
    }

    /**
     * Checks if the frame is a broadcast frame.
     * @param frame The CAN frame.
     * @return true if the frame is broadcast.
     */
    bool is_broadcast(const struct can_frame &frame)
    {
        uint32_t can_id = GET_CAN_FRAME_ID_EFF(frame);
        CanDefs::FrameType type = CanDefs::get_frame_type(can_id);

        if (type == CanDefs::CONTROL_MSG)
        {
            return true;
        }

        CanDefs::CanFrameType can_type = CanDefs::get_can_frame_type(can_id);
        Defs::MTI mti = static_cast<Defs::MTI>(CanDefs::get_mti(can_id));

        // is_broadcast is true when ((CanFrameType is GLOBAL_ADDRESSED) AND
        // (MTI & MTI_ADDRESS_MASK == 0)).
        if (can_type == CanDefs::GLOBAL_ADDRESSED &&
            (mti & Defs::MTI_ADDRESS_MASK) == 0)
        {
            return true;
        }

        return false;
    }

    /// Stores mapping from Alias to Port ID.
    std::multimap<NodeAlias, uintptr_t> routingTable_;

    /// True if the current packet should be broadcast (sent to all except
    /// source).
    bool isBroadcast_ {false};

    /// If not broadcast, contains the list of target ports.
    std::vector<uintptr_t> targetPorts_;

    /// Source port of the current packet.
    uintptr_t sourcePort_ {0};

    /// Mutex protecting the pending removals.
    OSMutex lock_;

    /// List of ports to be removed.
    std::vector<uintptr_t> pendingRemovals_;

    /// Flag indicating if there are pending removals.
    bool hasPendingRemovals_ {false};
};

} // namespace openlcb

#endif // _OPENLCB_CANFILTER_HXX_
