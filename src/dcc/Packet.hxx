/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Packet.hxx
 *
 * Defines a DCC Packet structure.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_PACKET_HXX_
#define _DCC_PACKET_HXX_

#include <stdint.h>
#include <string.h>

#include "dcc/Address.hxx"

namespace dcc
{

/** Represents a command to be sent to the track driver. Most of the commands
 * are "send packet X", but there are some that are controlling the track
 * booster itself, such as "power off". */
struct Packet
{
    /** Maximum number of payload bytes. */
    static const unsigned MAX_PAYLOAD = 5;
    /** Send this speed step to emergency-stop the locomotive. */
    static const unsigned EMERGENCY_STOP = 0xFFFF;
    /** Send this speed step to switch direction of the locomotive. Only used
     * for marklin-14-step speed commands. */
    static const unsigned CHANGE_DIR = 0xFFFF;

    Packet()
    {
        memset(this, 0, sizeof(*this));
    }

    struct pkt_t
    {
        // Always 0.
        uint8_t is_pkt : 1;
        // 0: DCC packet, 1: motorola packet.
        uint8_t is_marklin : 1;

        // typically for DCC packets:
        // 1: do NOT append an EC byte to the end of the packet.
        uint8_t skip_ec : 1;
        // 1: send long preamble instead of packet. 0: send normal preamble and
        // pkt.
        uint8_t send_long_preamble : 1;
        // 1: wait for service mode ack and report it back to the host.
        uint8_t sense_ack : 1;
        // Repeat count of the packet will be 1 + 5*rept_count. default: 0.
        uint8_t rept_count : 2;
        uint8_t reserved : 1;
    };

    struct cmd_t
    {
        // Always 1.
        uint8_t is_pkt : 1;
        // Command identifier.
        uint8_t cmd : 7;
    };

    union
    {
        uint8_t header_raw_data;
        pkt_t packet_header;
        cmd_t command_header;
    };
    /** Specifies the number of used payload bytes. */
    uint8_t dlc;
    /** Packet payload bytes. */
    uint8_t payload[MAX_PAYLOAD];

    /** Returns true if this is a packet, false if it is a command to the
     * track processor. */
    bool IsPacket()
    {
        return packet_header.is_pkt;
    }

    void set_cmd(uint8_t cmd)
    {
        dlc = 0;
        HASSERT(cmd & 1);
        header_raw_data = cmd;
    }

    /** Initializes the packet structure for a regular DCC packet. */
    void start_dcc_packet()
    {
        header_raw_data = 0;
        dlc = 0;
    }

    void add_dcc_address(DccShortAddress address);
    void add_dcc_address(DccLongAddress address);

    /** Adds a speed-and-direction command (dcc baseline command) ot the
     * packet. Speed is maximum 14. This should be called after
     * add_dcc_address. */
    void add_dcc_speed14(bool is_fwd, bool light, unsigned speed);
    template <class A>
    void set_dcc_speed14(A a, bool is_fwd, bool light, unsigned speed)
    {
        add_dcc_address(a);
        add_dcc_speed14(is_fwd, light, speed);
    }

    /** Adds a speed-and-direction command (dcc baseline command) to the
     * packet. Speed is maximum 28. This should be called after
     * add_dcc_address. */
    void add_dcc_speed28(bool is_fwd, unsigned speed);
    template <class A> void set_dcc_speed28(A a, bool is_fwd, unsigned speed)
    {
        add_dcc_address(a);
        add_dcc_speed28(is_fwd, speed);
    }

    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. */
    void add_dcc_function0_4(unsigned values);
    void add_dcc_function5_8(unsigned values);
    void add_dcc_function9_12(unsigned values);
    void add_dcc_function13_20(unsigned values);
    void add_dcc_function21_28(unsigned values);

    /** Appends one byte to the packet payload that represents the XOR checksum
     * for DCC. */
    void add_dcc_checksum();

    /** Creates a DCC idle packet. (Includes the checksum.) */
    void set_dcc_idle();

    /** Creates a DCC reset-all-decoders packet. (Includes the checksum.) */
    void set_dcc_reset_all_decoders();

    /** Sets the packet type to marklin-motorola. Initilizes the packet with 18
     * zero bits. */
    void start_mm_packet();

    /** Sets the address and F0 bits of an MM packet to a specific loco
     * address. */
    void add_mm_address(MMAddress address, bool light);

    /** Sets the packet to a 14-step MM speed-and-light packet. Max value of
     * speed is 14. A special value of speed == CHANGE_DIR will signal
     * direction change. */
    void add_mm_speed(unsigned speed);

    /** Sets the packet to a direction-aware 14-step MM speed-and-light
     * packet. Max value of speed is 14. */
    void add_mm_new_speed(bool is_fwd, unsigned speed);

    /** Creates a speed-and-fn packet for the new MM format.
     * @param fn_num is the function, valid values = 1..4
     * @param is whether the funciton is on or off
     * @param speed is the speed step (0..14). If it is set to emergency stop,
     * then no function packet will be generated.
     */
    void add_mm_new_fn(unsigned fn_num, bool value, unsigned speed);

private:
    /** Sets the speed bits of an MM packet. Clips speed to 15, avoids speed==1
     * and returns the final speed step number. */
    unsigned set_mm_speed_bits(unsigned speed);
};

} // namespace dcc

#endif // _DCC_PACKET_HXX_
