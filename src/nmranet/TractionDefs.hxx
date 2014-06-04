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
 * \file TractionDefs.hxx
 *
 * Static definitions for implementations of the NMRAnet Traction and Traction
 * Proxy protocols.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

#ifndef _NMRANET_TRACTIONDEFS_HXX_
#define _NMRANET_TRACTIONDEFS_HXX_

#include <cmath>
#include <stdint.h>

#include "nmranet/Velocity.hxx"
#include "nmranet/Payload.hxx"

namespace nmranet {

typedef Velocity SpeedType;

/** Parses a SpeedType value from an unaligned memory address, typically from
 * the input buffer. */
SpeedType fp16_to_speed(const void *fp16);

/** Renders a SpeedType value to an unaligned memory address, typically to the
 * output buffer.
 *
 * @param speed is the speed to write
 * @param fp16 is an unaligned two-byte location to write the float16 value
 * to.*/
void speed_to_fp16(SpeedType speed, void *fp16);

/** @returns NAN as speed. */
inline SpeedType nan_to_speed() {
    SpeedType s;
    s.set_wire(0xFFFFU);
    return s;
}

struct TractionDefs {
    static const uint64_t IS_TRAIN_EVENT = 0x0101000000000303ULL;
    static const uint64_t EMERGENCY_STOP_EVENT = 0x010100000000FFFFULL;

    static const uint64_t NODE_ID_DC_BLOCK = 0x060000000000ULL;
    static const uint64_t NODE_ID_DCC = 0x060100000000ULL;
    static const uint64_t NODE_ID_TMCC = 0x060200000000ULL;
    static const uint64_t NODE_ID_MARKLIN_MOTOROLA = 0x060300000000ULL;
    static const uint64_t NODE_ID_MTH_DCS = 0x060400000000ULL;

    enum {
        // Byte 0 of request commands.
        REQ_SET_SPEED = 0x00,
        REQ_SET_FN = 0x01,
        REQ_EMERGENCY_STOP = 0x02,

        REQ_QUERY_SPEED = 0x10,
        REQ_QUERY_FN = 0x11,

        REQ_CONTROLLER_CONFIG = 0x12,
        REQ_CONSIST_CONFIG = 0x13,
        REQ_TRACTION_MGMT = 0x14,

        // Byte 1 of REQ_CONTROLLER_CONFIG command
        CTRLREQ_ASSIGN_CONTROLLER = 0x01,
        CTRLREQ_RELEASE_CONTROLLER = 0x02,
        CTRLREQ_QUERY_CONTROLLER = 0x03,
        CTRLREQ_NOTIFY_CONTROLLER_CHANGED = 0x04,

        // Byte 1 of REQ_CONSIST_CONFIG command
        CNSTREQ_ATTACH_NODE = 0x01,
        CNSTREQ_DETACH_NODE = 0x02,
        CNSTREQ_QUERY_NODES = 0x03,

        // Byte 1 of REQ_TRACTION_MGMT command
        MGMTREQ_RESERVE = 0x01,
        MGMTREQ_RELEASE = 0x02,

        // Byte 0 of response commands
        RESP_QUERY_SPEED = REQ_QUERY_SPEED,
        RESP_QUERY_FN = REQ_QUERY_FN,
        RESP_CONTROLLER_CONFIG = REQ_CONTROLLER_CONFIG,
        RESP_CONSISST_CONFIG = REQ_CONSIST_CONFIG,
        RESP_TRACTION_MGMT = REQ_TRACTION_MGMT,

        // Byte 1 of Controller Configuration response
        CTRLRESP_ASSIGN_CONTROLLER = CTRLREQ_ASSIGN_CONTROLLER,
        CTRLRESP_QUERY_CONTROLLER = CTRLREQ_QUERY_CONTROLLER,
        CTRLRESP_NOTIFY_CONTROLLER_CHANGED = CTRLREQ_NOTIFY_CONTROLLER_CHANGED,

        // Byte 1 of Consist Configuration response
        CNSTRESP_ATTACH_NODE = CNSTREQ_ATTACH_NODE,
        CNSTRESP_DETACH_NODE = CNSTREQ_DETACH_NODE,
        CNSTRESP_QUERY_NODES = CNSTREQ_QUERY_NODES,

        // Byte 1 of Traction Management replies
        MGMTRESP_RESERVE = MGMTREQ_RESERVE,


        /** This is the memory space number for accessing an NMRA DCC
         * locomotive's functions via the memory config protocol. */
        FUNCTION_CONFIG_MEMORY_SPACE = 0xF9,
        // These declare the configuration space layout. They are always the
        // third byte of the address in the configuration memory space, aka
        // address = type << 16 + offset.
        /** F0-F28 are at offset 0 to 28 here. */
        FNCONFIG_FN = 0x0,
        /** Binary State Control Instruction long form. Offset 0 to 32767 */
        FNCONFIG_BINARYSTATE_LONG = 0x1,
        /** Binary State Control Instruction short form. Offset 0 to 127 */
        FNCONFIG_BINARYSTATE_SHORT = 0x2,
        /** Analog outputs, defined by NMRA DCC WG Topic 9910241. Offset 0-255,
         * values 0-255 each. */
        FNCONFIG_ANALOG_OUTPUT = 0x3
    };

    static Payload speed_set_payload(Velocity v) {
        Payload p(3, 0);
        p[0] = REQ_SET_SPEED;
        speed_to_fp16(v, &p[1]);
        return p;
    }

    static Payload speed_get_payload() {
        Payload p(1, 0);
        p[0] = REQ_QUERY_SPEED;
        return p;
    }

    /** Parses the response payload of a GET_SPEED packet.
     * @returns true if the last_set_speed value was present and non-NaN.
     * @param p is the response payload.
     * @param v is the velocity that will be set to the speed value. */
    static bool speed_get_parse_last(const Payload &p, Velocity *v)
    {
        if (p.size() < 3)
        {
            return false;
        }
        *v = fp16_to_speed(p.data() + 1);
        if (isnan(v->speed()))
        {
            return false;
        }
        return true;
    }

    static Payload fn_set_payload(unsigned address, uint16_t value) {
        Payload p(6, 0);
        p[0] = REQ_SET_FN;
        p[1] = (address >> 16) & 0xff;
        p[2] = (address >> 8) & 0xff;
        p[3] = address & 0xff;
        p[4] = (value >> 8) & 0xff;
        p[5] = value & 0xff;
        return p;
    }

    static Payload fn_get_payload(unsigned address) {
        Payload p(4, 0);
        p[0] = REQ_QUERY_FN;
        p[1] = (address >> 16) & 0xff;
        p[2] = (address >> 8) & 0xff;
        p[3] = address & 0xff;
        return p;
    }

    /** Parses the response payload of a GET_FN packet.
     * @returns true if there is a valid function value.
     * @param p is the response payload.
     * @param value will be set to the output value. */
    static bool fn_get_parse(const Payload &p, uint16_t *value)
    {
        if (p.size() < 6)
        {
            return false;
        }
        *value = (((uint16_t)p[4]) << 8) | p[5];
        return true;
    }


};

}  // namespace nmranet

#endif //_NMRANET_TRACTIONDEFS_HXX_
