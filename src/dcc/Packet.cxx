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
 * \file Packet.cxx
 *
 * Defines a DCC Packet structure.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

//#define LOGLEVEL WARNING

#include "dcc/Packet.hxx"

#include "dcc/Defs.hxx"
#include "utils/Crc.hxx"
#include "utils/logging.h"
#include "utils/macros.h"


namespace dcc
{

// Imports the bit declarations from the enums in Defs. This import may only be
// performed in a .cxx file.
using namespace Defs;

static_assert(sizeof(Packet) == sizeof(DCCPacket), "DCCPacket size missmatch");

void Packet::add_dcc_checksum()
{
    HASSERT(dlc < MAX_PAYLOAD);
    // Protects against double call of add checksum.
    HASSERT(!packet_header.skip_ec);

    // Performs CRC computation if needed.
    if ((payload[0] == ADDRESS_LOGON || payload[0] == ADDRESS_EXT) &&
        (dlc >= 6))
    {
        Crc8DallasMaxim m;
        for (int i = 0; i < dlc; ++i)
        {
            m.update16(payload[i]);
        }
        payload[dlc++] = m.get();
    }

    uint8_t cs = 0;
    for (int i = 0; i < dlc; ++i)
    {
        cs ^= payload[i];
    }
    payload[dlc++] = cs;
    packet_header.skip_ec = 1;
}

void Packet::set_dcc_idle()
{
    start_dcc_packet();
    dlc = 3;
    payload[0] = payload[2] = 0xFF;
    payload[1] = 0;
    packet_header.skip_ec = 1;
}

void Packet::set_dcc_reset_all_decoders()
{
    start_dcc_packet();
    dlc = 3;
    payload[0] = payload[1] = payload[2] = 0;
    packet_header.skip_ec = 1;
}

void Packet::add_dcc_address(DccShortAddress address)
{
    start_dcc_packet();
    payload[dlc++] = address.value & 0x7F;
    feedback_key = address.value;
}

void Packet::add_dcc_address(DccLongAddress address)
{
    start_dcc_packet();
    payload[dlc++] = DCC_LONG_ADDRESS_FIRST | (address.value >> 8);
    payload[dlc++] = address.value & 0xff;
    feedback_key = address.value;
}

void Packet::add_dcc_accy_address(bool is_basic, unsigned address)
{
    start_dcc_packet();

    payload[dlc++] = DCC_BASIC_ACCESSORY_B1 | ((address >> 2) & 0b111111);
    uint8_t b2 = is_basic ? DCC_BASIC_ACCESSORY_B2 : DCC_EXT_ACCESSORY_B2;
    b2 |= ((~(address >> 8)) & 0b111) << 4;
    b2 |= (address & 0b11) << 1;
    payload[dlc++] = b2;
    feedback_key = 0x10000 | address;
}

void Packet::add_dcc_speed14(bool is_fwd, bool light, unsigned speed)
{
    /// TODO(balazs.racz) add unittests.
    uint8_t b1 = DCC_BASELINE_SPEED;
    if (is_fwd)
        b1 |= DCC_BASELINE_SPEED_FORWARD;
    if (light)
        b1 |= DCC_BASELINE_SPEED_LIGHT;
    if (speed == EMERGENCY_STOP)
    {
        b1 |= 1;
    }
    else if (speed == 0)
    {
    }
    else
    {
        HASSERT(speed <= 14);
        speed += 1; // avoids 01 (e-stop)
        b1 |= speed & 0xf;
    }
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_speed28(bool is_fwd, unsigned speed)
{
    uint8_t b1 = DCC_BASELINE_SPEED;
    if (is_fwd)
        b1 |= DCC_BASELINE_SPEED_FORWARD;
    if (speed == EMERGENCY_STOP)
    {
        b1 |= 1;
    }
    else if (speed == 0)
    {
    }
    else
    {
        HASSERT(speed <= 28);
        speed += 3; // avoids 00, 01 (stop) and 10 11 (e-stop)
        if (speed & 1)
            b1 |= DCC_BASELINE_SPEED_LIGHT;
        b1 |= (speed >> 1) & 0xf;
    }
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_speed128(bool is_fwd, unsigned speed)
{
    payload[dlc++] = DCC_EXT_SPEED;
    uint8_t b = 0;
    if (is_fwd)
        b |= DCC_EXT_SPEED_FORWARD;
    if (speed == EMERGENCY_STOP)
    {
        b |= 1;
    }
    else if (speed > 0)
    {
        HASSERT(speed <= 126);
        speed += 1; // avoids 00, 01 ([e]stop)
        b |= speed & 0x7f;
    }
    payload[dlc++] = b;
    add_dcc_checksum();
}

void Packet::add_dcc_function0_4(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION1;
    if (values & 1)
        b1 |= DCC_FUNCTION1_F0;
    b1 |= (values >> 1) & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function5_8(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION2_F5;
    b1 |= values & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function9_12(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION2_F9;
    b1 |= values & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function_hi(uint8_t base, uint8_t values)
{
    base -= 13;
    HASSERT((base & 0b111) == 0);
    HASSERT(base <= (61 - 13));
    base >>= 3;
    base -= 2;
    payload[dlc++] = DCC_FEATURE_EXP_FNHI | (base & 0b111);
    payload[dlc++] = values;
    add_dcc_checksum();
}

void Packet::add_dcc_binary_state(uint16_t fn, bool value)
{
    if (fn <= 127)
    {
        payload[dlc++] = DCC_BINARY_SHORT;
        payload[dlc++] = fn | (value ? 0x80 : 0);
    }
    else
    {
        payload[dlc++] = DCC_BINARY_LONG;
        payload[dlc++] = (fn & 0x7F) | (value ? 0x80 : 0);
        payload[dlc++] = (fn >> 8) & 0xFF;
    }
    add_dcc_checksum();
}

void Packet::add_dcc_analog_function(uint8_t fn, uint8_t value)
{
    payload[dlc++] = DCC_ANALOG_FN;
    payload[dlc++] = fn;
    payload[dlc++] = value;
    add_dcc_checksum();
}

void Packet::add_dcc_prog_command(
    uint8_t cmd_hi, unsigned cv_number, uint8_t value)
{
    payload[dlc++] = (cmd_hi & (~3)) | ((cv_number >> 8) & 3);
    payload[dlc++] = cv_number & 0xff;
    payload[dlc++] = value;
    add_dcc_checksum();
}

void Packet::add_dcc_pom_read1(unsigned cv_number)
{
    add_dcc_prog_command(DCC_PROG_READ1, cv_number, 0);
}

void Packet::add_dcc_pom_write1(unsigned cv_number, uint8_t value) {
    add_dcc_prog_command(DCC_PROG_WRITE1, cv_number, value);
}

void Packet::set_dcc_svc_verify_byte(unsigned cv_number, uint8_t value)
{
    start_dcc_svc_packet();
    add_dcc_prog_command(DCC_SVC_VERIFY, cv_number, value);
}

void Packet::set_dcc_svc_write_byte(unsigned cv_number, uint8_t value)
{
    start_dcc_svc_packet();
    add_dcc_prog_command(DCC_SVC_WRITE, cv_number, value);
}

void Packet::set_dcc_svc_verify_bit(
    unsigned cv_number, unsigned bit, bool expected)
{
    start_dcc_svc_packet();
    uint8_t vvv = DCC_SVC_BITVAL_VERIFY |
        (expected ? DCC_SVC_BITVAL_VALUE : 0) | (bit & 7);
    add_dcc_prog_command(DCC_SVC_BIT_MANIPULATE, cv_number, vvv);
}

void Packet::set_dcc_svc_write_bit(
    unsigned cv_number, unsigned bit, bool desired)
{
    start_dcc_svc_packet();
    uint8_t vvv =
        DCC_SVC_BITVAL_WRITE | (desired ? DCC_SVC_BITVAL_VALUE : 0) | (bit & 7);
    add_dcc_prog_command(DCC_SVC_BIT_MANIPULATE, cv_number, vvv);
}

void Packet::set_dcc_svc_paged_write_reg(uint8_t reg, uint8_t value)
{
    HASSERT(reg < 8);
    start_dcc_svc_packet();
    payload[dlc++] = DCC_SVC_PAGED_WRITE | reg;
    payload[dlc++] = value;
    add_dcc_checksum();
}

void Packet::set_dcc_svc_paged_verify_reg(uint8_t reg, uint8_t value)
{
    HASSERT(reg < 8);
    start_dcc_svc_packet();
    payload[dlc++] = DCC_SVC_PAGED_VERIFY | reg;
    payload[dlc++] = value;
    add_dcc_checksum();
}

void Packet::set_dcc_basic_accy_params(bool is_normal, bool is_activate)
{
    if (is_normal)
    {
        payload[1] |= DCC_BASIC_ACCESSORY_B2_CLOSED;
    }
    else
    {
        payload[1] |= DCC_BASIC_ACCESSORY_B2_THROWN;
    }
    if (is_activate)
    {
        payload[1] |= DCC_BASIC_ACCESSORY_B2_ACTIVATE;
    }
    else
    {
        payload[1] |= DCC_BASIC_ACCESSORY_B2_DEACTIVATE;
    }
}

void Packet::add_dcc_basic_accessory(unsigned address, bool is_activate)
{
    add_dcc_accy_address(true, address >> 1);
    set_dcc_basic_accy_params(address & 1, is_activate);
    add_dcc_checksum();
}

void Packet::add_dcc_ext_accessory(unsigned address, uint8_t aspect)
{
    add_dcc_accy_address(false, address);
    payload[dlc++] = aspect;
    add_dcc_checksum();
}


void Packet::set_dcc_logon_enable(
    Defs::LogonEnableParam param, uint16_t cid, uint8_t session_id)
{
    start_dcc_packet();
    payload[dlc++] = ADDRESS_LOGON;
    payload[dlc++] = DCC_LOGON_ENABLE | ((uint8_t)param & 0x3);
    payload[dlc++] = cid >> 8;
    payload[dlc++] = cid & 0xff;
    payload[dlc++] = session_id;
    add_dcc_checksum();
}

void Packet::set_dcc_select_shortinfo(uint64_t decoder_id)
{
    start_dcc_packet();
    payload[dlc++] = ADDRESS_LOGON;
    
    payload[dlc++] = DCC_SELECT | ((decoder_id >> 40) & 0xf);
    payload[dlc++] = (decoder_id >> 32) & 0xff;
    payload[dlc++] = (decoder_id >> 24) & 0xff;
    payload[dlc++] = (decoder_id >> 16) & 0xff;
    payload[dlc++] = (decoder_id >> 8) & 0xff;
    payload[dlc++] = (decoder_id) & 0xff;
    payload[dlc++] = CMD_READ_SHORT_INFO;
    add_dcc_checksum();
}

void Packet::set_dcc_logon_assign(uint64_t decoder_id, uint16_t address)
{
    start_dcc_packet();
    payload[dlc++] = ADDRESS_LOGON;
    
    payload[dlc++] = DCC_LOGON_ASSIGN | ((decoder_id >> 40) & 0xf);
    payload[dlc++] = (decoder_id >> 32) & 0xff;
    payload[dlc++] = (decoder_id >> 24) & 0xff;
    payload[dlc++] = (decoder_id >> 16) & 0xff;
    payload[dlc++] = (decoder_id >> 8) & 0xff;
    payload[dlc++] = (decoder_id) & 0xff;
    payload[dlc++] = ((address >> 8) & 0xff) ^ 0b11000000;
    payload[dlc++] = (address) & 0xff;
    add_dcc_checksum();
}

void Packet::start_mm_packet()
{
    dlc = 3;
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 0;
    header_raw_data = MARKLIN_DEFAULT_CMD;
}

/// Helper array to translate a marklin address to a bit sequence.
static const uint8_t marklin_address[3] = {0b00, 0b11, 0b10};
/// Helper array to translate a marklin function set command to a packet.
static const uint8_t marklin_fn_bits[5] = {0,          0b01010000, 0b00000100,
                                           0b00010100, 0b01010100};

/** Sets the address bits of an MM packet to a specific loco address. @param a
 * is the train address @param light if true, light (f0) will be set to ON. */
void Packet::add_mm_address(MMAddress a, bool light)
{
    uint8_t address = a.value;
    payload[0] |= marklin_address[address % 3];
    address /= 3;
    payload[1] |= marklin_address[address % 3] << 6;
    address /= 3;
    payload[1] |= marklin_address[address % 3] << 4;
    address /= 3;
    payload[1] |= marklin_address[address % 3] << 2;
    if (light)
    {
        payload[1] |= 0b11;
    }
}

unsigned Packet::set_mm_speed_bits(unsigned speed)
{
    payload[2] = 0;
    // avoids speed step 1.
    if (speed > 0)
        ++speed;
    // clips speed
    if (speed > 15)
        speed = 15;
    if (speed & 1)
        payload[2] |= 0x80;
    if (speed & 2)
        payload[2] |= 0x20;
    if (speed & 4)
        payload[2] |= 0x08;
    if (speed & 8)
        payload[2] |= 0x02;
    return speed;
}

/** Sets the packet to a 14-step MM speed-and-light packet. Max value of
 * speed is 14. A special value of speed == CHANGE_DIR will signal
 * direction change. */
void Packet::add_mm_speed(unsigned speed)
{
    if (speed == CHANGE_DIR || speed == EMERGENCY_STOP)
    {
        payload[2] = MARKLIN_CHANGE_DIR_B2;
        // Up the repeat count so that it will be surely heard.
        header_raw_data |= 0b01100000;
    }
    else
    {
        set_mm_speed_bits(speed);
        // The old marklin protocol needs all bits doubled.
        payload[2] |= (payload[2] >> 1);
    }
}

/** Sets the packet to a direction-aware 14-step MM speed-and-light
 * packet. Max value of speed is 14. */
void Packet::add_mm_new_speed(bool is_fwd, unsigned speed)
{
    if (speed == CHANGE_DIR || speed == EMERGENCY_STOP)
    {
        payload[2] = MARKLIN_CHANGE_DIR_B2;
        // Up the repeat count so that it will be surely heard.
        header_raw_data |= 0b01100000;
    }
    else
    {
        speed = set_mm_speed_bits(speed);
        if (is_fwd)
        {
            payload[2] |= 0x10;
        }
        else
        {
            payload[2] |= 0x44;
        }
        if (speed <= 7)
        {
            payload[2] |= 1;
        }
    }
}

void Packet::add_mm_new_fn(unsigned fn_num, bool value, unsigned speed)
{
    if (speed == CHANGE_DIR || speed == EMERGENCY_STOP || fn_num < 1 ||
        fn_num > 4)
    {
        return add_mm_new_speed(false, speed);
    }
    // Sets the base speed bits.
    speed = set_mm_speed_bits(speed);
    // Sets the function number bits.
    payload[2] |= marklin_fn_bits[fn_num];
    // And the function bit.
    if (value)
    {
        payload[2] |= 1;
    }
    // Checks for exceptions, when we generated a packet valid for the old
    // protocol. (aka all bits are doubled)
    if (((payload[2] & 0xaa) >> 1) == (payload[2] & 0x55))
    {
        payload[2] &= 0xaa;
        if (speed >= 8)
        {
            // High speeds. new bits are 0101.
            payload[2] |= 0b00010001;
        }
        else
        {
            // Low speeds. New bits are 1010.
            payload[2] |= 0b01000100;
        }
    }
}

void Packet::mm_shift() {
    HASSERT(dlc == 3);
    dlc = 6;
    payload[3] = payload[0];
    payload[4] = payload[1];
    payload[5] = payload[2];
}

} // namespace dcc
