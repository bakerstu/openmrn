/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file main.cxx
 *
 * Application that dumps all DCC packets to USB-serial.
 *
 * @author Balazs Racz
 * @date 4 Sep 2018
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "dcc/DccDebug.hxx"
#include "dcc/PacketProcessor.hxx"
#include "dcc/RailCom.hxx"
#include "executor/Executor.hxx"
#include "freertos/tc_ioctl.h"
#include "freertos_drivers/common/RailcomDriver.hxx"
#include "os/os.h"
#include "utils/StringPrintf.hxx"
#include "utils/constants.hxx"

#include "hardware.hxx"

Executor<1> executor("executor", 0, 2048);

// We reserve a lot of buffer for transmit to cover for small hiccups in the
// host reading data.
OVERRIDE_CONST(serial_tx_buffer_size, 2048);
OVERRIDE_CONST(main_thread_priority, 3);

// The DCC address to listen at. This is in wire format. The first address byte
// is the high byte.
uint16_t dcc_address_wire = 3 << 8;

uint8_t f0 = 0;

/// Stores a programming packet which is conditional on a repetition.
DCCPacket progStored;
/// Feedback for the stored programming packet.
dcc::Feedback fbStored;

// File descriptor for the eeprom storage.
int eefd;

/// Constants used for DCC packet decoding.
enum PacketCode {
    /// Which bits in the command are coding the CV number high bits.
    DCC_PROG_CVMASK = 3,
    /// POM read 1 byte command
    DCC_PROG_READ1 = 0b11100100,
    /// POM write 1 byte command
    DCC_PROG_WRITE1 = 0b11101100,
};

/// Reads CVs. Should only be called from the main thread.
void read_cvs(uint32_t ofs, unsigned len, uint8_t *dst)
{
    for (unsigned i = 0; i < len; i++)
    {
        dst[i] = 42;
    }
}

void write_cv(uint32_t ofs, uint8_t value) {
    
}

/// Checks if a packet is addressed to our current DCC address.
/// @param payload the DCC packet payload
/// @return true if addressed to us.
bool match_dcc_address(const uint8_t *payload)
{
    return ((payload[0] == (dcc_address_wire >> 8)) &&
        ((dcc_address_wire < (128 << 8)) ||
            (payload[1] == (dcc_address_wire & 0xff))));
}

/// @return true if the two DCC packets are the same on the wire.
bool packet_match(const DCCPacket &a, const DCCPacket &b)
{
    if (a.dlc != b.dlc)
    {
        return false;
    }
    for (unsigned i = 0; i < a.dlc; i++)
    {
        if (a.payload[i] != b.payload[i])
            return false;
    }
    return true;
}

class IrqProcessor : public dcc::PacketProcessor
{
public:
    IrqProcessor()
    {
    }

    /// Called in the main to prepare the railcom feedback packets.
    void init()
    {
        update_address();
        // Programming response packet
        ((dcc::Packet*)&progStored)->clear();
        // 6-byte Ack packet
        ack_.reset(0);
        for (unsigned i = 0; i < 6; i++)
        {
            ack_.add_ch2_data(dcc::RailcomDefs::CODE_ACK);
        }
    }

    /// Updates the broadcast datagrams based on the active DCC address.
    void update_address()
    {
        bcastHigh_.reset(0);
        bcastLow_.reset(0);
        if (dcc_address_wire < (128 << 8))
        {
            dcc::RailcomDefs::append12(
                dcc::RMOB_ADRHIGH, 0, bcastHigh_.ch1Data);
            dcc::RailcomDefs::append12(
                dcc::RMOB_ADRLOW, dcc_address_wire >> 8, bcastLow_.ch1Data);
        }
        else
        {
            uint8_t ah = 0x80 | ((dcc_address_wire >> 8) & 0x3F);
            uint8_t al = dcc_address_wire & 0xFF;
            dcc::RailcomDefs::append12(
                dcc::RMOB_ADRHIGH, ah, bcastHigh_.ch1Data);
            dcc::RailcomDefs::append12(dcc::RMOB_ADRLOW, al, bcastLow_.ch1Data);
        }
        bcastHigh_.ch1Size = 2;
        bcastLow_.ch1Size = 2;
    }

    /// Called from the interrupt routine to process a packet and generate the
    /// railcom response.
    /// @param pkt the last received packet
    /// @param railcom pointer to the railcom driver where the railcom feedback
    /// needs to be sent.
    void packet_arrived(const DCCPacket *pkt, RailcomDriver *railcom) override
    {
        DEBUG1_Pin::set(true);
        if (pkt->packet_header.csum_error)
        {
            return;
        }
        uint8_t adrhi = pkt->payload[0];
        if (adrhi && (adrhi < 232) && ((adrhi & 0xC0) != 0x80))
        {
            // Mobile decoder addressed. Send back address.
            if (bcastAtHi_)
            {
                bcastHigh_.feedbackKey = pkt->feedback_key;
                railcom->send_ch1(&bcastHigh_);
            }
            else
            {
                bcastLow_.feedbackKey = pkt->feedback_key;
                railcom->send_ch1(&bcastLow_);
            }
            bcastAtHi_ ^= 1;
        }
        // Checks for regular addressing.
        if (match_dcc_address(pkt->payload))
        {
            // Addressed packet to our DCC address.

            // Check for a known POM packet.
            if (packet_match(*pkt, progStored) &&
                (fbStored.feedbackKey == progStored.feedback_key))
            {
                prog_ = fbStored;
                prog_.feedbackKey = pkt->feedback_key;
                railcom->send_ch2(&prog_);
            }
            else
            {
                // No specific reply prepared -- send just some acks.
                ack_.feedbackKey = pkt->feedback_key;
                railcom->send_ch2(&ack_);
            }
        }
        DEBUG1_Pin::set(false);
    }

private:
    /// RailCom packet to send for address high in the broadcast channel.
    dcc::Feedback bcastHigh_;
    /// RailCom packet to send for address low in the broadcast channel.
    dcc::Feedback bcastLow_;

    /// Copy of the stored programming feedback.
    dcc::Feedback prog_;
    
    /// RailCom packet with all ACKs in channel2.
    dcc::Feedback ack_;

    /// 1 if the next broadcast packet should be adrhi, 0 if adrlo.
    uint8_t bcastAtHi_ : 1;
} irqProc;

/// Called from main with the last received packet.
/// @param p the DCC packet from the track.
void process_packet(const DCCPacket &p)
{
    if (p.packet_header.csum_error)
    {
        return;
    }
    if (p.dlc < 3)
    {
        return;
    }
    unsigned ofs = 1;
    if (match_dcc_address(p.payload))
    {
        if (dcc_address_wire >= (128 << 8))
        {
            // Two byte address.
            ofs++;
        }
        if ((p.payload[ofs] >> 5) == 0b100)
        {
            // F0-F4 packet
            ofs++;
            f0 = p.payload[ofs] & 0b00010000 ? 1 : 0;
        } else if ((p.payload[ofs] & ~DCC_PROG_CVMASK) == DCC_PROG_READ1) {
            if (packet_match(p, progStored))
            {
                // We already processed this packet, nothing to do.
                return;
            }
            fbStored.reset(0);
            progStored = p;
            uint32_t cv = ((p.payload[ofs] & DCC_PROG_CVMASK) << 8) |
                (p.payload[ofs + 1]);
            uint8_t value;
            read_cvs(cv, 1, &value);
            dcc::RailcomDefs::append12(dcc::RMOB_POM, value, fbStored.ch2Data);
            fbStored.ch2Size = 2;
            fbStored.feedbackKey = progStored.feedback_key;
        }
    }
}

extern "C"
{
    void set_dcc_interrupt_processor(dcc::PacketProcessor *p);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    setblink(0);
    int fd = ::open("/dev/dcc_decoder0", O_RDONLY);
    HASSERT(fd >= 0);
    // int wfd = ::open("/dev/serUSB0", O_RDWR);
    int wfd = ::open("/dev/ser0", O_RDWR);
    HASSERT(wfd >= 0);
    int rcfd = ::open("/dev/ser1", O_WRONLY);
    HASSERT(rcfd >= 0);
    auto ret = ::ioctl(rcfd, TCBAUDRATE, 250000);
    HASSERT(ret == 0);
    eefd = ::open("/dev/eeprom", O_RDWR);
    HASSERT(eefd >= 0);

    static const char HELLO[] = "DCC Decoder program.\n";
    ::write(wfd, HELLO, sizeof(HELLO));
    
    irqProc.init();
    set_dcc_interrupt_processor(&irqProc);

    int cnt = 0;
    while (1)
    {
        DCCPacket packet_data;
        int sret = ::read(fd, &packet_data, sizeof(packet_data));
        HASSERT(sret == sizeof(packet_data));
        DEBUG1_Pin::set(true);
        process_packet(packet_data);
        long long t = os_get_time_monotonic();
        string txt = StringPrintf("\n%02d.%06d %04d ",
            (unsigned)((t / 1000000000) % 100),
            (unsigned)((t / 1000) % 1000000), cnt % 10000);
        ::write(wfd, txt.data(), txt.size());
        txt = dcc::packet_to_string(packet_data);
        ::write(wfd, txt.data(), txt.size());
        // we purposefully do not check the return value, because if there was
        // not enough space in the serial write buffer, we need to throw away
        // data.
        ++cnt;
        uint8_t blink = ((cnt >> 3) & 15) == 1u ? 1 : 0;
        resetblink(f0 ^ blink);
        DEBUG1_Pin::set(false);
    }
    return 0;
}
