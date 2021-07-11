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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "os/os.h"
#include "executor/Executor.hxx"
#include "dcc/DccDebug.hxx"
#include "utils/constants.hxx"
#include "utils/StringPrintf.hxx"
#include "dcc/PacketProcessor.hxx"
#include "dcc/RailCom.hxx"
#include "freertos_drivers/common/RailcomDriver.hxx"
#include "freertos/tc_ioctl.h"

#include "hardware.hxx"

Executor<1> executor("executor", 0, 2048);

// We reserve a lot of buffer for transmit to cover for small hiccups in the
// host reading data.
OVERRIDE_CONST(serial_tx_buffer_size, 2048);
OVERRIDE_CONST(main_thread_priority, 3);

// The DCC address to listen at. This is in wire format. The first address byte
// is the high byte.
static const uint16_t dcc_address_wire = 3 << 8;

uint8_t f0 = 0;

void process_packet(const DCCPacket& p) {
    if (p.packet_header.csum_error) {
        return;
    }
    if (p.dlc < 3) {
        return;
    }
    if (p.payload[0] != (dcc_address_wire >> 8)) {
        return;
    }
    unsigned ofs = 1;
    if ((dcc_address_wire >> 8) > 127) {
        // Two byte address.
        ofs++;
        if (p.payload[1] != (dcc_address_wire & 0xff)) {
            return;
        }
    }
    if ((p.payload[ofs] >> 5) == 0b100)
    {
        // F0-F4 packet
        ofs++;
        f0 = p.payload[ofs] & 0b00010000 ? 1 : 0;
    }
}

class IrqProcessor : public dcc::PacketProcessor {
public:
    IrqProcessor() {
    }

    /// Called in the main to prepare the railcom feedback packets.
    void init()
    {
        ch1_.reset(0);
        ch1_.add_ch1_data(0x00);
        ch1_.add_ch1_data(0x00);

        ch2_.reset(0);
        ch2_.add_ch2_data(0);
        ch2_.add_ch2_data(0);
        ch2_.add_ch2_data(0);
        ch2_.add_ch2_data(0);
        ch2_.add_ch2_data(0);
        ch2_.add_ch2_data(0);
#if 0
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
        ch2_.add_ch2_data(dcc::RailcomDefs::ACK);
#endif
    }

    void packet_arrived(
        const DCCPacket *pkt, RailcomDriver *railcom) override {
        DEBUG1_Pin::set(true);
        if (pkt->packet_header.csum_error) {
            return;
        }
        ch1_.feedbackKey = pkt->feedback_key;
        ch2_.feedbackKey = pkt->feedback_key;
        railcom->send_ch1(&ch1_);
        railcom->send_ch2(&ch2_);
        DEBUG1_Pin::set(false);
    }

private:
    dcc::Feedback ch1_;
    dcc::Feedback ch2_;
} irqProc;

extern "C" {
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
    //int wfd = ::open("/dev/serUSB0", O_RDWR);
    int wfd = ::open("/dev/ser0", O_RDWR);
    HASSERT(wfd >= 0);
    int rcfd = ::open("/dev/ser1", O_WRONLY);
    HASSERT(rcfd >= 0);
    auto ret = ::ioctl(rcfd, TCBAUDRATE, 250000);
    HASSERT(ret == 0);

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
