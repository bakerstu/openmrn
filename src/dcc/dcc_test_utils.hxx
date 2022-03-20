#ifndef _DCC_DCC_TEST_UTILS_HXX_
#define _DCC_DCC_TEST_UTILS_HXX_

#include "dcc/DccDebug.hxx"
#include "dcc/Packet.hxx"

void PrintTo(const DCCPacket &pkt, ::std::ostream *os)
{
    *os << dcc::packet_to_string(pkt);
}

namespace dcc
{
void PrintTo(const Packet &pkt, ::std::ostream *os)
{
    *os << packet_to_string(pkt);
}
}

string PrintToString(const dcc::Packet &pkt)
{
    return dcc::packet_to_string(pkt);
}

std::vector<uint8_t> dcc_from(uint8_t d0, uint8_t d1, int d2 = -1, int d3 = -1, int d4 = -1)
{
    std::vector<uint8_t> ret;
    ret.push_back(d0);
    ret.push_back(d1);
    if (d2 >= 0)
    {
        ret.push_back(d2);
    }
    else if (d2 == -2)
    {
        ret.push_back(d0 ^ d1);
    }
    if (d3 >= 0)
    {
        ret.push_back(d3);
    }
    else if (d3 == -2)
    {
        ret.push_back(d0 ^ d1 ^ ret[2]);
    }
    if (d4 >= 0)
    {
        ret.push_back(d4);
    }
    else if (d4 == -2)
    {
        ret.push_back(d0 ^ d1 ^ ret[2] ^ ret[3]);
    }
    return ret;
}

dcc::Packet packet_from(uint8_t hdr, std::vector<uint8_t> payload)
{
    dcc::Packet pkt;
    pkt.header_raw_data = hdr;
    pkt.dlc = payload.size();
    memcpy(pkt.payload, &payload[0], pkt.dlc);
    return pkt;
}

MATCHER_P2(PacketIs, hdr, payload, PrintToString(packet_from(hdr, payload)))
{
    dcc::Packet pkt = packet_from(hdr, payload);
    dcc::Packet argc = arg;
    vector<uint8_t> exp_bytes(pkt.payload, pkt.payload + pkt.dlc);
    vector<uint8_t> act_bytes(arg.payload, arg.payload + arg.dlc);
    if (pkt.packet_header.is_pkt == 0 && pkt.packet_header.is_marklin == 0 &&
        arg.packet_header.is_pkt == 0 && arg.packet_header.is_marklin == 0 &&
        pkt.packet_header.skip_ec != arg.packet_header.skip_ec)
    {
        // Mismatch in whether the EC byte is there. We fix this by adding the
        // EC byte to the place where it is missing. This avoids test failures
        // where the packets really are equivalent.
        if (pkt.packet_header.skip_ec == 0)
        {
            uint8_t ec = 0;
            for (uint8_t b : exp_bytes)
            {
                ec ^= b;
            }
            exp_bytes.push_back(ec);
            pkt.packet_header.skip_ec = 1;
        }
        if (arg.packet_header.skip_ec == 0)
        {
            uint8_t ec = 0;
            for (uint8_t b : act_bytes)
            {
                ec ^= b;
            }
            act_bytes.push_back(ec);
            argc.packet_header.skip_ec = 1;
        }
        return (pkt.header_raw_data == argc.header_raw_data &&
            exp_bytes == act_bytes);
    }
    return (pkt.header_raw_data == arg.header_raw_data && pkt.dlc == arg.dlc &&
        memcmp(pkt.payload, arg.payload, pkt.dlc) == 0);
}

#endif // _DCC_DCC_TEST_UTILS_HXX_
