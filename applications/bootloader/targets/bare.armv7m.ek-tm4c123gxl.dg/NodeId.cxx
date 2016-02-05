#include "nmranet/If.hxx"

extern const nmranet::NodeID NODE_ID;
const nmranet::NodeID NODE_ID = 0x050101011F00ULL | NODEID_LOW_BITS;
extern const uint16_t DEFAULT_ALIAS;
const uint16_t DEFAULT_ALIAS = 0x400 | NODEID_LOW_BITS;

#define BOOTLOADER_DATAGRAM
#include "nmranet/Bootloader.hxx"
