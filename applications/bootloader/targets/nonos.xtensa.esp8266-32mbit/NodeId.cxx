#include "nmranet/If.hxx"

#define NODEID_LOW_BITS 0xDD

extern const nmranet::NodeID NODE_ID;
const nmranet::NodeID NODE_ID = 0x050101011400ULL | NODEID_LOW_BITS;
extern const uint16_t DEFAULT_ALIAS;
const uint16_t DEFAULT_ALIAS = 0x400 | NODEID_LOW_BITS;

#define BOOTLOADER_STREAM
//#define BOOTLOADER_DATAGRAM
#include "nmranet/Bootloader.hxx"
