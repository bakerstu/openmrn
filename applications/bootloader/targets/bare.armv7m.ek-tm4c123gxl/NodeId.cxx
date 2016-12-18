#include "openlcb/If.hxx"
#include "address.h"

extern const openlcb::NodeID NODE_ID;
const openlcb::NodeID NODE_ID = 0x050101011800ULL | NODEID_LOW_BITS;
extern const uint16_t DEFAULT_ALIAS;
const uint16_t DEFAULT_ALIAS = 0x400 | NODEID_LOW_BITS;

#define BOOTLOADER_STREAM
//#define BOOTLOADER_DATAGRAM
#include "openlcb/Bootloader.hxx"
