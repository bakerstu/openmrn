#include "openlcb/If.hxx"
#include "address.h"

#ifndef NODEID_HIGH_BITS
#define NODEID_HIGH_BITS 0x18
#endif

extern const openlcb::NodeID NODE_ID;
const openlcb::NodeID NODE_ID = 0x030014325700ULL | NODEID_LOW_BITS;
