// Basic tests.

#include "utils/if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "if/nmranet_if.h"

TEST_F(IfTest, Setup) {}

TEST_F(IfTest, WriteMessage) {
  ExpectPacket(":X195B412DN0102030405060708;");
  event_write_helper1.WriteAsync(node_, MTI_EVENT_REPORT, WriteHelper::Global(),
                                 EventIdToBuffer(0x0102030405060708ULL),
                                 nullptr);
}
