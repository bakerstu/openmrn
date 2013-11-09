// Basic tests.

#include "utils/if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "if/nmranet_if.h"

TEST_F(IfTest, Setup) {}

TEST_F(IfTest, WriteMessageSync) {
  // We write a message using the WriteFlow class directly into the interface.
  ExpectPacket(":X195B412DN0102030405060708;");
  event_write_helper1.WriteAsync(node_, MTI_EVENT_REPORT, WriteHelper::Global(),
                                 EventIdToBuffer(0x0102030405060708ULL),
                                 nullptr);
}

TEST_F(IfTest, WriteMessageASync) {
  // We write a message using the WriteFlow class asynchronously.
  ExpectPacket(":X195B412DN0102030405060708;");
  SyncNotifiable n;
  event_write_helper1.WriteAsync(node_, MTI_EVENT_REPORT, WriteHelper::Global(),
                                 EventIdToBuffer(0x0102030405060708ULL),
                                 &n);
  n.WaitForNotification();
}


TEST_F(IfTest, ReadMessageAndReply) {
  // We send an alias mapping enquiry frame and expect the node ID back.
  ExpectPacket(":X1070112DN02010d000003;");
  SendPacket(  ":X10702000N;");

}
