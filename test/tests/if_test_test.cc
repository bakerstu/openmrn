// Basic tests.

#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/NMRAnetIf.hxx"

namespace NMRAnet {

TEST_F(AsyncNodeTest, Setup) {}

TEST_F(AsyncNodeTest, WriteMessageSync) {
  // We write a message using the WriteFlow class directly into the interface.
  ExpectPacket(":X195B422AN0102030405060708;");
  event_write_helper1.WriteAsync(node_, If::MTI_EVENT_REPORT, WriteHelper::global(),
                                 EventIdToBuffer(0x0102030405060708ULL),
                                 EmptyNotifiable::DefaultInstance());
}

TEST_F(AsyncNodeTest, WriteMessageASync) {
  // We write a message using the WriteFlow class asynchronously.
  ExpectPacket(":X195B422AN0102030405060708;");
  SyncNotifiable n;
  event_write_helper1.WriteAsync(node_, If::MTI_EVENT_REPORT, WriteHelper::global(),
                                 EventIdToBuffer(0x0102030405060708ULL),
                                 &n);
  n.WaitForNotification();
}

/** This is disabled because AME frmaes are not yet supported. */
TEST_F(AsyncNodeTest, DISABLED_ReadMessageAndReply) {
  // We send an alias mapping enquiry frame and expect the node ID back.
  ExpectPacket(":X1070122AN02010d000003;");
  SendPacket(  ":X10702001N;");
}

}  // namespace NMRAnet
