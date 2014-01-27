#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetAsyncDatagram.hxx"
#include "nmranet/NMRAnetAsyncDatagramCan.hxx"

namespace NMRAnet
{

class AsyncDatagramTest : public AsyncNodeTest
{
protected:
    AsyncDatagramTest() : parser_(if_can_.get())
    {
    }

    CanDatagramParser parser_;
    StrictMock<MockMessageHandler> handler_;

};

TEST_F(AsyncDatagramTest, CreateDestroy) {}

TEST_F(AsyncDatagramTest, SingleFrameDatagramArrivesWrongTarget) {
    SendPacket(":X1A333555NFF01020304050607;");
}

TEST_F(AsyncDatagramTest, MultiFrameDatagramArrivesWrongTarget) {
    SendPacket(":X1B333555NFF01020304050607;");
    SendPacket(":X1C333555NFF01020304050607;");
    SendPacket(":X1C333555NFF01020304050607;");
    SendPacket(":X1D333555NFF01020304050607;");
}

TEST_F(AsyncDatagramTest, SingleFrameDatagramArrivesRightTarget) {
    SendPacket(":X1A22A555NFF01020304050607;");
}

TEST_F(AsyncDatagramTest, MultiFrameDatagramArrivesRightTarget) {
    SendPacket(":X1B22A555NFF01020304050607;");
    SendPacket(":X1C22A555NFF01020304050607;");
    SendPacket(":X1C22A555NFF01020304050607;");
    SendPacket(":X1D22A555NFF01020304050607;");
}

} // namespace NMRAnet
