#include "utils/async_if_test_helper.hxx"

namespace NMRAnet
{

TEST_F(AsyncNodeTest, VerifyNodeIdGlobalSingleNode)
{
    SendPacketAndExpectResponse(":X1949033AN;", ":X1917022AN02010d000003;");
}

class TwoNodeTest : public AsyncNodeTest
{
public:
    TwoNodeTest()
    {
        CreateAllocatedAlias();
        ExpectNextAliasAllocation();
        ExpectPacket(":X1070133AN02010D000004;");
        ExpectPacket(":X1910033AN02010D000004;");
        secondNode_.reset(
            new DefaultAsyncNode(ifCan_.get(), TEST_NODE_ID + 1));
        Wait();
    }

    ~TwoNodeTest()
    {
        Wait();
    }

protected:
    std::unique_ptr<DefaultAsyncNode> secondNode_;
};

TEST_F(TwoNodeTest, VerifyNodeIdGlobalTwoNodes)
{
    ExpectPacket(":X1917022AN02010d000003;");
    ExpectPacket(":X1917033AN02010d000004;");
    SendPacket(":X19490997N;");
    Wait();
    Mock::VerifyAndClear(&canBus_);
    // Same thing again.
    ExpectPacket(":X1917022AN02010d000003;");
    ExpectPacket(":X1917033AN02010d000004;");
    SendPacket(":X19490997N;");
}

TEST_F(TwoNodeTest, VerifyNodeIdGlobalTwoNodesWithNodeId)
{
    PrintAllPackets();
    SendPacketAndExpectResponse(":X19490997N02010d000004;",
                                ":X1917033AN02010d000004;");
    SendPacketAndExpectResponse(":X19490997N02010d000003;",
                                ":X1917022AN02010d000003;");
    SendPacket(":X19490997N02010d000002;");  // No response.
}

TEST_F(AsyncIfTest, VerifyNodeIdGlobalNoNodes)
{
    PrintAllPackets();
    SendPacket(":X1949033AN;");
    // Try once more to check proper release.
    SendPacket(":X1949033AN;");
}

TEST_F(TwoNodeTest, VerifyNodeIdAddressed)
{
    SendPacketAndExpectResponse(":X19488997N022A;",
                                ":X1917022AN02010d000003;");
    // With mismatched node id.
    SendPacketAndExpectResponse(":X19488997N033A02010d000003;",
                                ":X1917033AN02010d000004;");
    // With correct node id.
    SendPacketAndExpectResponse(":X19488997N033A02010d000004;",
                                ":X1917033AN02010d000004;");
    // Nonexistant node.
    SendPacket(":X19488997N044C;");  // No response.
}

} // namespace NMRAnet
