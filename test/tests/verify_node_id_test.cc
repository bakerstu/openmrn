#include "utils/async_if_test_helper.hxx"

namespace nmranet
{

TEST_F(AsyncNodeTest, VerifyNodeIdGlobalSingleNode)
{
    send_packet_and_expect_response(":X1949033AN;", ":X1917022AN02010d000003;");
}

class TwoNodeTest : public AsyncNodeTest
{
public:
    TwoNodeTest()
    {
        create_allocated_alias();
        expect_next_alias_allocation();
        expect_packet(":X1070133AN02010D000004;");
        expect_packet(":X1910033AN02010D000004;");
        secondNode_.reset(
            new DefaultAsyncNode(ifCan_.get(), TEST_NODE_ID + 1));
        wait();
    }

    ~TwoNodeTest()
    {
        wait();
    }

protected:
    std::unique_ptr<DefaultAsyncNode> secondNode_;
};

TEST_F(TwoNodeTest, VerifyNodeIdGlobalTwoNodes)
{
    print_all_packets();
    expect_packet(":X1917022AN02010d000003;");
    expect_packet(":X1917033AN02010d000004;");
    send_packet(":X19490997N;");
    wait();
    // Same thing again.
    expect_packet(":X1917022AN02010d000003;");
    expect_packet(":X1917033AN02010d000004;");
    send_packet(":X19490997N;");
    wait();
}

TEST_F(TwoNodeTest, VerifyNodeIdGlobalTwoNodesWithNodeId)
{
    print_all_packets();
    send_packet_and_expect_response(":X19490997N02010d000004;",
                                ":X1917033AN02010d000004;");
    send_packet_and_expect_response(":X19490997N02010d000003;",
                                ":X1917022AN02010d000003;");
    send_packet(":X19490997N02010d000002;");  // No response.
}

TEST_F(AsyncIfTest, VerifyNodeIdGlobalNoNodes)
{
    print_all_packets();
    send_packet(":X1949033AN;");
    // Try once more to check proper release.
    send_packet(":X1949033AN;");
}

TEST_F(TwoNodeTest, VerifyNodeIdAddressed)
{
    send_packet_and_expect_response(":X19488997N022A;",
                                ":X1917022AN02010d000003;");
    // With mismatched node id.
    send_packet_and_expect_response(":X19488997N033A02010d000003;",
                                ":X1917033AN02010d000004;");
    // With correct node id.
    send_packet_and_expect_response(":X19488997N033A02010d000004;",
                                ":X1917033AN02010d000004;");
    // Nonexistant node.
    send_packet(":X19488997N044C;");  // No response.
}

} // namespace nmranet
