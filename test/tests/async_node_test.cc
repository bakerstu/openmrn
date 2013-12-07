#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetAsyncDefaultNode.hxx"

namespace NMRAnet
{

TEST_F(AsyncIfTest, CreateNodeSendsInitializer)
{
    if_can_->AddWriteFlows(2, 2);
    //ExpectPacket(":X1070122AN02010d000003"); // AMD frame
    ExpectPacket(":X1910022AN02010d000003;"); // initialization complete
    //CreateAllocatedAlias();
    LOG(INFO, "before");
    DefaultAsyncNode node(if_can_.get(), TEST_NODE_ID);
    // Technically there is a race condition here. The initialization could
    // happen before we get to this expectation.
    EXPECT_FALSE(node.is_initialized());
    LOG(INFO, "after");
    Wait();
    EXPECT_TRUE(node.is_initialized());
}

TEST_F(AsyncIfTest, TwoNodesInitialize)
{
    if_can_->AddWriteFlows(2, 2);
    ExpectPacket(":X1910022AN02010d000003;"); // initialization complete
    CreateAllocatedAlias();
    LOG(INFO, "before");
    DefaultAsyncNode node(if_can_.get(), TEST_NODE_ID);
    LOG(INFO, "after");
    Wait();
    ExpectPacket(":X1070133AN02010d000004;"); // AMD frame
    ExpectPacket(":X1910033AN02010d000004;"); // initialization complete
    ExpectNextAliasAllocation();
    DefaultAsyncNode node2(if_can_.get(), TEST_NODE_ID + 1);
    Wait();
}

} // namespace NMRAnet
