#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetAsyncDefaultNode.hxx"

namespace NMRAnet
{

TEST_F(AsyncIfTest, CreateNodeSendsInitializer)
{
    //ExpectPacket(":X1070122AN02010d000003"); // AMD frame
    ExpectPacket(":X1910022AN02010d000003;"); // initialization complete
    //CreateAllocatedAlias();
    LOG(INFO, "before");
    DefaultAsyncNode node(if_can_.get(), TEST_NODE_ID);
    // Technically there is a race condition here. The initialization could
    // happen before we get to this expectation.
    EXPECT_FALSE(node.is_initialized());
    if_can_->add_addressed_message_support(2);
    LOG(INFO, "after");
    Wait();
    EXPECT_TRUE(node.is_initialized());
    EXPECT_EQ(&node, if_can_->lookup_local_node(TEST_NODE_ID));
}

TEST_F(AsyncIfTest, TwoNodesInitialize)
{
    if_can_->add_addressed_message_support(2);
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

TEST_F(AsyncIfTest, WriteHelperByMTI)
{
    if_can_->add_addressed_message_support(2);
    ExpectPacket(":X1910022AN02010d000003;"); // initialization complete
    DefaultAsyncNode node(if_can_.get(), TEST_NODE_ID);
    Wait();  // for initialized

    WriteHelper helper;
    SyncNotifiable n;
    ExpectPacket(":X195B422AN0102030405060708;");
    helper.WriteAsync(&node, If::MTI_EVENT_REPORT,
                      WriteHelper::global(),
                      EventIdToBuffer(0x0102030405060708ULL), &n);
    n.WaitForNotification();
}

} // namespace NMRAnet
