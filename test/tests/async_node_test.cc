#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetAsyncDefaultNode.hxx"

namespace NMRAnet
{

TEST_F(AsyncIfTest, CreateNodeSendsInitializer)
{
    //expect_packet(":X1070122AN02010d000003"); // AMD frame
    expect_packet(":X1910022AN02010d000003;"); // initialization complete
    //CreateAllocatedAlias();
    LOG(INFO, "before");
    DefaultAsyncNode node(ifCan_.get(), TEST_NODE_ID);
    // Technically there is a race condition here. The initialization could
    // happen before we get to this expectation.
    EXPECT_FALSE(node.is_initialized());
    ifCan_->add_addressed_message_support(2);
    LOG(INFO, "after");
    wait();
    EXPECT_TRUE(node.is_initialized());
    EXPECT_EQ(&node, ifCan_->lookup_local_node(TEST_NODE_ID));
}

TEST_F(AsyncIfTest, TwoNodesInitialize)
{
    ifCan_->add_addressed_message_support(2);
    expect_packet(":X1910022AN02010d000003;"); // initialization complete
    CreateAllocatedAlias();
    LOG(INFO, "before");
    DefaultAsyncNode node(ifCan_.get(), TEST_NODE_ID);
    LOG(INFO, "after");
    wait();
    expect_packet(":X1070133AN02010d000004;"); // AMD frame
    expect_packet(":X1910033AN02010d000004;"); // initialization complete
    ExpectNextAliasAllocation();
    DefaultAsyncNode node2(ifCan_.get(), TEST_NODE_ID + 1);
    wait();
}

TEST_F(AsyncIfTest, WriteHelperByMTI)
{
    ifCan_->add_addressed_message_support(2);
    expect_packet(":X1910022AN02010d000003;"); // initialization complete
    DefaultAsyncNode node(ifCan_.get(), TEST_NODE_ID);
    wait();  // for initialized

    WriteHelper helper;
    SyncNotifiable n;
    expect_packet(":X195B422AN0102030405060708;");
    helper.WriteAsync(&node, If::MTI_EVENT_REPORT,
                      WriteHelper::global(),
                      EventIdToBuffer(0x0102030405060708ULL), &n);
    n.WaitForNotification();
}

} // namespace NMRAnet
