#include "utils/async_if_test_helper.hxx"

#include "nmranet/NMRAnetWriteFlow.hxx"

using ::testing::Field;
using ::testing::Pointee;
using ::testing::NotNull;

namespace NMRAnet
{

/** This class will create an AsyncIf, two virtual nodes on it, and send one
 * unaddressed global packet each. */
class TestNode
{
public:
    TestNode(NodeID node_id)
        : nodeId_(node_id), ifCan_(&g_executor, &can_pipe0, 10, 10)
    {
    }

    void start(BarrierNotifiable* done)
    {
        ifCan_.AddWriteFlows(2, 2);
        ifCan_.set_alias_allocator(new AsyncAliasAllocator(nodeId_, &ifCan_));
        ifCan_.alias_allocator()->empty_aliases()->Release(&testAlias_);
        WriteFlow* f = ifCan_.global_write_allocator()->TypedAllocateOrNull();
        HASSERT(f);
        f->WriteGlobalMessage(If::MTI_EVENT_REPORT, nodeId_,
                              EventIdToBuffer(nodeId_), done->NewChild());
        f = ifCan_.global_write_allocator()->TypedAllocateOrNull();
        HASSERT(f);
        f->WriteGlobalMessage(If::MTI_EVENT_REPORT, nodeId_ + 1,
                              EventIdToBuffer(nodeId_ + 1), done->NewChild());
    }

    ~TestNode()
    {
    }

private:
    NodeID nodeId_;
    AsyncIfCan ifCan_;
    AliasInfo testAlias_;
};

class AsyncIfStressTest : public AsyncIfTest
{
protected:
    AsyncIfStressTest() : barrier_(&n_), nextNodeID_(0x050201000000ULL)
    {
    }

    ~AsyncIfStressTest()
    {
        Wait();
    }

    void CreateNodes(int count)
    {
        int start = nodes_.size();
        for (int i = 0; i < count; ++i)
        {
            nodes_.push_back(std::unique_ptr<TestNode>(
                new TestNode(nextNodeID_)));
            nextNodeID_ += 2;
        }
        for (int i = start; i < nodes_.size(); ++i)
        {
            nodes_[i]->start(&barrier_);
        }
    }

    SyncNotifiable n_;
    BarrierNotifiable barrier_;
    NodeID nextNodeID_;
    vector<std::unique_ptr<TestNode>> nodes_;
};

TEST_F(AsyncIfStressTest, nonode)
{
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

TEST_F(AsyncIfStressTest, onenode)
{
    CreateNodes(1);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

TEST_F(AsyncIfStressTest, tennodes)
{
    CreateNodes(10);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

TEST_F(AsyncIfStressTest, hundrednodes)
{
    CreateNodes(100);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

} // namespace NMRAnet
