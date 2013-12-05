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
    TestNode(NodeID node_id, BarrierNotifiable* done)
        : ifCan_(&g_executor, &can_pipe0, 10, 10)
    {
        ifCan_.AddWriteFlows(2, 2);
        ifCan_.set_alias_allocator(
            new AsyncAliasAllocator(node_id, &ifCan_));
        ifCan_.alias_allocator()->empty_aliases()->Release(&testAlias_);
        WriteFlow* f = ifCan_.global_write_allocator()->TypedAllocateOrNull();
        HASSERT(f);
        f->WriteGlobalMessage(If::MTI_EVENT_REPORT, node_id,
                              EventIdToBuffer(node_id), done->NewChild());
        f = ifCan_.global_write_allocator()->TypedAllocateOrNull();
        HASSERT(f);
        f->WriteGlobalMessage(If::MTI_EVENT_REPORT, node_id + 1,
                              EventIdToBuffer(node_id + 1), done->NewChild());
    }

    ~TestNode()
    {
    }

private:
    AsyncIfCan ifCan_;
    AliasInfo testAlias_;
};

class AsyncIfStressTest : public AsyncIfTest
{
protected:
    AsyncIfStressTest() : barrier_(&n_), nextNodeID_(0x050201000000ULL)
    {
    }

    void CreateNodes(int count)
    {
        for (int i = 0; i < count; ++i)
        {
            nodes_.push_back(
                std::unique_ptr<TestNode>(new TestNode(nextNodeID_, &barrier_)));
            nextNodeID_ += 2;
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

TEST_F(AsyncIfStressTest, DISABLED_onenode)
{
    CreateNodes(1);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

TEST_F(AsyncIfStressTest, DISABLED_tennodes)
{
    CreateNodes(10);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

} // namespace NMRAnet
