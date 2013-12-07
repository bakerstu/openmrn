#include "utils/async_if_test_helper.hxx"

#include <set>

#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet/AsyncIfCan.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "os/OS.hxx"

namespace NMRAnet
{

ThreadExecutor g1_executor("g1_exec", 0, 2000);
ThreadExecutor g2_executor("g2_exec", 0, 2000);
ThreadExecutor g3_executor("g3_exec", 0, 2000);
ThreadExecutor g4_executor("g4_exec", 0, 2000);

Executor* round_execs[] = {&g1_executor, &g2_executor,
                           &g3_executor, &g4_executor};

/** This class will create an AsyncIf, two virtual nodes on it, and send one
 * unaddressed global packet each. */
class TestNode
{
public:
    TestNode(NodeID node_id)
        : nodeId_(node_id),
          ifCan_(round_execs[(node_id >> 1) & 3], &can_pipe0, 10, 10)
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

class Stats : public PipeMember
{
public:
    Stats()
        : timer_(&Stats::timer_callback, this, nullptr),
          numFrames_(0),
          badFrames_(0),
          cidFrames_(0),
          ridFrames_(0),
          amdFrames_(0),
          eventFrames_(0)
    {
        can_pipe0.RegisterMember(this);
        timer_.start(MSEC_TO_NSEC(1000));
    }

    ~Stats()
    {
        can_pipe0.UnregisterMember(this);
    }

    virtual void write(const void* buf, size_t count)
    {
        const struct can_frame* f = static_cast<const struct can_frame*>(buf);
        while (count >= sizeof(*f))
        {
            handle_frame(f);
            f++;
            count -= sizeof(*f);
        }
        HASSERT(!count);
    }

    void handle_frame(const struct can_frame* f)
    {
        OSMutexLock h(&lock_);
        numFrames_++;
        if (!IS_CAN_FRAME_EFF(*f))
        {
            badFrames_++;
            return;
        }
        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        uint32_t high_fields = id >> 24;
        if ((high_fields & 0x14) == 0x14)
        {
            ++cidFrames_;
        }
        uint16_t alias = id & 0xFFF;
        seenAliases_.insert(alias);
        uint32_t mid_fields = id >> 12;
        if (mid_fields == 0x10700)
        {
            ridFrames_++;
        }
        else if (mid_fields == 0x10701)
        {
            amdFrames_++;
        }
        else if (mid_fields == 0x195B4)
        {
            eventFrames_++;
        }
    }

    static long long timer_callback(void* me, void*)
    {
        Stats* s = static_cast<Stats*>(me);
        s->print_stats();
        return OS_TIMER_RESTART;
    }

    void print_stats()
    {
        OSMutexLock h(&lock_);
        LOG(INFO, "%d aliases (%u/%u cnf), %d frame: %d CID, %d RID, "
                  "%d AMD, %d event.",
            seenAliases_.size(), g_alias_test_conflicts, g_alias_use_conflicts,
            numFrames_, cidFrames_, ridFrames_, amdFrames_, eventFrames_);
    }

private:
    OSMutex lock_;
    OSTimer timer_;
    std::set<uint16_t> seenAliases_;
    size_t numFrames_;
    size_t badFrames_;
    size_t cidFrames_;
    size_t ridFrames_;
    size_t amdFrames_;
    size_t eventFrames_;
};

class AsyncIfStressTest : public AsyncIfTest
{
protected:
    AsyncIfStressTest() : barrier_(&n_), nextNodeID_(0x050201000000ULL)
    {
    }

    ~AsyncIfStressTest()
    {
        while (!(g1_executor.empty() && g2_executor.empty() &&
                 g3_executor.empty() && g4_executor.empty() &&
                 DefaultWriteFlowExecutor()->empty()))
        {
            usleep(100);
        }
        Wait();
    }

    void CreateNodes(int count)
    {
        int start = nodes_.size();
        for (int i = 0; i < count; ++i)
        {
            nodes_.push_back(
                std::unique_ptr<TestNode>(new TestNode(nextNodeID_)));
            nextNodeID_ += 2;
        }
        for (size_t i = start; i < nodes_.size(); ++i)
        {
            nodes_[i]->start(&barrier_);
        }
    }

    Stats stats_;
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

TEST_F(AsyncIfStressTest, DISABLED_thousandnodes)
{
    CreateNodes(1000);
    barrier_.MaybeDone();
    n_.WaitForNotification();
}

} // namespace NMRAnet
