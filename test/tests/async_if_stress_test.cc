#include "utils/async_if_test_helper.hxx"

#include <set>

#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet/IfCan.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "os/OS.hxx"

namespace nmranet
{

Executor<1> g1_executor("g1_exec", 0, 2000);
Executor<1> g2_executor("g2_exec", 0, 2000);
Executor<1> g3_executor("g3_exec", 0, 2000);
Executor<1> g4_executor("g4_exec", 0, 2000);

Executor<1>* round_execs[] = {&g1_executor, &g2_executor,
                              &g3_executor, &g4_executor};

/** This class will create an AsyncIf, two virtual nodes on it, and send one
 * unaddressed global packet each. */
class TestNode
{
public:
    TestNode(NodeID node_id)
        : nodeId_(node_id),
          ifCan_(round_execs[(node_id >> 1) & 3], &can_hub0, 10, 10, 2)
    {
    }

    void start(BarrierNotifiable* done)
    {
        ifCan_.add_addressed_message_support();
        ifCan_.set_alias_allocator(new AliasAllocator(nodeId_, &ifCan_));
        {
            // Adds one alias buffer to the alias allocation flow.
            auto* b = ifCan_.alias_allocator()->alloc();
            ifCan_.alias_allocator()->send(b);
        }
        auto* b = ifCan_.global_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_EVENT_REPORT, nodeId_,
                         EventIdToBuffer(nodeId_));
        b->set_done(done->new_child());
        ifCan_.global_message_write_flow()->send(b);
        b = ifCan_.global_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_EVENT_REPORT, nodeId_ + 1,
                         EventIdToBuffer(nodeId_ + 1));
        b->set_done(done->new_child());
        ifCan_.global_message_write_flow()->send(b);
    }

    ~TestNode()
    {
        //ifCan_.alias_allocator()->TEST_finish_pending_allocation();
        Executor<1>* e = round_execs[(nodeId_ >> 1) & 3];
        while(!e->empty() || !g_executor.empty()
              || !ifCan_.alias_allocator()->is_waiting()
              || !ifCan_.dispatcher()->is_waiting()
              || !ifCan_.frame_dispatcher()->is_waiting()
              ) {
/*              !ifCan_.frame_dispatcher()->IsNotStarted() ||
              !ifCan_.dispatcher()->IsNotStarted() ||
              !can_hub0.empty() ||
              !gc_hub0.empty()) {*/
            usleep(100);
        }
    }

private:
    NodeID nodeId_;
    IfCan ifCan_;
//    AliasInfo testAlias_;
};

class Stats : public CanHubPortInterface
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
        can_hub0.register_port(this);
        timer_.start(MSEC_TO_NSEC(1000));
    }

    ~Stats()
    {
        timer_.stop();
        can_hub0.unregister_port(this);
    }

    void send(Buffer<CanHubData>* b, unsigned /*priority*/)
    {
        const struct can_frame* f = b->data();
        AutoReleaseBuffer<CanHubData> releaser(b);
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
                 g_executor.empty()))
        {
            usleep(100);
            wait();
        }
        wait();
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
    barrier_.maybe_done();
    n_.wait_for_notification();
}

TEST_F(AsyncIfStressTest, onenode)
{
    CreateNodes(1);
    barrier_.maybe_done();
    n_.wait_for_notification();
}

TEST_F(AsyncIfStressTest, tennodes)
{
    CreateNodes(10);
    barrier_.maybe_done();
    n_.wait_for_notification();
}

TEST_F(AsyncIfStressTest, hundrednodes)
{
    CreateNodes(100);
    barrier_.maybe_done();
    n_.wait_for_notification();
}

TEST_F(AsyncIfStressTest, DISABLED_thousandnodes)
{
    CreateNodes(1000);
    barrier_.maybe_done();
    n_.wait_for_notification();
}

} // namespace nmranet
