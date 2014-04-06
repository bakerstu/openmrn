#include "utils/test_main.hxx"

#include "executor/Timer.hxx"

using ::testing::ElementsAre;

class TimerTest : public ::testing::Test
{
protected:
    vector<Timer *> active_list(ActiveTimers *timers)
    {
        vector<Timer *> t;
        Timer *current_timer = static_cast<Timer *>(timers->activeTimers_.next);
        while (current_timer)
        {
            t.push_back(current_timer);
            current_timer = static_cast<Timer *>(current_timer->next);
        }
        return t;
    }
};

class CountingTimer : public Timer
{
public:
    CountingTimer(ActiveTimers *parent)
        : Timer(parent)
        , count_(0)
    {
    }

    ///@returns the number of times the timeout method was called.
    int count()
    {
        return count_;
    }

    long long timeout()
    {
        ++count_;
        return NONE;
    }

    bool is_active()
    {

        return isActive_;
    }

    bool is_expired()
    {

        return isExpired_;
    }

private:
    int count_;
};

class RestartingTimer : public CountingTimer
{
public:
    RestartingTimer(ActiveTimers *parent)
        : CountingTimer(parent)
        , stop_(false)
    {
    }

    long long timeout()
    {
        CountingTimer::timeout();
        return stop_ ? NONE : RESTART;
    }

    // Instructs the timer not to restart the period upon wakeup.
    void stop()
    {
        stop_ = true;
    }

private:
    // set to true to not restart the timer.
    bool stop_;
};

TEST(SemTest, TestExpire)
{
    OSSem sem;
    EXPECT_EQ(-1, sem.timedwait(MSEC_TO_NSEC(20)));
    EXPECT_EQ(ETIMEDOUT, errno);
}

TEST_F(TimerTest, TestExpire)
{
    ActiveTimers tim(&g_executor);
    CountingTimer t1(&tim);
    t1.start(MSEC_TO_NSEC(30));
    EXPECT_TRUE(t1.is_active());
    usleep(60000);
    EXPECT_EQ(0, tim.get_next_timeout());
    EXPECT_FALSE(t1.is_active());
    wait_for_main_executor();
}

TEST_F(TimerTest, Simple)
{
    CountingTimer t1(g_executor.active_timers());
    EXPECT_EQ(0, t1.count());
    usleep(20000);
    EXPECT_EQ(0, t1.count());
    EXPECT_FALSE(t1.is_active());
    t1.start(MSEC_TO_NSEC(30));
    EXPECT_TRUE(t1.is_active());
    EXPECT_LT(MSEC_TO_NSEC(20), g_executor.active_timers()->get_next_timeout());
    EXPECT_GT(MSEC_TO_NSEC(40), g_executor.active_timers()->get_next_timeout());
    EXPECT_EQ(0, t1.count());
    usleep(20000);
    wait_for_main_executor();
    EXPECT_EQ(0, t1.count());
    usleep(20000);
    EXPECT_EQ(1, t1.count());
    EXPECT_FALSE(t1.is_active());
    EXPECT_LT(SEC_TO_NSEC(1800),
              g_executor.active_timers()->get_next_timeout());
}

TEST_F(TimerTest, TwoTimers)
{
    CountingTimer t1(g_executor.active_timers());
    CountingTimer t2(g_executor.active_timers());
    EXPECT_EQ(0, t1.count());
    EXPECT_EQ(0, t2.count());
    t1.start(MSEC_TO_NSEC(10));
    EXPECT_THAT(active_list(g_executor.active_timers()), ElementsAre(&t1));
    t2.start(MSEC_TO_NSEC(20));
    EXPECT_THAT(active_list(g_executor.active_timers()), ElementsAre(&t1, &t2));
    EXPECT_TRUE(t1.is_active());
    EXPECT_TRUE(t2.is_active());
    usleep(15000);
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(0, t2.count());
    usleep(7000);
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(1, t2.count());

    // Let's restart one of the timers.
    t2.restart();
    usleep(15000);
    EXPECT_EQ(1, t2.count());
    usleep(7000);
    EXPECT_EQ(2, t2.count());
    wait_for_main_executor();
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(2, t2.count());
}

TEST_F(TimerTest, ExpireTogether)
{
    // In this test we create two timers that will expire roughly at the same
    // time, and a third expiring later. We block the main executor to pile up
    // the two timers. Then release it, which should trigger them both. The
    // third will show up later.
    CountingTimer t1(g_executor.active_timers());
    CountingTimer t2(g_executor.active_timers());
    CountingTimer t3(g_executor.active_timers());
    t1.start(MSEC_TO_NSEC(10));
    t2.start(MSEC_TO_NSEC(10) - 1);
    t3.start(MSEC_TO_NSEC(20));
    usleep(5000);
    BlockExecutor b;
    g_executor.add(&b);
    b.wait_for_blocked();

    EXPECT_EQ(0, t1.count());
    EXPECT_EQ(0, t2.count());
    EXPECT_EQ(0, t3.count());

    usleep(10000);

    EXPECT_EQ(0, t1.count());
    EXPECT_EQ(0, t2.count());
    EXPECT_EQ(0, t3.count());
    b.release_block();
    wait_for_main_executor();
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(1, t2.count());
    EXPECT_EQ(0, t3.count());
    usleep(10000);
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(1, t2.count());
    EXPECT_EQ(1, t3.count());
}

TEST_F(TimerTest, Wakeup)
{
    CountingTimer t1(g_executor.active_timers());
    CountingTimer t2(g_executor.active_timers());
    t1.start(MSEC_TO_NSEC(1000));
    t2.start(MSEC_TO_NSEC(30));
    usleep(10000);
    EXPECT_EQ(0, t1.count());
    t1.trigger();
    wait_for_main_executor();
    EXPECT_EQ(1, t1.count());
    EXPECT_EQ(0, t2.count());
    usleep(25000);
    EXPECT_EQ(1, t2.count());
}

TEST_F(TimerTest, Restart)
{
    RestartingTimer t1(g_executor.active_timers());
    t1.start(MSEC_TO_NSEC(10));
    usleep(5000);
    EXPECT_EQ(0, t1.count());
    usleep(10000);
    EXPECT_EQ(1, t1.count());
    usleep(20000);
    EXPECT_EQ(3, t1.count());
    EXPECT_TRUE(t1.is_active());
    t1.stop();
    t1.trigger();
    wait_for_main_executor();
    EXPECT_FALSE(t1.is_active());
}

TEST_F(TimerTest, NoRestartSkew)
{
    RestartingTimer t1(g_executor.active_timers());
    t1.start(MSEC_TO_NSEC(10));
    usleep(55000);
    EXPECT_EQ(5, t1.count());
    // We block the executor for 10 periods. We expect the triggers to have
    // piled up, so when we release the block all the ticks should happen
    // immediately.
    BlockExecutor b;
    g_executor.add(&b);
    b.wait_for_blocked();
    EXPECT_EQ(5, t1.count());
    usleep(100000);

    // All of this happens within a short time, increasing the cound form 5 to
    // 15.
    EXPECT_EQ(5, t1.count());
    b.release_block();
    wait_for_main_executor();
    EXPECT_EQ(15, t1.count());

    usleep(30000);
    EXPECT_EQ(18, t1.count());
    t1.stop();
    t1.trigger();
    wait_for_main_executor();
    EXPECT_FALSE(t1.is_active());
}
