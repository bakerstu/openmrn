#include "utils/test_main.hxx"

#include "executor/Timer.hxx"

class TimerTest : public ::testing::Test
{
protected:
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
    EXPECT_EQ(0, t1.count());
    usleep(20000);
    EXPECT_EQ(1, t1.count());
    EXPECT_FALSE(t1.is_active());
    EXPECT_LT(SEC_TO_NSEC(1800),
              g_executor.active_timers()->get_next_timeout());
}
