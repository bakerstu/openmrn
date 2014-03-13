#include <map>

#include "utils/async_if_test_helper.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/NMRAnetAliasCache.hxx"

namespace NMRAnet
{
class AsyncAliasAllocatorTest : public AsyncIfTest
{
protected:
    AsyncAliasAllocatorTest()
        : b_(nullptr)
        , alias_allocator_(TEST_NODE_ID, ifCan_.get())
    {
    }

    ~AsyncAliasAllocatorTest()
    {
        wait();
    }

    void set_seed(unsigned seed, AsyncAliasAllocator *alloc = nullptr)
    {
        if (!alloc)
            alloc = &alias_allocator_;
        alloc->seed_ = seed;
    }

    unsigned next_seed(AsyncAliasAllocator *alloc = nullptr)
    {
        if (!alloc)
            alloc = &alias_allocator_;
        alloc->next_seed();
        return alloc->seed_;
    }

    /** Takes the next allocated alias from the async alias allocator. Waits
     * until one is available. The alias will be saved into the buffer b_. */
    void get_next_alias()
    {
        while (b_ = static_cast<Buffer<AliasInfo> *>(
                   alias_allocator_.reserved_aliases()->next().item),
               !b_)
        {
            usleep(10);
        }
    }

    Buffer<AliasInfo> *b_;
    AsyncAliasAllocator alias_allocator_;
};

TEST_F(AsyncAliasAllocatorTest, SetupTeardown)
{
}

TEST_F(AsyncAliasAllocatorTest, AllocateOne)
{
    set_seed(0x555);

    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");

    mainBufferPool->alloc(&b_);
    alias_allocator_.send(b_);
    b_ = nullptr;

    /** @todo(balazs.racz) this should be after the wait because there should be
     * a delay before sending it. */
    expect_packet(":X10700555N;");
    wait();

    get_next_alias();
    ASSERT_TRUE(b_);
    EXPECT_EQ(0x555U, b_->data()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, b_->data()->state);
}
#if 0
TEST_F(AsyncAliasAllocatorTest, TestDelay)
{
    set_seed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();
    usleep(150000);
    expect_packet(":X10700555N;");
    usleep(60000);
}
#endif

TEST_F(AsyncAliasAllocatorTest, AllocateMultiple)
{
    set_seed(0x555);
    mainBufferPool->alloc(&b_);
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    expect_packet(":X10700555N;");
    alias_allocator_.send(b_);
    b_ = nullptr;
    get_next_alias();
    EXPECT_EQ(0x555U, b_->data()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, b_->data()->state);

    set_seed(0xAAA);
    expect_packet(":X17020AAAN;");
    expect_packet(":X1610DAAAN;");
    expect_packet(":X15000AAAN;");
    expect_packet(":X14003AAAN;");
    expect_packet(":X10700AAAN;");

    mainBufferPool->alloc(&b_);
    alias_allocator_.send(b_);
    b_ = nullptr;

    /* Conflicts with the previous alias to be tested. That's not a problem at
     * this point however, because that alias has already left the
     * allocator. */
    send_packet(
        ":X10700555N;");

    get_next_alias();
    EXPECT_EQ(0xAAAU, b_->data()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, b_->data()->state);
}

#if 0
TEST_F(AsyncAliasAllocatorTest, AllocationConflict)
{
    set_seed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();
    set_seed(0xAA5);
    send_packet(":X10700555N;");
    expect_packet(":X17020AA5N;");
    expect_packet(":X1610DAA5N;");
    expect_packet(":X15000AA5N;");
    expect_packet(":X14003AA5N;");
    expect_packet(":X10700AA5N;");
    TypedSyncAllocation<AliasInfo> ialloc(alias_allocator_.reserved_aliases());
    EXPECT_EQ(0xAA5U, b_->data()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, b_->data()->state);
    // This one should be marked as reserved.
    EXPECT_EQ(AliasCache::RESERVED_ALIAS_NODE_ID,
              ifCan_->local_aliases()->lookup(NodeAlias(0xAA5)));
    // This one should be unknown.
    EXPECT_EQ(0U, ifCan_->local_aliases()->lookup(NodeAlias(0x555)));
}

TEST_F(AsyncAliasAllocatorTest, LateAllocationConflict)
{
    set_seed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();
    set_seed(0xAA5);
    usleep(100000);
    send_packet(":X10700555N;");
    expect_packet(":X17020AA5N;");
    expect_packet(":X1610DAA5N;");
    expect_packet(":X15000AA5N;");
    expect_packet(":X14003AA5N;");
    wait();
    usleep(100000);
    send_packet(":X10700555N;");
    expect_packet(":X10700AA5N;");
    TypedSyncAllocation<AliasInfo> ialloc(alias_allocator_.reserved_aliases());
    EXPECT_EQ(0xAA5U, b_->data()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, b_->data()->state);
}
#endif

TEST_F(AsyncAliasAllocatorTest, GenerationCycleLength)
{
    std::map<unsigned, bool> seen_seeds;
    // Checks that the first 4096 aliases generated are all different.
    for (int i = 0; i < 4096; i++)
    {
        unsigned current_seed = next_seed();
        EXPECT_GE(0xfffU, current_seed);
        EXPECT_FALSE(seen_seeds[current_seed]);
        seen_seeds[current_seed] = true;
    }
    // And then we find a repeat.
    EXPECT_TRUE(seen_seeds[next_seed()]);
}

TEST_F(AsyncAliasAllocatorTest, DifferentGenerated)
{
    set_seed(0x555);
    AsyncAliasAllocator other(TEST_NODE_ID + 13, ifCan_.get());
    set_seed(0x555, &other);
    // Checks that the two alias allocators generate different values after a
    // conflict.
    EXPECT_NE(next_seed(), next_seed(&other));
    EXPECT_NE(next_seed(), next_seed(&other));
    // Makes sure 'other' disappears from the executor before destructing it.
    wait();
}
} // namespace NMRAnet
