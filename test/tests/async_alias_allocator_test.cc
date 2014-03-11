#include <map>

#include "utils/async_if_test_helper.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/NMRAnetAliasCache.hxx"

namespace NMRAnet
{
class AsyncAliasAllocatorTest : public AsyncIfTest
{
protected:
    AsyncAliasAllocatorTest() : alias_allocator_(TEST_NODE_ID, ifCan_.get())
    {
    }

    ~AsyncAliasAllocatorTest()
    {
        wait();
    }

    void SetSeed(unsigned seed, AsyncAliasAllocator* alloc = nullptr)
    {
        if (!alloc) alloc = &alias_allocator_;
        alloc->seed_ = seed;
    }

    unsigned NextSeed(AsyncAliasAllocator* alloc = nullptr)
    {
        if (!alloc) alloc = &alias_allocator_;
        alloc->NextSeed();
        return alloc->seed_;
    }

    AsyncAliasAllocator alias_allocator_;
};

TEST_F(AsyncAliasAllocatorTest, SetupTeardown)
{
}

TEST_F(AsyncAliasAllocatorTest, AllocateOne)
{
    SetSeed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();

    expect_packet(":X10700555N;");
    TypedSyncAllocation<AliasInfo> ialloc(alias_allocator_.reserved_aliases());
    EXPECT_EQ(0x555U, ialloc.result()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
}

TEST_F(AsyncAliasAllocatorTest, TestDelay)
{
    SetSeed(0x555);
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

TEST_F(AsyncAliasAllocatorTest, AllocateMultiple)
{
    SetSeed(0x555);
    AliasInfo info;
    AliasInfo info2;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    expect_packet(":X10700555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    {
        TypedSyncAllocation<AliasInfo> ialloc(
            alias_allocator_.reserved_aliases());
        EXPECT_EQ(0x555U, ialloc.result()->alias);
        EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
    }
    SetSeed(0xAAA);
    expect_packet(":X17020AAAN;");
    expect_packet(":X1610DAAAN;");
    expect_packet(":X15000AAAN;");
    expect_packet(":X14003AAAN;");
    expect_packet(":X10700AAAN;");
    alias_allocator_.empty_aliases()->Release(&info2);
    send_packet(
        ":X10700555N;"); // Conflicts with the previous alias to be tested.
    {
        TypedSyncAllocation<AliasInfo> ialloc(
            alias_allocator_.reserved_aliases());
        EXPECT_EQ(0xAAAU, ialloc.result()->alias);
        EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
    }
}

TEST_F(AsyncAliasAllocatorTest, AllocationConflict)
{
    SetSeed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();
    SetSeed(0xAA5);
    send_packet(":X10700555N;");
    expect_packet(":X17020AA5N;");
    expect_packet(":X1610DAA5N;");
    expect_packet(":X15000AA5N;");
    expect_packet(":X14003AA5N;");
    expect_packet(":X10700AA5N;");
    TypedSyncAllocation<AliasInfo> ialloc(alias_allocator_.reserved_aliases());
    EXPECT_EQ(0xAA5U, ialloc.result()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
    // This one should be marked as reserved.
    EXPECT_EQ(AliasCache::RESERVED_ALIAS_NODE_ID,
              ifCan_->local_aliases()->lookup(NodeAlias(0xAA5)));
    // This one should be unknown.
    EXPECT_EQ(0U, ifCan_->local_aliases()->lookup(NodeAlias(0x555)));
}

TEST_F(AsyncAliasAllocatorTest, LateAllocationConflict)
{
    SetSeed(0x555);
    AliasInfo info;
    expect_packet(":X17020555N;");
    expect_packet(":X1610D555N;");
    expect_packet(":X15000555N;");
    expect_packet(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    wait();
    SetSeed(0xAA5);
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
    EXPECT_EQ(0xAA5U, ialloc.result()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
}

TEST_F(AsyncAliasAllocatorTest, GenerationCycleLength)
{
    std::map<unsigned, bool> seen_seeds;
    // Checks that the first 4096 aliases generated are all different.
    for (int i = 0; i < 4096; i++)
    {
        unsigned current_seed = NextSeed();
        EXPECT_GE(0xfffU, current_seed);
        EXPECT_FALSE(seen_seeds[current_seed]);
        seen_seeds[current_seed] = true;
    }
    // And then we find a repeat.
    EXPECT_TRUE(seen_seeds[NextSeed()]);
}

TEST_F(AsyncAliasAllocatorTest, DifferentGenerated)
{
    SetSeed(0x555);
    AsyncAliasAllocator other(TEST_NODE_ID + 13, ifCan_.get());
    SetSeed(0x555, &other);
    // Checks that the two alias allocators generate different values after a
    // conflict.
    EXPECT_NE(NextSeed(), NextSeed(&other));
    EXPECT_NE(NextSeed(), NextSeed(&other));
    // Makes sure 'other' disappears from the executor before destructing it.
    wait();
}

} // namespace NMRAnet
