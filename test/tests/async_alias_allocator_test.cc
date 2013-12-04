#include "utils/async_if_test_helper.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"

namespace NMRAnet
{
static const NodeID TEST_NODE_ID = 0x02010d000003ULL;

class AsyncAliasAllocatorTest : public AsyncIfTest
{
protected:
    AsyncAliasAllocatorTest() : alias_allocator_(TEST_NODE_ID, if_can_.get())
    {
    }

    void SetSeed(unsigned seed) {
        alias_allocator_.seed_ = seed;
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
    ExpectPacket(":X17020555N;");
    ExpectPacket(":X1610D555N;");
    ExpectPacket(":X15000555N;");
    ExpectPacket(":X14003555N;");
    // ExpectPacket(":X17020B84N;");
    alias_allocator_.empty_aliases()->Release(&info);
    Wait();

    ExpectPacket(":X10700555N;");
    TypedSyncAllocation<AliasInfo> ialloc(alias_allocator_.reserved_aliases());
    EXPECT_EQ(0x555U, ialloc.result()->alias);
    EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
}

TEST_F(AsyncAliasAllocatorTest, TestDelay)
{
    SetSeed(0x555);
    AliasInfo info;
    ExpectPacket(":X17020555N;");
    ExpectPacket(":X1610D555N;");
    ExpectPacket(":X15000555N;");
    ExpectPacket(":X14003555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    Wait();
    usleep(150000);
    ExpectPacket(":X10700555N;");
    usleep(60000);
}

TEST_F(AsyncAliasAllocatorTest, AllocateMultiple)
{
    SetSeed(0x555);
    AliasInfo info;
    AliasInfo info2;
    ExpectPacket(":X17020555N;");
    ExpectPacket(":X1610D555N;");
    ExpectPacket(":X15000555N;");
    ExpectPacket(":X14003555N;");
    ExpectPacket(":X10700555N;");
    alias_allocator_.empty_aliases()->Release(&info);
    {
        TypedSyncAllocation<AliasInfo> ialloc(
            alias_allocator_.reserved_aliases());
        EXPECT_EQ(0x555U, ialloc.result()->alias);
        EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
    }
    SetSeed(0xAAA);
    ExpectPacket(":X17020AAAN;");
    ExpectPacket(":X1610DAAAN;");
    ExpectPacket(":X15000AAAN;");
    ExpectPacket(":X14003AAAN;");
    ExpectPacket(":X10700AAAN;");
    alias_allocator_.empty_aliases()->Release(&info2);
    {
        TypedSyncAllocation<AliasInfo> ialloc(
            alias_allocator_.reserved_aliases());
        EXPECT_EQ(0xAAAU, ialloc.result()->alias);
        EXPECT_EQ(AliasInfo::STATE_RESERVED, ialloc.result()->state);
    }
}

} // namespace NMRAnet
