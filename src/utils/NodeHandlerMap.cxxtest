#include "utils/test_main.hxx"

#include "utils/NodeHandlerMap.hxx"

namespace
{

struct Node;
struct Handler;
typedef TypedNodeHandlerMap<Node, Handler> MapType;

Node* GetNode(uint32_t n)
{
    return reinterpret_cast<Node*>(n);
}

Handler* GetHandler(uint32_t n)
{
    return reinterpret_cast<Handler*>(n);
}

/* Certain map implementations don't like to be constructed without size.
TEST(NodeHandlerMap, CreateDestroy)
{
    MapType map;
}*/

TEST(NodeHandlerMap, CreateDestroySz)
{
    MapType map(15);
}

TEST(NodeHandlerMap, InsertLookupSimple)
{
    MapType map(15);
    map.insert(GetNode(42), 3, GetHandler(11));
    map.insert(GetNode(42), 5, GetHandler(12));
    map.insert(GetNode(24), 3, GetHandler(13));
    EXPECT_EQ(GetHandler(11), map.lookup(GetNode(42), 3));
    EXPECT_EQ(GetHandler(12), map.lookup(GetNode(42), 5));
    EXPECT_EQ(GetHandler(13), map.lookup(GetNode(24), 3));
    EXPECT_EQ(nullptr, map.lookup(GetNode(24), 5));
    EXPECT_EQ(nullptr, map.lookup(nullptr, 5));
}

TEST(NodeHandlerMap, InsertLookupFallback)
{
    MapType map(15);
    map.insert(GetNode(42), 3, GetHandler(11));
    map.insert(nullptr, 3, GetHandler(12));
    map.insert(GetNode(24), 5, GetHandler(13));
    EXPECT_EQ(GetHandler(11), map.lookup(GetNode(42), 3));
    EXPECT_EQ(GetHandler(12), map.lookup(GetNode(24), 3));

    EXPECT_EQ(GetHandler(13), map.lookup(GetNode(24), 5));
    EXPECT_EQ(nullptr, map.lookup(GetNode(42), 5));
}

}  // namespace
