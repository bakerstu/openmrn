#include <unistd.h>

#include "openlcb/IfTcp.hxx"
#include "openlcb/IfTcpImpl.hxx"
#include "openlcb/PIPClient.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "utils/FdUtils.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/async_if_test_helper.hxx"
#include "utils/format_utils.hxx"

using ::testing::_;
using ::testing::SaveArg;
using ::testing::StrictMock;

namespace openlcb
{

class MockHubPort : public HubPortInterface
{
public:
    MOCK_METHOD2(send, void(const string &message, unsigned priority));

    void send(Buffer<HubData> *b, unsigned priority) override
    {
        {
            GenMessage actual;
            actual.clear();
            EXPECT_TRUE(TcpDefs::parse_tcp_message(*b->data(), &actual));
            LOG(INFO,
                "[sent] 0x%012" PRIx64 " -> %012" PRIx64 " MTI %03x payload %s",
                actual.src.id, actual.dst.id, actual.mti,
                string_to_hex(actual.payload).c_str());
        }
        send(*b->data(), priority);
        b->unref();
    }
};

class TcpIfTest : public ::testing::Test
{
protected:
    static int local_node_count;

    TcpIfTest()
    {
        device_.register_port(&listenPort_);
    }

    ~TcpIfTest()
    {
        device_.unregister_port(&listenPort_);
        wait();
    }

    void wait()
    {
        wait_for_main_executor();
    }

    /// Instructs the mock device to save the next sent packet.
    void capture_next_packet()
    {
        lastPacket_.clear();
        EXPECT_CALL(listenPort_, send(_, _)).WillOnce(SaveArg<0>(&lastPacket_));
    }

    /// Instructs the mock device to ignore all sent packets.
    void ignore_all_packets()
    {
        EXPECT_CALL(listenPort_, send(_, _)).Times(AtLeast(0));
    }

    /// Instructs the mock device to ignore all sent packets.
    void capture_all_packets()
    {
        EXPECT_CALL(listenPort_, send(_, _))
            .WillRepeatedly(WithArg<0>(
                Invoke([this](const string &d) { allPackets_.push_back(d); })));
    }

    /// Creates a GenMessage, renders it to binary format, and injects it as if
    /// it came as an input from the network.
    /// @param args same arguments as GenMessage::reset(). Example
    /// reset(Defs::MTI_EVENT_REPORT, TEST_NODE_ID,
    ///       eventid_to_buffer(UINT64_C(0x0102030405060708)));
    template <typename... Args> void generate_input_message(Args &&... args)
    {
        GenMessage msg;
        msg.reset(std::forward<Args>(args)...);
        auto *b = device_.alloc();
        TcpDefs::render_tcp_message(msg, INPUT_GW_NODE_ID, 0x42, b->data());
        b->data()->skipMember_ = &listenPort_;
        device_.send(b);
        wait();
    }

    /// Creates a GenMessage and sends it out via the IfTcp.
    /// @param args same arguments as GenMessage::reset(). Example
    /// reset(Defs::MTI_EVENT_REPORT, TEST_NODE_ID,
    ///       eventid_to_buffer(UINT64_C(0x0102030405060708)));
    template <typename... Args> void generate_output_message(Args &&... args)
    {
        generate_output_message(&ifTcp_, std::forward<Args>(args)...);
    }

    /// Creates a GenMessage and sends it out via the IfTcp.
    /// @param args same arguments as GenMessage::reset(). Example
    /// reset(Defs::MTI_EVENT_REPORT, TEST_NODE_ID,
    ///       eventid_to_buffer(UINT64_C(0x0102030405060708)));
    template <typename... Args>
    void generate_output_message(IfTcp *iface, Args &&... args)
    {
        auto *f = iface->global_message_write_flow();
        auto *b = f->alloc();
        b->data()->reset(std::forward<Args>(args)...);
        f->send(b);
        wait();
    }

#define expect_packet_is(pkt, x...)                                            \
    do                                                                         \
    {                                                                          \
        GenMessage actual;                                                     \
        actual.clear();                                                        \
        EXPECT_TRUE(TcpDefs::parse_tcp_message(pkt, &actual));                 \
        GenMessage expected;                                                   \
        expected.reset(x);                                                     \
        EXPECT_EQ(expected.mti, actual.mti);                                   \
        EXPECT_EQ(expected.src, actual.src);                                   \
        EXPECT_EQ(expected.dst.id, actual.dst.id);                             \
        EXPECT_EQ(expected.payload, actual.payload);                           \
    } while (0)

    /// Helper function to create a new virtual node.
    /// @param p is the pointer where the node objects' ownership will be
    /// transferred.
    /// @param node_id is the id of the new node.
    /// @param iface the TCP interface to which to bind the node. nullptr = the
    /// default interface of the test.
    /// @return the new node pointer.
    template <class NodeType = DefaultNode>
    Node *create_new_node(
        std::unique_ptr<NodeType> *p, NodeID node_id, IfTcp *iface = nullptr)
    {
        if (!iface)
        {
            iface = &ifTcp_;
        }
        capture_next_packet();
        p->reset(new DefaultNode(iface, node_id));
        wait();
        Mock::VerifyAndClear(&listenPort_);
        expect_packet_is(lastPacket_, Defs::MTI_INITIALIZATION_COMPLETE,
            node_id, node_id_to_buffer(node_id));
        return p->get();
    }

    BarrierNotifiable *get_notifiable()
    {
        bn_.reset(&n_);
        return &bn_;
    }

    void wait_for_notification()
    {
        n_.wait_for_notification();
    }

    SyncNotifiable n_;
    BarrierNotifiable bn_;
    HubFlow device_{&g_service};
    ::testing::StrictMock<MockHubPort> listenPort_;
    /// Stores the data from capture_next_packet.
    string lastPacket_;
    /// Stores data from capture all packets.
    vector<string> allPackets_;

    static constexpr NodeID TEST_NODE_ID = 0x101212231225ULL;
    static constexpr NodeID REMOTE_NODE_ID = 0x050902030405ULL;
    static constexpr NodeID INPUT_GW_NODE_ID = 0x101112131415ULL;
    static constexpr NodeID GW_NODE_ID = 0x101112131415ULL;

    IfTcp ifTcp_{GW_NODE_ID, &device_, local_node_count};
};

constexpr NodeID TcpIfTest::TEST_NODE_ID;
constexpr NodeID TcpIfTest::REMOTE_NODE_ID;
constexpr NodeID TcpIfTest::INPUT_GW_NODE_ID;
constexpr NodeID TcpIfTest::GW_NODE_ID;

int TcpIfTest::local_node_count = 9;

class TcpNodeTest : public TcpIfTest
{
protected:
    TcpNodeTest()
    {
        node_ = create_new_node(&ownedNode_, TEST_NODE_ID);
    }

    std::unique_ptr<DefaultNode> ownedNode_;
    Node *node_;
};

class MultiTcpIfTest : public TcpIfTest
{
protected:
    MultiTcpIfTest()
    {
#ifdef __linux__
        // We expect write failures to occur but we want to handle them where
        // the error occurs rather than in a SIGPIPE handler.
        signal(SIGPIPE, SIG_IGN);
#endif
    }

    ~MultiTcpIfTest()
    {
        wait();
    }

    friend struct ClientIf;

    struct ClientIf
    {
        ClientIf(MultiTcpIfTest *parent, NodeID node_id)
            : parent_(parent)
            , nodeId_(node_id)
        {
            ERRNOCHECK("socketpair", socketpair(AF_UNIX, SOCK_STREAM, 0, fds_));
            ERRNOCHECK("fcntl", fcntl(fds_[0], F_SETFL,
                                    fcntl(fds_[0], F_GETFL, 0) | O_NONBLOCK));
            ERRNOCHECK("fcntl", fcntl(fds_[1], F_SETFL,
                                    fcntl(fds_[1], F_GETFL, 0) | O_NONBLOCK));
            ifTcp_.add_network_fd(fds_[1]);
            parent->ifTcp_.add_network_fd(fds_[0]);
        }
        int fds_[2];
        MultiTcpIfTest *parent_;
        NodeID nodeId_;
        HubFlow device_{&g_service};
        IfTcp ifTcp_{nodeId_, &device_, parent_->local_node_count};
    };

    void add_client(NodeID node_id)
    {
        clients_.emplace_back(new ClientIf(this, node_id));
    }

    vector<std::unique_ptr<ClientIf>> clients_;
};

} // namespace openlcb
