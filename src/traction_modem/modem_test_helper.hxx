#include "traction_modem/ModemTrainHwInterface.hxx"
#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"

namespace traction_modem
{

//
// Mock objects.
//
class MockRxFlow : public RxInterface
{
public:
    MOCK_METHOD1(start, void(int));
    MOCK_METHOD3(register_handler,
        void(PacketFlowInterface*, Message::id_type, Message::id_type));
    MOCK_METHOD3(unregister_handler,
        void(PacketFlowInterface*, Message::id_type, Message::id_type));
    MOCK_METHOD1(unregister_handler_all, void(PacketFlowInterface*));
};

class MyMockRxFlow : public MockRxFlow
{
public:
    void register_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.register_handler(interface, id, mask);
        MockRxFlow::register_handler(interface, id, mask);
    }
 
    void unregister_handler_all(PacketFlowInterface *interface) override
    {
        dispatcher_.unregister_handler_all(interface);
        MockRxFlow::unregister_handler_all(interface);
    }

    DispatchFlow<Buffer<Message>, 2> dispatcher_{&g_service};
};

class MockTxFlow : public TxInterface
{
public:
    MOCK_METHOD1(start, void(int));
    MOCK_METHOD1(send_packet, void(Defs::Payload));
};

class MockTrainHwInterface : public ModemTrainHwInterface
{
public:
    MOCK_METHOD2(output_state, void(uint16_t, uint16_t));
    MOCK_METHOD1(output_restart, void(uint16_t));
};

} // namespace traction_modem