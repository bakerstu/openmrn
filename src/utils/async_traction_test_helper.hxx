#ifndef _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
#define _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"

#include "dcc/TrackIf.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TractionTrain.hxx"
#include "utils/MockTrain.hxx"

namespace openlcb
{

/// Test fixture base for traction tests.
class TractionTest : public AsyncNodeTest
{
protected:
    TractionTest()
        : trainService_(ifCan_.get())
    {
    }

    TrainService trainService_;
    StrictMock<MockTrain> m1_, m2_;
};

} // namespace openlcb

namespace dcc
{

class MockTrackIf : public dcc::TrackIf
{
public:
    MOCK_METHOD2(
        packet, void(const vector<uint8_t> &payload, uintptr_t feedback_key));
    void send(Buffer<dcc::Packet> *b, unsigned prio) OVERRIDE
    {
        vector<uint8_t> payload;
        payload.assign(
            b->data()->payload, b->data()->payload + b->data()->dlc - 1);
        this->packet(payload, b->data()->feedback_key);
        b->unref();
    }
};

}

#endif // _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
