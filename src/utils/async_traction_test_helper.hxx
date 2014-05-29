#ifndef _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
#define _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"

#include "nmranet/TractionTrain.hxx"
#include "nmranet/TractionDefs.hxx"

namespace nmranet
{

class MockTrain : public TrainImpl
{
public:
    MOCK_METHOD1(set_speed, void(SpeedType speed));
    MOCK_METHOD0(get_speed, SpeedType());
    MOCK_METHOD0(get_commanded_speed, SpeedType());
    MOCK_METHOD0(get_actual_speed, SpeedType());
    MOCK_METHOD0(set_emergencystop, void());
    MOCK_METHOD2(set_fn, void(uint32_t address, uint16_t value));
    MOCK_METHOD1(get_fn, uint16_t(uint32_t address));
    MOCK_METHOD0(legacy_address, uint32_t());
};

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


} // namespace nmranet

#endif // _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
