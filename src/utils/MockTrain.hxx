#ifndef _UTILS_MOCKTRAIN_HXX_
#define _UTILS_MOCKTRAIN_HXX_

#include "gmock/gmock.h"
#include "nmranet/TractionDefs.hxx"
#include "nmranet/TractionTrain.hxx"

namespace nmranet {

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

}  // namespace nmranet

#endif // _UTILS_MOCKTRAIN_HXX_
