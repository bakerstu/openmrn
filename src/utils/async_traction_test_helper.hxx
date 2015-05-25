#ifndef _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
#define _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"

#include "nmranet/TractionDefs.hxx"
#include "nmranet/TractionTrain.hxx"
#include "utils/MockTrain.hxx"

namespace nmranet
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


} // namespace nmranet

#endif // _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
