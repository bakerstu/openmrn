#include "utils/test_main.hxx"
#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/HubDevice.hxx"

struct TestData
{
    int from;
    int payload;
};

typedef HubContainer<StructContainer<TestData>> TestHubData;
typedef FlowInterface<Buffer<TestHubData>> TestHubPortInterface;
typedef StateFlow<Buffer<TestHubData>, QList<1>> TestHubPort;
typedef GenericHubFlow<TestHubData> TestHubFlow;
typedef HubDeviceSelect<TestHubFlow> TestHubDeviceAsync;
