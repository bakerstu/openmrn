
#include "utils/HubDeviceSelect.hxx"

template<>
void HubDeviceSelect<HubFlow>::resize_target(HubFlow::buffer_type* b) {
    b->data()->resize(64);
}
template<>
void HubDeviceSelect<HubFlow>::check_target_size(HubFlow::buffer_type* b, int remaining) {
    HASSERT(remaining >= 0);
    HASSERT(remaining <= 64);
    b->data()->resize(64-remaining);
}
