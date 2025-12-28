#ifndef _OPENLCB_FILTERINGCANHUBFLOW_HXX_
#define _OPENLCB_FILTERINGCANHUBFLOW_HXX_

#include "utils/Hub.hxx"
#include "openlcb/CanFilter.hxx"

namespace openlcb
{

/// A CAN Hub Flow that uses CanFilter to route messages.
class FilteringCanHubFlow : public CanHubFlow
{
public:
    FilteringCanHubFlow(Service *service);

    /// Sets whether filtering is enabled.
    /// @param is_filtering true to enable filtering, false for legacy behavior.
    void set_filtering(bool is_filtering)
    {
        isFiltering_ = is_filtering;
    }

    Action entry() override;
    Action iterate() override;

    void unregister_port(CanHubFlow::port_type* port) override;

private:
    CanFilter filter_;
    bool isFiltering_;
};

} // namespace openlcb

#endif // _OPENLCB_FILTERINGCANHUBFLOW_HXX_
