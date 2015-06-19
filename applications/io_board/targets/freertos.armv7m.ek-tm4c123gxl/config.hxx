#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "nmranet/ConfiguredConsumer.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "nmranet/MemoryConfig.hxx"

namespace nmranet
{

using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;

BEGIN_GROUP(ProducerConfig, base);
EXTEND_GROUP(ProducerConfig, base, debounce, Uint8ConfigEntry);
EXTEND_GROUP(ProducerConfig, debounce, event_on, EventConfigEntry);
EXTEND_GROUP(ProducerConfig, event_on, event_off, EventConfigEntry);
END_GROUP(ProducerConfig, event_off);

using AllProducers = RepeatedGroup<ProducerConfig, 2>;

BEGIN_GROUP(IoBoardSegment, base, Segment(MemoryConfigDefs::SPACE_CONFIG),
            Offset(0));
EXTEND_GROUP(IoBoardSegment, base, snip_data, EmptyGroup<128>);
EXTEND_GROUP(IoBoardSegment, snip_data, consumers, AllConsumers);
EXTEND_GROUP(IoBoardSegment, consumers, producers, AllProducers);
END_GROUP(IoBoardSegment, producers);

BEGIN_GROUP(ConfigDef, base, MainCdi());
EXTEND_GROUP(ConfigDef, base, ident, Identification);
EXTEND_GROUP(ConfigDef, ident, seg, IoBoardSegment);
END_GROUP(ConfigDef, seg);

} // namespace nmranet

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
