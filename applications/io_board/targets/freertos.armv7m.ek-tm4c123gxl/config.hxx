#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "nmranet/ConfiguredConsumer.hxx"
#include "nmranet/ConfiguredProducer.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "nmranet/MemoryConfig.hxx"

namespace nmranet
{

using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;
using AllProducers = RepeatedGroup<ProducerConfig, 2>;

BEGIN_GROUP(
    IoBoardSegment, base, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
EXTEND_GROUP(
    IoBoardSegment, base, consumers, AllConsumers, Name("Output LEDs"));
EXTEND_GROUP(
    IoBoardSegment, consumers, producers, AllProducers, Name("Input buttons"));
END_GROUP(IoBoardSegment, producers);

BEGIN_GROUP(ConfigDef, base, MainCdi());
EXTEND_GROUP(ConfigDef, base, ident, Identification);
EXTEND_GROUP(ConfigDef, ident, acdi, Acdi);
EXTEND_GROUP(ConfigDef, acdi, userinfo, UserInfoSegment);
EXTEND_GROUP(ConfigDef, userinfo, seg, IoBoardSegment);
END_GROUP(ConfigDef, seg);

} // namespace nmranet

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
