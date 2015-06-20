#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "nmranet/ConfiguredConsumer.hxx"
#include "nmranet/ConfiguredProducer.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "nmranet/MemoryConfig.hxx"

namespace nmranet
{

extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,               "OpenMRN", "Test IO Board - Tiva Launchpad 123",
    "ek-tm4c123gxl", "1.01"};

using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;
using AllProducers = RepeatedGroup<ProducerConfig, 2>;

BEGIN_GROUP(
    IoBoardSegment, base, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
EXTEND_GROUP(
    IoBoardSegment, base, consumers, AllConsumers, Name("Output LEDs"));
EXTEND_GROUP(
    IoBoardSegment, consumers, producers, AllProducers, Name("Input buttons"));
END_GROUP(IoBoardSegment, producers);

BEGIN_GROUP(VersionSeg, base, Segment(MemoryConfigDefs::SPACE_CONFIG),
    Name("Version information"));
EXTEND_GROUP(VersionSeg, base, acdi_user_version, Uint8ConfigEntry,
    Name("ACDI User Data version"), Description("Set to 2 and do not change."));
END_GROUP(VersionSeg, acdi_user_version);

BEGIN_GROUP(ConfigDef, base, MainCdi());
EXTEND_GROUP(ConfigDef, base, ident, Identification);
EXTEND_GROUP(ConfigDef, ident, acdi, Acdi);
EXTEND_GROUP(ConfigDef, acdi, userinfo, UserInfoSegment);
EXTEND_GROUP(ConfigDef, userinfo, seg, IoBoardSegment);
EXTEND_GROUP(ConfigDef, seg, version, VersionSeg);
END_GROUP(ConfigDef, version);

} // namespace nmranet

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
