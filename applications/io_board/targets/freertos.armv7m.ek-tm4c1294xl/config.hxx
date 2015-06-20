#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "nmranet/ConfiguredConsumer.hxx"
#include "nmranet/ConfiguredProducer.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "nmranet/MemoryConfig.hxx"

namespace nmranet
{

/// Defines the identification information for the node. The arguments are:
///
/// - 4 (version info, always 4 by the standard
/// - Manufacturer name
/// - Model name
/// - Hardware version
/// - Software version
///
/// This data will be used for all purposes of the identification:
///
/// - the generated cdi.xml will include this data
/// - the Simple Node Ident Info Protocol will return this data
/// - the ACDI memory space will contain this data.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,               "OpenMRN", "Test IO Board - Tiva Connected Launchpad",
    "ek-tm4c1294xl", "1.01"};

/// Declares a repeated group of a given base group and number of repeats. The
/// ProducerConfig and ConsumerConfig groups represent the configuration layout
/// needed by the ConfiguredProducer and ConfiguredConsumer classes, and come
/// from their respective hxx file.
using AllConsumers = RepeatedGroup<ConsumerConfig, 4>;
using AllProducers = RepeatedGroup<ProducerConfig, 2>;

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
BEGIN_GROUP(
    IoBoardSegment, base, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
EXTEND_GROUP(
    IoBoardSegment, base, consumers, AllConsumers, Name("Output LEDs"));
/// The entries are chained: each entry refers to the previous entry, then
/// declares the name of the current entry, then the type and then optional
/// arguments list.
EXTEND_GROUP(
    IoBoardSegment, consumers, producers, AllProducers, Name("Input buttons"));
END_GROUP(IoBoardSegment, producers);

/// This segment is only needed temporarily until there is program code to set
/// the ACDI user data version byte.
BEGIN_GROUP(VersionSeg, base, Segment(MemoryConfigDefs::SPACE_CONFIG),
    Name("Version information"));
EXTEND_GROUP(VersionSeg, base, acdi_user_version, Uint8ConfigEntry,
    Name("ACDI User Data version"), Description("Set to 2 and do not change."));
END_GROUP(VersionSeg, acdi_user_version);

/// The main structure of the CDI. ConfigDef is the symbol we use in main.cxx
/// to refer to the configuration defined here.
BEGIN_GROUP(ConfigDef, base, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA above.
EXTEND_GROUP(ConfigDef, base, ident, Identification);
/// Adds an <acdi> tag.
EXTEND_GROUP(ConfigDef, ident, acdi, Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
EXTEND_GROUP(ConfigDef, acdi, userinfo, UserInfoSegment);
/// Adds the main configuration segment.
EXTEND_GROUP(ConfigDef, userinfo, seg, IoBoardSegment);
/// Adds the versioning segment.
EXTEND_GROUP(ConfigDef, seg, version, VersionSeg);
/// To end the group we have to declare what was the last entry.
END_GROUP(ConfigDef, version);

} // namespace nmranet

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
