#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "DmxSceneConfig.hxx"

namespace openlcb
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
    4,               "OpenMRN", "Test IO Board - CC3220SF Launchpad",
    "CC3220SF-LAUNCHXL", "1.01"};

#define NUM_OUTPUTS 3
#define NUM_INPUTS 2

/// Declares a repeated group of a given base group and number of repeats. The
/// ProducerConfig and ConsumerConfig groups represent the configuration layout
/// needed by the ConfiguredProducer and ConfiguredConsumer classes, and come
/// from their respective hxx file.
using AllConsumers = RepeatedGroup<ConsumerConfig, NUM_OUTPUTS>;
using AllProducers = RepeatedGroup<ProducerConfig, NUM_INPUTS>;

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update.
static constexpr uint16_t CANONICAL_VERSION = 0x124f;


CDI_GROUP(LegacyConfig);
/// Each entry declares the name of the current entry, then the type and then
/// optional arguments list.
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_ENTRY(consumers, AllConsumers, Name("Output LEDs"));
CDI_GROUP_ENTRY(producers, AllProducers, Name("Input buttons"));
CDI_GROUP_END();


/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(IoBoardSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
CDI_GROUP_ENTRY(legacy, LegacyConfig, FixedSize(64));
CDI_GROUP_ENTRY(scenes, dmx::AllSceneConfig, Name("Scenes"), Description("Scenes allow loading specific visual configurations by LCC events."), RepName("Scene"));
CDI_GROUP_END();

CDI_GROUP(DmxSegment, Segment(72), Offset(0), Name("Direct DMX edit"),
    Description("Allows directly setting the DMX channel data."));
CDI_GROUP_ENTRY(ch1, Uint8ConfigEntry, Name("Channel 1"));
CDI_GROUP_ENTRY(ch2, Uint8ConfigEntry, Name("Channel 2"));
CDI_GROUP_ENTRY(ch3, Uint8ConfigEntry, Name("Channel 3"));
CDI_GROUP_ENTRY(ch4, Uint8ConfigEntry, Name("Channel 4"));
CDI_GROUP_ENTRY(ch5, Uint8ConfigEntry, Name("Channel 5"));
CDI_GROUP_ENTRY(ch6, Uint8ConfigEntry, Name("Channel 6"));
CDI_GROUP_ENTRY(ch7, Uint8ConfigEntry, Name("Channel 7"));
CDI_GROUP_ENTRY(ch8, Uint8ConfigEntry, Name("Channel 8"));
CDI_GROUP_ENTRY(ch9, Uint8ConfigEntry, Name("Channel 9"));
CDI_GROUP_ENTRY(cha, Uint8ConfigEntry, Name("Channel 10"));
CDI_GROUP_ENTRY(chb, Uint8ConfigEntry, Name("Channel 11"));
CDI_GROUP_ENTRY(chc, Uint8ConfigEntry, Name("Channel 12"));
CDI_GROUP_ENTRY(chd, Uint8ConfigEntry, Name("Channel 13"));
CDI_GROUP_ENTRY(che, Uint8ConfigEntry, Name("Channel 14"));
CDI_GROUP_END();

/// The main structure of the CDI. ConfigDef is the symbol we use in main.cxx
/// to refer to the configuration defined here.
CDI_GROUP(ConfigDef, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA above.
CDI_GROUP_ENTRY(ident, Identification);
/// Adds an <acdi> tag.
CDI_GROUP_ENTRY(acdi, Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
CDI_GROUP_ENTRY(userinfo, UserInfoSegment);
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, IoBoardSegment);
CDI_GROUP_ENTRY(dmxseg, DmxSegment);
CDI_GROUP_END();

} // namespace openlcb

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
