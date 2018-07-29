#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "ModbusConfig.hxx"

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
    4,               "OpenMRN", "Test modbus Board - CC3220SF Launchpad",
    "CC3220SF-LAUNCHXL", "1.01"};

#define NUM_OUTPUTS 3
#define NUM_INPUTS 1

/// Declares a repeated group of a given base group and number of repeats. The
/// ProducerConfig and ConsumerConfig groups represent the configuration layout
/// needed by the ConfiguredProducer and ConfiguredConsumer classes, and come
/// from their respective hxx file.
using AllConsumers = RepeatedGroup<ConsumerConfig, NUM_OUTPUTS>;
using AllProducers = RepeatedGroup<ProducerConfig, NUM_INPUTS>;

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update.
static constexpr uint16_t CANONICAL_VERSION = 0x1541;


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
CDI_GROUP_ENTRY(scenes, modbus::AllSceneConfig, Name("Scenes"), Description("Scenes allow sending modbus packets by LCC events."), RepName("Scene"));
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
CDI_GROUP_END();

} // namespace openlcb

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
