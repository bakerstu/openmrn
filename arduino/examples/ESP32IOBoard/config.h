#ifndef _ARDUINO_EXAMPLE_ESP32IOBOARD_CONFIG_H_
#define _ARDUINO_EXAMPLE_ESP32IOBOARD_CONFIG_H_

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

#include "freertos_drivers/esp32/Esp32WiFiConfiguration.hxx"

// catch invalid configuration at compile time
#if !defined(USE_TWAI) && !defined(USE_WIFI)
#error "Invalid configuration detected, USE_TWAI or USE_WIFI must be defined."
#endif

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
    4,
    "OpenMRN",
#if defined(USE_WIFI) && !defined(USE_TWAI)
    "Arduino IO Board (WiFi)",
#elif defined(USE_TWAI) && !defined(USE_WIFI)
    "Arduino IO Board (TWAI)",
#elif defined(USE_TWAI) && defined(USE_WIFI)
    "Arduino IO Board (WiFi/TWAI)",
#else
    "Arduino IO Board",
#endif
    ARDUINO_VARIANT,
    "1.00"};

constexpr uint8_t NUM_OUTPUTS = 8;
constexpr uint8_t NUM_INPUTS = 8;

/// Declares a repeated group of a given base group and number of repeats. The
/// ProducerConfig and ConsumerConfig groups represent the configuration layout
/// needed by the ConfiguredProducer and ConfiguredConsumer classes, and come
/// from their respective hxx file.
using AllConsumers = RepeatedGroup<ConsumerConfig, NUM_OUTPUTS>;
using AllProducers = RepeatedGroup<ProducerConfig, NUM_INPUTS>;

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update.
static constexpr uint16_t CANONICAL_VERSION = 0x100a;

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(IoBoardSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
/// Each entry declares the name of the current entry, then the type and then
/// optional arguments list.
CDI_GROUP_ENTRY(internal_config, InternalConfigData);
CDI_GROUP_ENTRY(consumers, AllConsumers, Name("Outputs"), RepName("Output"));
CDI_GROUP_ENTRY(producers, AllProducers, Name("Inputs"), RepName("Input"));
#if defined(USE_WIFI)
CDI_GROUP_ENTRY(wifi, WiFiConfiguration, Name("WiFi Configuration"));
#endif
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
CDI_GROUP_ENTRY(userinfo, UserInfoSegment, Name("User Info"));
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, IoBoardSegment, Name("Settings"));
CDI_GROUP_END();

} // namespace openlcb

#endif // _ARDUINO_EXAMPLE_ESP32IOBOARD_CONFIG_H_
