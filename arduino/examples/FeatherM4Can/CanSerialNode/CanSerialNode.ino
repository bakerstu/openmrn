// OpenMRN demo application for Adafruit SAMD Feather M4 CAN board.
//
// This application acts as a USB-CAN interface while also appearing on the
// OpenLCB bus as a regular node (which does not do anything special really).
//
//-----------------------------------------------------------------
// Board Manager: Adafruit SAMD board
// Board Manager URL: https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

#ifndef ARDUINO_FEATHER_M4_CAN
  #error "This sketch should be compiled for Arduino Feather M4 CAN (SAME51)"
#endif

// These are required to get the right CAN setup working.
#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (1728)
#include <ACANFD_FeatherM4CAN.h>

//-----------------------------------------------------------------

#include <OpenMRNLite.h>

/// Which serial port to use.
#define SERIAL_PORT Serial

/// Specify how fast the serial port should be going. In order not to lose
/// packets on a fully loaded CAN-bus, this has to be at least 460800, but the
/// default is set to 115200 for better compatibility.
#define SERIAL_BAUD_RATE 115200

/// This is the OpenLCB Node ID. It must be coming from the Node ID range
/// assigned to the developer (get a range assigned to you via openlcb.org).
static constexpr uint64_t NODE_ID = UINT64_C(0x050101011824);

FeatherM4Can CanDriver;
OpenMRN openmrn(NODE_ID);

OVERRIDE_CONST_TRUE(gc_generate_newlines);

namespace openlcb {
/// These definitions tell how the Node will appear on the OpenLCB bus for a
/// network browser.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "OpenMRN",
    "CAN-Serial-Node Adafruit Feather-M4-CAN",
    "Feather M4 CAN",
    "1.00"
};

extern const char* const SNIP_DYNAMIC_FILENAME = nullptr;

} // namespace openlcb

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  SERIAL_PORT.begin(SERIAL_BAUD_RATE);
  Serial.println ("OpenMRNLite demo app for Feather M4 CAN") ;
  openmrn.add_gridconnect_port(&SERIAL_PORT);

  HASSERT(CanDriver.begin());
  openmrn.add_can_port(&CanDriver);
  openmrn.begin();
}
  

//-----------------------------------------------------------------

void loop () {
  openmrn.loop();
}

//-----------------------------------------------------------------
