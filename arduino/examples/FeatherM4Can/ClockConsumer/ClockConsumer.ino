// OpenMRN demo application for Adafruit SAMD Feather M4 CAN board.
//
// This application acts as a CAN-only OpenLCB node which consumes the
// default fast clock, and displays the model time in minutes by printing it
// to the Arduino monitor port every model minute.
//
//-----------------------------------------------------------------
// Board Manager: Adafruit SAMD board
// Board Manager URL:
// https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

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

namespace openlcb
{
/// These definitions tell how the Node will appear on the OpenLCB bus for a
/// network browser.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {4, "OpenMRN",
    "CAN-Serial-Node Adafruit Feather-M4-CAN", "Feather M4 CAN", "1.00"};

extern const char *const SNIP_DYNAMIC_FILENAME = nullptr;

} // namespace openlcb

// This hack is needed because some arduino variants define a macro called
// "abs" in their toplevel include header.
#ifdef abs
#undef abs
#endif

#include "openlcb/BroadcastTimeAlarm.hxx"
#include "openlcb/BroadcastTimeClient.hxx"

// Which clock to display.
static constexpr auto CLOCK_ID =
    openlcb::BroadcastTimeDefs::DEFAULT_FAST_CLOCK_ID;

// Whether to allow remote control of the fast clock.
static constexpr auto ALLOW_CONTROL = true;

openlcb::BroadcastTimeClient time_client {
    openmrn.stack()->node(), CLOCK_ID, ALLOW_CONTROL};

/// Called when the date is changed.
void update_date(BarrierNotifiable *done)
{
    int y = time_client.year();
    int mon, day;
    time_client.date(&mon, &day);
    auto date = openlcb::BroadcastTimeDefs::date_to_string(y, mon, day);
    SERIAL_PORT.print("Date: ");
    SERIAL_PORT.println(date.c_str());
    if (done)
        done->notify();
}

/// Called when the minute on the clock is changed.
void update_minute(BarrierNotifiable *done)
{
    struct tm time;
    time_client.gmtime_r(&time);
    auto str =
        openlcb::BroadcastTimeDefs::time_to_string(time.tm_hour, time.tm_min);
    SERIAL_PORT.println(str.c_str());
    if (done)
        done->notify();
}

/// Called when the current known time makes a jump.
void update(time_t old, time_t current)
{
    struct tm time;
    time_client.gmtime_r(&time);
    auto time_s =
        openlcb::BroadcastTimeDefs::time_to_string(time.tm_hour, time.tm_min);
    int y = time_client.year();
    int mon, day;
    time_client.date(&mon, &day);
    auto date_s = openlcb::BroadcastTimeDefs::date_to_string(y, mon, day);
    const char *is_run = time_client.is_running() ? "running" : "stopped";
    int rate_q = time_client.get_rate_quarters();
    string rate_s = openlcb::BroadcastTimeDefs::rate_quarters_to_string(rate_q);
    SERIAL_PORT.printf("Clock %s, rate %s, %s %s\n", is_run, rate_s.c_str(),
        date_s.c_str(), time_s.c_str());
}

openlcb::BroadcastTimeAlarmMinute alarm_minute {
    openmrn.stack()->node(), &time_client, &update_minute};

openlcb::BroadcastTimeAlarmDate alarm_date {
    openmrn.stack()->node(), &time_client, &update_date};

/// Called periodically in the loop. Checks whether we got connected to a
/// server.
void check_server()
{
    static bool server_shadow = false;
    bool has_srv = time_client.is_server_detected();
    if (has_srv != server_shadow)
    {
        SERIAL_PORT.println(
            has_srv ? "Found time source." : "Time source lost.");
    }
    server_shadow = has_srv;
    static bool warning = false;
    if (!has_srv && !warning && millis() > 5000)
    {
        SERIAL_PORT.println("No time source. Make sure that there is a clock "
                            "generator for the given clock ID.");
        warning = true;
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    SERIAL_PORT.begin(SERIAL_BAUD_RATE);
    // Waits for the host to connect via USB because we are outputting time
    // information to the host.
    while (!SERIAL_PORT)
    {
        delay(50);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    SERIAL_PORT.println("OpenMRNLite demo app for Feather M4 CAN");

    HASSERT(CanDriver.begin());
    openmrn.add_can_port(&CanDriver);
    openmrn.begin();
    time_client.update_subscribe_add(&update);
}

//-----------------------------------------------------------------

void loop()
{
    openmrn.loop();
    check_server();
}

//-----------------------------------------------------------------
