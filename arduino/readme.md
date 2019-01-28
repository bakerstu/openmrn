# OpenMRN-lite Arduino Library
The OpenMRN-lite Arduino Library is a packaged version of the OpenMRN code that
has been designed to run in the Arduino environment. The primary features of the
OpenMRN library are available through this library.

## Supported platforms
At this time the only supported platform for execution is the ESP32 using the
[arduino-esp32](https://github.com/espressif/arduino-esp32) core as the
underlying stack.

Additional platforms could be added in the future.

## Preparing the library for use in Arduino IDE or PlatformIO IDE
The OpenMRN-lite library is not currently delivered as a standalone released
library and must be generated using the libify.sh script. Executing this script
creates a library directory that is usable in the various IDEs that support
Arduino development.

### Executing libify.sh to create the OpenMRN-lite library
The libify.sh script requires a bash like environment for execution, on Windows
the GitBash commandline will work. On Linux/MacOS the native bash shell will
work.
    sh libify.sh {path to OpenMRN-lite creation directory} {path to OpenMRN}

#### Arduino IDE library generation
On Windows the Arduino IDE stores the libraries under
"Documents\Arduino\libraries", this can be accessed via the GitBash commandline
as:
```bash
    sh arduino/libify.sh "$USERPROFILE/Documents/Arduino/libraries/OpenMRN-lite" .
```
when executed from the OpenMRN repository root folder.

#### PlatformIO IDE library generation
For PlatformIO IDE it would be recommended to put this into the project lib
folder instead. By default the PlatformIO build process will inspect the lib
folder for project specific libraries and will automatically include them in
the compilation.
```bash
    sh arduino/libify.sh "/path/to/project/lib/OpenMRN-lite" .
```
when executed from the OpenMRN repository root folder.

# ESP32 supported hardware
At this time the ESP32 supports both WiFi and hardware CAN adapters. All
variants of the ESP32 boards are supported. For the hardware CAN support they
will require two additional GPIO pins. WiFi does not require any additional
GPIO pins.

## ESP32 WiFi support
The ESP32 WiFi stack has issues at times but is generally stable. If you
observe failures in connecting to WiFi add the following compiler option
to turn on additional diagnostic output from the
[WiFi](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)
library:
    `-DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_DEBUG`
This will give additional output on the Serial console which can help
to resolve this connection issue. The following is an example of the output
which could be observed:
```
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 5 - STA_DISCONNECTED
    [W][WiFiGeneric.cpp:357] _eventCallback(): Reason: 2 - AUTH_EXPIRE
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 0 - WIFI_READY
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 2 - STA_START
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 2 - STA_START
    [D][WiFiGeneric.cpp:342] _eventCallback(): Event: 5 - STA_DISCONNECTED
```
If you observe this output this generally means there was a timeout condition
where the ESP32 did not receive a response from the access point. This timeout
is unfortuntely not configurable at this time. It is not known if this is due
to a poor signal quality or an underlying bug in the ESP-IDF WiFi driver. The
solution for this appears to be power down the ESP32 and restart the AP. The
ESP32 should successfully connect. Additional options to try if this does not
resolve the connection issues:
1. Before connecting to the AP add these lines:
```C++
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(true);
```
    The above two lines should be set before the call to WiFi.begin();
2. If the ESP32 board supports an external WiFi antenna use one, this will
provide a higher signal strength which should allow a more successful
connection.
3. Clear the persistent WiFi connection details from NVS:
```C
        #include <nvs_flash.h>
        nvs_flash_init();
```
    This should be considered a last resort option as it will erase any
    data in the NVS partition. There is no recovery of data from NVS after
    executing the above function. This can also be achieved by using a flash
    erase tool (esptool.py erase_flash ...) and reflashing the ESP32.

## ESP32 Hardware CAN support
The ESP32 has a built in CAN Controller but lacks a CAN Transceiver. There are
two types of transcievers that are recommended by Espressif:
1. SN65HVD23x, also known as VP230 or similar. This is a 3.3V transceiver that
typically is available as a breakout board with on board termination resistor.
2. MCP2551. This is a 5V transceiver that is available as a DIP-8, SMD or as a
breakout board. This does not include a termination resistor and for this
reason is preferred over the SN65HVD23x boards.

Connecting the CAN transceiver to the ESP32 requires four pins:
1. 3v3 / 5v to the VCC/VDD pin on the transceiver. Make sure to use the right
voltage for the transceiver.
2. GND to the GND pin on the transceiver.
3. GPIO for RX to the RX pin on the transceiver. This can be any unused GPIO
pin.
4. GPIO for TX to the TX pin on the transceiver. This pin must be usable as an
output pin, GPIO 34-39 on the ESP32 are input only.

## Powering the ESP32
It is not recommended to power the ESP32 from the CAN PWR_POS (7) / PWR_NEG (8)
pins due to the ESP32 having the potential to draw in excess of the 500mA limit
defined in the CAN physical specification (section 7). The WiFi RF system will
consume up to 250mA during RX/TX and each GPIO that is set to HIGH will consume
approximately 12mA. When all 20 usable GPIO pins are HIGH and WiFi is active
this will be approximately 490mA if not more.

Therefore, it is recommended to provide an external power supply to the ESP32
of at least 5V DC 750mA. If multiple ESP32 devices will share this power supply
a power supply with higher amperage is recommended.
