#include <Arduino.h>
#include <OpenMRN.h>

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

void setup() {
    Serial.begin(115200L);
    openmrn.add_gridconnect_port(&Serial);
    openmrn.init();
}

void loop() {
    openmrn.loop();
}