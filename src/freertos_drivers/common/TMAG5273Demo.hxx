#include "os/OS.hxx"
#include "freertos_drivers/common/TMAG5273.hxx"


class MagSensorTest : public OSThread {
public:
    MagSensorTest()
    {
        start("mag_sensor", 0, 2048);
    }

    void* entry() override {
        sen_.init("/dev/i2c0");
        while(true) {
            uint8_t rc[2];
            sen_.register_read(TMAG5273::MANUFACTURER_ID_LSB, rc, 2);
            LOG(ALWAYS, "hello %02x %02x", rc[0], rc[1]);
            microsleep(500000);
        }
    }

    TMAG5273 sen_;
};

