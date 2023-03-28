#include "os/OS.hxx"
#include "freertos_drivers/common/TMAG5273.hxx"

#include <endian.h>


class MagSensorTest : public OSThread {
public:
    MagSensorTest()
    {
        start("mag_sensor", 0, 2048);
    }

    void* entry() override {
        sen_.init("/dev/i2c0");
        // Enables all three mag channels, and sleeps 10 msec between
        // conversions.
        sen_.register_write(TMAG5273::SENSOR_CONFIG_1, 0x72);
        // Operating mode = sleep-and-wake.
        sen_.register_write(TMAG5273::DEVICE_CONFIG_2, 0x3);
        // Clear error bits
        sen_.register_write(TMAG5273::DEVICE_STATUS, 0xF);
        // Int config to disable the nINT pin
        sen_.register_write(TMAG5273::INT_CONFIG_1, 0x1);
        // Clears POR bit
        sen_.register_write(TMAG5273::CONV_STATUS, 0x0);
        // Sets lower gain / higher range
        sen_.register_write(TMAG5273::SENSOR_CONFIG_2, 0b11);
        while(true) {
            uint8_t rc[2];
            sen_.register_read(TMAG5273::MANUFACTURER_ID_LSB, rc, 2);
            uint8_t status = 0;
            sen_.register_read(TMAG5273::DEVICE_STATUS, &status, 1);
            uint8_t conv_status = 0;
            sen_.register_read(TMAG5273::CONV_STATUS, &conv_status, 1);
            LOG(ALWAYS, "hello %02x %02x status=0x%02x convst=0x%02x", rc[0],
                rc[1], status, conv_status);
            int16_t results[3];
            sen_.register_read(TMAG5273::X_MSB_RESULT, (uint8_t*)results, 6);
            results[0] = be16toh(results[0]);
            results[1] = be16toh(results[1]);
            results[2] = be16toh(results[2]);
            LOG(ALWAYS, "x=%6d y=%6d z=%6d", results[0], results[1], results[2]);
            // Clear error bits and trigger a new conversion
            sen_.register_write(TMAG5273::DEVICE_STATUS | 0x80, 0xF);
            microsleep(200000);
        }
    }

    TMAG5273 sen_;
};

