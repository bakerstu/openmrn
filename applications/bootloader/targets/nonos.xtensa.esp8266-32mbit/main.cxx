/** \copyright
 * Copyright (c) 2016, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file BootloaderHal.cxx
 *
 * ESP8266-specific implementation of the HAL (Hardware Abstraction Layer) used
 * by the OpenLCB bootloader.
 *
 * @author Balazs Racz
 * @date 19 Jun 2016
 */

#include <string.h>
#include <stdint.h>

#include <c_types.h>
#include "freertos/bootloader_hal.h"

extern "C" {
#include "eboot_command.h"
#include "flash.h"
}

#include "freertos_drivers/esp8266/Esp8266Gpio.hxx"
#include "utils/ESPWifiClient.hxx"
#include "utils/Crc.hxx"
#include "utils/GpioInitializer.hxx"
#include "utils/blinker.h"
#include "nmranet_config.h"
#include "nmranet/Defs.hxx"
#include "nmranet/If.hxx"
#include "nmranet/BootloaderPort.hxx"

extern "C" {

void reboot_now();

struct HW
{
    /* original / standard definitions.
        GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 4);
        GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 5);

        GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 14);
        GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 12);

        // forward: A=HI B=LO

        GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, 13);
        GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 15);

        GPIO_PIN(F1, GpioOutputSafeLow, 2);

        //typedef DummyPin F1_Pin;

    */

    GPIO_PIN(MOT_A_HI, GpioOutputSafeLow, 4);
    GPIO_PIN(MOT_A_LO, GpioOutputSafeLow, 5);

    GPIO_PIN(MOT_B_HI, GpioOutputSafeLow, 14);
    GPIO_PIN(MOT_B_LO, GpioOutputSafeLow, 12);

    static constexpr bool invertLow = false;

    // forward: A=HI B=LO

    // typedef BLINKER_Pin LIGHT_FRONT_Pin;
    GPIO_PIN(LIGHT_FRONT, GpioOutputSafeLow, 13);
    GPIO_PIN(LIGHT_BACK, GpioOutputSafeLow, 15);

    GPIO_PIN(F1, GpioOutputSafeHigh, 2);
    // typedef DummyPin F1_Pin;

    typedef GpioInitializer<        //
        MOT_A_HI_Pin, MOT_A_LO_Pin, //
        MOT_B_HI_Pin, MOT_B_LO_Pin, //
        LIGHT_FRONT_Pin, LIGHT_BACK_Pin, F1_Pin> GpioInit;
};

void bootloader_hw_set_to_safe(void)
{
    HW::GpioInit::hw_set_to_safe();
}

void bootloader_hw_init()
{
}

void bootloader_led(enum BootloaderLed id, bool value)
{
    switch (id)
    {
        case LED_ACTIVE:
            HW::F1_Pin::set(value);
            return;
        case LED_WRITING:
            resetblink(value ? 1 : 0);
            return;
        case LED_CSUM_ERROR:
            printf("Checksum: %s\n", value ? "error" : "ok");
            return;
        case LED_REQUEST:
            printf("Manual request: %s\n", value ? "yes" : "no");
            return;
        default:
            ; /* ignore */
    }
}

bool request_bootloader()
{
    extern uint32_t __bootloader_magic_ptr;
    if (__bootloader_magic_ptr == REQUEST_BOOTLOADER)
    {
        __bootloader_magic_ptr = 0;
        return true;
    }
    // there is no way to request bootloader mode by pressing a button for now.
    return false;
}

/// Pointer in the memory where the SPI flash is mapped.
#define FLASH_MAP_ADDRESS (0x40200000)
/// Offset in the SPI flash where the user code block is located. This has to
/// be larger than the size of the bootloader binary.
#define USER_CODE_OFFSET (3 * 128 * 1024)
/// Size of the memory mapped region of flash.
#define USER_CODE_END (1024 * 1024)
/// Offset inside the user code block where the application starts. Used to
/// calculate where to load the application from when booting.
#define HEADER_LEN (4096)

void get_flash_boundaries(const void **flash_min, const void **flash_max,
    const struct app_header **app_header)
{
    *flash_min = (void *)(FLASH_MAP_ADDRESS + USER_CODE_OFFSET);
    *flash_max = (void *)(FLASH_MAP_ADDRESS + USER_CODE_END);
    *app_header = (struct app_header *)(FLASH_MAP_ADDRESS + USER_CODE_OFFSET);
}

void checksum_data(const void *data, uint32_t size, uint32_t *checksum)
{
    memset(checksum, 0, 16);
    crc3_crc16_ibm(data, size, (uint16_t*) checksum);
}

extern const uint16_t DEFAULT_ALIAS;

uint16_t nmranet_alias()
{
    /// TODO(balazs.racz):  fix this
    return DEFAULT_ALIAS;
}

extern const nmranet::NodeID NODE_ID;

uint64_t nmranet_nodeid()
{
    /// TODO(balazs.racz):  read some form of EEPROM instead.
    return NODE_ID;
}

Executor<1> g_executor{NO_THREAD()};
Service g_service(&g_executor);
CanHubFlow g_can_hub(&g_service);
nmranet::BootloaderPort g_bootloader_port(&g_service);
bool bootloader_reset_request = false;

bool read_can_frame(struct can_frame *frame)
{
    return g_bootloader_port.read_can_frame(frame);
}

bool try_send_can_frame(const struct can_frame &frame)
{
    auto *b = g_can_hub.alloc();
    *b->data()->mutable_frame() = frame;
    b->data()->skipMember_ = &g_bootloader_port;
    g_can_hub.send(b);
    return true;
}

void bootloader_reboot(void)
{
    bootloader_reset_request = true;
}

void application_entry(void)
{
    printf("Starting application.\n");
    struct eboot_command cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.action = ACTION_LOAD_APP;
    cmd.args[0] = USER_CODE_OFFSET + HEADER_LEN - APP_START_OFFSET;
    eboot_command_write(&cmd);
    reboot_now();
}

void erase_flash_page(const void *address)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);
    uint32_t a = (uint32_t) address;
    a -= FLASH_MAP_ADDRESS;
    SPIEraseSector(a/FLASH_SECTOR_SIZE);
    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);

    uint32_t a = (uint32_t) address;
    a -= FLASH_MAP_ADDRESS;
    SPIWrite(a, (void*)data, size_bytes);

    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void get_flash_page_info(
    const void *address, const void **page_start, uint32_t *page_length_bytes)
{
    uint32_t value = (uint32_t)address;
    value &= (FLASH_SECTOR_SIZE - 1);
    *page_start = (const void *)value;
    *page_length_bytes = FLASH_SECTOR_SIZE;
}

extern char WIFI_SSID[];
extern char WIFI_PASS[];
extern char WIFI_HUB_HOSTNAME[];
extern int WIFI_HUB_PORT;

bool bootloader_init();
bool bootloader_loop();

} // extern "C"

class BootloaderFlow : public StateFlowBase {
public:
    BootloaderFlow() : StateFlowBase(&g_service) {}

    void start() {
        start_flow(STATE(loop));
    }

private:
    Action loop() {
        bootloader_loop();
        if (bootloader_reset_request) {
            return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(reset));
        }
        return yield_and_call(STATE(loop));
    }

    Action reset() {
        printf("Rebooting bootloader.\n");
        eboot_command_clear();
        reboot_now();
        return exit();
    }

    StateFlowTimer timer_{this};
};

int appl_main(int argc, char**argv) {
    bootloader_hw_set_to_safe();
    bootloader_hw_init();
    bootloader_init();
    printf("bootloader init done.\n");
    g_can_hub.register_port(&g_bootloader_port);
    resetblink(1);
    new ESPWifiClient(WIFI_SSID, WIFI_PASS, &g_can_hub, WIFI_HUB_HOSTNAME,
                      WIFI_HUB_PORT, 1200, []()
        {
            resetblink(0);
            // This will actually return due to the event-driven OS
            // implementation of the stack.
            g_executor.thread_body();
            (new BootloaderFlow())->start();
        });
    return 0;
}

