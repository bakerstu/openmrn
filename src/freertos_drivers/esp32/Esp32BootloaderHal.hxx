/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file Esp32BootloaderHal.hxx
 *
 * ESP32 specific implementation of the HAL (Hardware Abstraction Layer) used
 * by the OpenLCB bootloader.
 * 
 * Additional functions from bootloader_hal.h will be required to be defined by
 * the application code.
 *
 * @author Mike Dunston
 * @date 3 May 2021
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32BOOTLOADERHAL_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32BOOTLOADERHAL_HXX_

#include "sdkconfig.h"

#ifndef BOOTLOADER_LOG_LEVEL
#define BOOTLOADER_LOG_LEVEL VERBOSE
#endif // BOOTLOADER_LOG_LEVEL

#ifndef BOOTLOADER_TWAI_LOG_LEVEL
#define BOOTLOADER_TWAI_LOG_LEVEL VERBOSE
#endif // BOOTLOADER_TWAI_LOG_LEVEL

// Enable streaming support for the bootloader
#define BOOTLOADER_STREAM

#ifndef WRITE_BUFFER_SIZE
// Set the buffer size to half of the sector size to minimize the flash writes.
#define WRITE_BUFFER_SIZE (CONFIG_WL_SECTOR_SIZE / 2)
#endif // WRITE_BUFFER_SIZE

#include <driver/twai.h>
#include <esp_app_format.h>
#include <esp_chip_info.h>
#include <esp_ota_ops.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp32/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp32s2/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s3/rom/rtc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#include <esp32c3/rom/rtc.h>
#else
#error Unknown/Unsupported ESP32 variant.
#endif
#include <stdint.h>

#include "openlcb/Bootloader.hxx"
#include "openlcb/Defs.hxx"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"

/// ESP32 Bootloader internal data.
struct Esp32BootloaderState
{
    /// Chip identifier for the currently running firmware.
    esp_chip_id_t chip_id;

    /// Currently running application header information.
    ///
    /// NOTE: Only the size is populated in this structure, the checksum values are
    /// initialized to zero as they are not used.
    struct app_header app_header;

    /// Node ID to use for the bootloader.
    uint64_t node_id;

    /// Partition where the firmware is currently running from.
    esp_partition_t *current;

    /// Partition where the new firmware should be written to.
    esp_partition_t *target;

    /// OTA handle used to track the firmware update progress.
    esp_ota_handle_t ota_handle;

    /// Internal flag to indicate that we have initialized the TWAI peripheral and
    /// should deinit it before exit.
    bool twai_initialized;

    /// GPIO pin connected to the CAN transceiver TX pin.
    gpio_num_t tx_pin;

    /// GPIO pin connected to the CAN transceiver RX pin.
    gpio_num_t rx_pin;
};

/// Bootloader configuration data.
static Esp32BootloaderState esp_bl_state;

/// Maximum time to wait for a TWAI frame to be received before giving up.
static constexpr BaseType_t MAX_TWAI_WAIT_RX = pdMS_TO_TICKS(250);

/// Maximum time to wait for a TWAI frame to be transmitted before giving up.
static constexpr BaseType_t MAX_TWAI_WAIT_TX = pdMS_TO_TICKS(0);

/// Flag used to indicate that we have been requested to enter the bootloader
/// instead of normal node operations. Note that this value will not be
/// initialized by the system and a check for power on reset will need to be
/// made to initialize it on first boot.
static uint32_t RTC_NOINIT_ATTR bootloader_request;

/// Value to be assigned to @ref bootloader_request when the bootloader should
/// run instead of normal node operations.
static constexpr uint32_t RTC_BOOL_TRUE = 0x92e01a42;

/// Default value to assign to @ref bootloader_request when the ESP32-C3 starts
/// the first time or when the bootloader should not be run.
static constexpr uint32_t RTC_BOOL_FALSE = 0;

extern "C"
{

/// Callback from the bootloader to configure and start the TWAI hardware.
void bootloader_hw_init(void)
{
    // TWAI driver timing configuration, 125kbps.
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();

    // TWAI driver filter configuration, accept all frames.
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // TWAI driver general configuration.
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(esp_bl_state.tx_pin, esp_bl_state.rx_pin,
                                    TWAI_MODE_NORMAL);
    g_config.tx_queue_len = config_can_tx_buffer_size();
    g_config.rx_queue_len = config_can_rx_buffer_size();

    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Configuring TWAI driver");
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Starting TWAI driver");
    ESP_ERROR_CHECK(twai_start());
    esp_bl_state.twai_initialized = true;
}

/// Callback from the bootloader for entering the application.
///
/// This will default to reboot the ESP32.
void application_entry(void)
{
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] application_entry");

    // reset the RTC persistent variable to not enter bootloader on next
    // restart.
    bootloader_request = RTC_BOOL_FALSE;

    // restart the esp32 since we do not have a way to rerun app_main.
    esp_restart();
}

/// Callback from the bootloader when a reboot should be triggered.
///
/// Currently a NO-OP.
void bootloader_reboot(void)
{
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Reboot requested");
}

/// Callback from the bootloader to read a single CAN frame.
///
/// @param frame buffer for receiving a CAN frame into.
/// @return true when successful, false otherwise.
bool read_can_frame(struct can_frame *frame)
{
    twai_message_t rx_msg;
    memset(&rx_msg, 0, sizeof(twai_message_t));
    if (twai_receive(&rx_msg, MAX_TWAI_WAIT_RX) == ESP_OK)
    {
        LOG(BOOTLOADER_TWAI_LOG_LEVEL, "[Bootloader] CAN_RX");
        frame->can_id = rx_msg.identifier;
        frame->can_dlc = rx_msg.data_length_code;
        frame->can_err = 0;
        frame->can_eff = rx_msg.extd;
        frame->can_rtr = rx_msg.rtr;
        memcpy(frame->data, rx_msg.data, frame->can_dlc);
        return true;
    }
    return false;
}

/// Callback from the bootloader to transmit a single CAN frame.
///
/// @param frame CAN frame to transmit.
/// @return true when successful, false otherwise.
bool try_send_can_frame(const struct can_frame &frame)
{
    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(twai_message_t));
    tx_msg.identifier = frame.can_id;
    tx_msg.data_length_code = frame.can_dlc;
    tx_msg.extd = frame.can_eff;
    tx_msg.rtr = frame.can_rtr;
    memcpy(tx_msg.data, frame.data, frame.can_dlc);
    if (twai_transmit(&tx_msg, MAX_TWAI_WAIT_TX) == ESP_OK)
    {
        LOG(BOOTLOADER_TWAI_LOG_LEVEL, "[Bootloader] CAN_TX");
        return true;
    }
    return false;
}

/// Callback from the bootloader to retrieve flash boundaries.
///
/// @param flash_min Minimum flash address to write to.
/// @param flash_max Maximum flash address to write to.
/// @param app_header Pointer to the app_header struct for the currently
/// running firmware.
///
/// NOTE: This will default to set @param flash_min to zero, @param flash_max
/// to the partition size and @param app_header to a statically declared struct
/// with only the size populated.
void get_flash_boundaries(const void **flash_min, const void **flash_max,
    const struct app_header **app_header)
{
    LOG(BOOTLOADER_LOG_LEVEL,
        "[Bootloader] get_flash_boundaries(%d, %" PRIu32 ")",
        0, esp_bl_state.app_header.app_size);
    *((uint32_t *)flash_min) = 0;
    *((uint32_t *)flash_max) = esp_bl_state.app_header.app_size;
    *app_header = &esp_bl_state.app_header;
}

/// Callback from the bootloader to retrieve flash page information.
///
/// @param address Flash address to retrieve information on.
/// @param page_state Starting address for the page.
/// @param page_length_bytes Length of the page.
void get_flash_page_info(
    const void *address, const void **page_start, uint32_t *page_length_bytes)
{
    uint32_t value = (uint32_t)address;
    value &= ~(CONFIG_WL_SECTOR_SIZE - 1);
    *page_start = (const void *)value;
    *page_length_bytes = CONFIG_WL_SECTOR_SIZE;
    LOG(BOOTLOADER_LOG_LEVEL,
        "[Bootloader] get_flash_page_info(%" PRIu32 ", %" PRIu32")",
        value, *page_length_bytes);
}

/// Callback from the bootloader to erase a flash page.
///
/// @param address Flash address to erase.
///
/// NOTE: This is a NO-OP on the ESP32 as this is handled internally by the
/// esp_ota_write() API.
void erase_flash_page(const void *address)
{
    // NO OP as this is handled automatically as part of esp_ota_write.
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Erase: %" PRIu32,
        (uint32_t)address);
}

/// Callback from the bootloader to write to flash.
///
/// @param address Flash address to write to.
/// @param data Buffer to write to flash.
/// @param size_bytes Number of bytes to write to flash.
///
/// This method leverages the ESP32 OTA APIs to write the flash data. Upon
/// writing to flash address zero (first address), the @param data will be
/// validated to ensure the firmware being received is applicable to the
/// ESP32 and that it appears correct (has correct image magic byte). If the
/// firmware is not applicable or otherwise corrupted for the first flash
/// address the ESP32 will reboot to abort the bootloader download.
void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    uint32_t addr = (uint32_t)address;
    LOG(VERBOSE, "[Bootloader] Write: %" PRIu32 ", %" PRIu32, addr,
        size_bytes);

    // The first part of the received binary should have the image header,
    // segment header and app description. These are used as a first pass
    // validation of the received data to ensure it is a valid firmware.
    if (addr == 0)
    {
        // Mapping of known ESP32 chip id values.
        const char * const CHIP_ID_NAMES[] =
        {
            // Note: this must be kept in sync with esp_chip_id_t.
            "ESP32",            // 0x00 ESP_CHIP_ID_ESP32
            "INVALID",          // 0x01 invalid (placeholder)
            "ESP32-S2",         // 0x02 ESP_CHIP_ID_ESP32S2
            "INVALID",          // 0x03 invalid (placeholder)
            "INVALID",          // 0x04 invalid (placeholder)
            "ESP32-C3",         // 0x05 ESP_CHIP_ID_ESP32C3
            "INVALID",          // 0x06 invalid (placeholder)
            "INVALID",          // 0x07 invalid (placeholder)
            "INVALID",          // 0x08 invalid (placeholder)
            "ESP32-S3",         // 0x09 ESP_CHIP_ID_ESP32S3
            "ESP32-H2",         // 0x0A ESP_CHIP_ID_ESP32H2
            "INVALID",          // 0x0B invalid (placeholder)
            "ESP32-C2",         // 0x0C ESP_CHIP_ID_ESP32C2
        };

        bool should_abort = false;
        esp_image_header_t *image_header = (esp_image_header_t *)data;
        // If the image magic is correct we can proceed with validating the
        // basic details of the image.
        if (image_header->magic == ESP_IMAGE_HEADER_MAGIC)
        {
            LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Chip ID: %s / %x (%x)",
                CHIP_ID_NAMES[image_header->chip_id],
                image_header->chip_id, esp_bl_state.chip_id);
            // validate the image magic byte and chip type to
            // ensure it matches the currently running chip.
            if (image_header->chip_id != ESP_CHIP_ID_INVALID &&
                image_header->chip_id == esp_bl_state.chip_id)
            {
                // start the OTA process at this point, if we have had a
                // previous failure this will reset the OTA process so we can
                // start fresh.
                esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(
                    esp_ota_begin(esp_bl_state.target, OTA_SIZE_UNKNOWN,
                                  &esp_bl_state.ota_handle));
                should_abort = (err != ESP_OK);
            }
            else
            {
                LOG_ERROR("[Bootloader] Firmware does not appear to be valid "
                          "or is for a different chip (%s - %x vs %s - %x).",
                          CHIP_ID_NAMES[image_header->chip_id],
                          image_header->chip_id,
                          CHIP_ID_NAMES[esp_bl_state.chip_id],
                          esp_bl_state.chip_id);
                should_abort = true;
            }
        }
        else
        {
            LOG_ERROR("[Bootloader] Image magic is incorrect: %d vs %d!",
                      image_header->magic, ESP_IMAGE_HEADER_MAGIC);
            should_abort = true;
        }

        // It would be ideal to abort the firmware upload at this point but the
        // bootloader HAL does not offer a way to abort the transfer so instead
        // reboot the node.
        if (should_abort || esp_bl_state.ota_handle == 0)
        {
            // reset the RTC persistent variable to not enter bootloader on
            // next restart.
            bootloader_request = RTC_BOOL_FALSE;

            // cleanup the OTA handle since we are aborting.
            if (esp_bl_state.ota_handle != 0)
            {
                // abort the OTA operation, we do not validate the return code
                // other than logging the error (if any) as this method only
                // cleans up the ota_handle reference within the OTA system and
                // does not impact future usage.
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    esp_ota_abort(esp_bl_state.ota_handle));
            }

            // invalidate ota_handle in case the esp32 does not reboot before
            // the next time address zero comes up again.
            esp_bl_state.ota_handle = 0;
            esp_restart();
        }
    }
    bootloader_led(LED_WRITING, true);
    bootloader_led(LED_ACTIVE, false);
    ESP_ERROR_CHECK(
        esp_ota_write(esp_bl_state.ota_handle, data, size_bytes));
    bootloader_led(LED_WRITING, false);
    bootloader_led(LED_ACTIVE, true);
}

/// Callback from the bootloader to indicate that the full firmware file has
/// been received.
///
/// @return `ERROR_CODE_OK` (0x0000) if the firmware has been received and
/// updated to be used for the next startup, otherwise `ERROR_FIRMWARE_CSUM`
/// (0x2088) indicating there was a failure that may be recoverable by
/// retrying.
uint16_t flash_complete(void)
{
    LOG(INFO, "[Bootloader] Finalizing firmware update");
    esp_err_t res = esp_ota_end(esp_bl_state.ota_handle);
    if (res != ESP_OK)
    {
        LOG_ERROR("[Bootloader] Firmware update failed: %s (%04x), aborting!",
            esp_err_to_name(res), res);
        return openlcb::Defs::ERROR_FIRMWARE_CSUM;        
    }
    LOG(INFO,
        "[Bootloader] Firmware appears valid, updating the next boot "
        "partition to %s.", esp_bl_state.target->label);
    res = esp_ota_set_boot_partition(esp_bl_state.target);
    if (res != ESP_OK)
    {
        LOG_ERROR("[Bootloader] Failed to update the boot partition %s (%04x)!",
            esp_err_to_name(res), res);
        return openlcb::Defs::ERROR_FIRMWARE_CSUM;
    }
    return openlcb::Defs::ERROR_CODE_OK;
}

/// Callback from the bootloader to calculate the checksum of a data block.
///
/// @param data Start of block to calculate checksum for.
/// @param size Number of bytes to calcuate checksum for.
/// @param checksum Calculated checksum for the data block.
///
/// NOTE: The ESP32 does not use this method and will always set the checksum
/// value to zero.
void checksum_data(const void* data, uint32_t size, uint32_t* checksum)
{
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] checksum_data(%" PRIu32 ")", size);
    // Force the checksum to be zero since it is not currently used on the
    // ESP32. The startup of the node may validate the built-in SHA256 and
    // fallback to previous application binary if the SHA256 validation fails.
    memset(checksum, 0, sizeof(uint32_t) * CHECKSUM_COUNT);
}

/// Callback from the bootloader to obtain the pre-defined alias to use.
///
/// @return zero to have the bootloader assign one based on the node-id.
uint16_t nmranet_alias(void)
{
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] nmranet_alias");
    // let the bootloader generate it based on nmranet_nodeid().
    return 0;
}

/// Callback from the bootloader to obtain the node-id to use.
///
/// @return node-id provided to @ref esp32_bootloader_run.
uint64_t nmranet_nodeid(void)
{
    LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] nmranet_nodeid");
    return esp_bl_state.node_id;
}

} // extern "C"

/// Initializes the ESP32 Bootloader.
///
/// @param reset_reason Reason for the ESP32 startup.
///
/// NOTE: This method must be called during the startup of the ESP32, failure
/// to call this method will result in the bootloader checks having undefined
/// behavior since RTC memory is not cleared upon startup!
///
/// Example:
///```
/// void setup()
/// {
///   ...
///   uint8_t reset_reason = Esp32SocInfo.print_soc_info();
///   esp32_bootloader_init(reset_reason);
///   ...
/// }
///```
void esp32_bootloader_init(uint8_t reset_reason)
{
    // If this is the first power up of the node we need to reset the flag
    // since it will not be initialized automatically.
    if (reset_reason == POWERON_RESET)
    {
        bootloader_request = RTC_BOOL_FALSE;
    }
}

/// Runs the ESP32 Bootloader.
///
/// @param id Node ID to advertise on the TWAI (CAN) bus.
/// @param rx Pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
/// @param tx Pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
/// @param reboot_on_exit Reboot the ESP32 upon completion, default is true.
void esp32_bootloader_run(uint64_t id, gpio_num_t rx, gpio_num_t tx,
                          bool reboot_on_exit = true)
{
    memset(&esp_bl_state, 0, sizeof(Esp32BootloaderState));

    esp_bl_state.node_id = id;
    esp_bl_state.tx_pin = tx;
    esp_bl_state.rx_pin = rx;
    esp_bl_state.chip_id = ESP_CHIP_ID_INVALID;

    // Extract the currently running chip details so we can use it to confirm
    // the received firmware is for this chip.
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    switch (chip_info.model)
    {
        case CHIP_ESP32:
            esp_bl_state.chip_id = ESP_CHIP_ID_ESP32;
            break;
        case CHIP_ESP32S2:
            esp_bl_state.chip_id = ESP_CHIP_ID_ESP32S2;
            break;
        case CHIP_ESP32S3:
            esp_bl_state.chip_id = ESP_CHIP_ID_ESP32S3;
            break;
        case CHIP_ESP32C3:
            esp_bl_state.chip_id = ESP_CHIP_ID_ESP32C3;
            break;
        default:
            LOG(FATAL, "[Bootloader] Unknown/Unsupported Chip ID: %x",
                chip_info.model);
    }

    // Initialize the app header details based on the currently running
    // partition.
    esp_bl_state.current = (esp_partition_t *)esp_ota_get_running_partition();
    esp_bl_state.app_header.app_size = esp_bl_state.current->size;

    // Find the next OTA partition and confirm it is not the currently running
    // partition.
    esp_bl_state.target =
        (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
    if (esp_bl_state.target != nullptr &&
        esp_bl_state.target != esp_bl_state.current)
    {
        LOG(INFO, "[Bootloader] Preparing to receive firmware");
        LOG(INFO, "[Bootloader] Current partition: %s",
            esp_bl_state.current->label);
        LOG(INFO, "[Bootloader] Target partition: %s",
            esp_bl_state.target->label);

        // since we have the target partition identified, start the bootloader.
        LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] calling bootloader_entry");
        bootloader_entry();
    }
    else
    {
        LOG_ERROR("[Bootloader] Unable to locate next OTA partition!");
    }

    if (esp_bl_state.twai_initialized)
    {
        LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Stopping TWAI driver");
        ESP_ERROR_CHECK(twai_stop());
        LOG(BOOTLOADER_LOG_LEVEL, "[Bootloader] Disabling TWAI driver");
        ESP_ERROR_CHECK(twai_driver_uninstall());
    }

    // reset the RTC persistent variable to not enter bootloader on next
    // restart.
    bootloader_request = RTC_BOOL_FALSE;

    if (reboot_on_exit)
    {
        // If we reach here we should restart the node.
        LOG(INFO, "[Bootloader] Restarting!");
        esp_restart();
    }
}

#endif // _FREERTOS_DRIVERS_ESP32_ESP32BOOTLOADERHAL_HXX_