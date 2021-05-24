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
 * \file Esp32HardwareTwai.hxx
 *
 * TWAI driver implementation for OpenMRN. This leverages the ESP-IDF TWAI HAL
 * API rather than the TWAI driver to allow for a more integrated solution than
 * the TWAI driver which requires polling for RX. This implementation supports
 * both ::select and the non-blocking ::ioctl/::fnctl approach.
 *
 * @author Mike Dunston
 * @date 1 May 2021
 */
#ifndef _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_

namespace openmrn_arduino
{

#include <esp_idf_version.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "nmranet_config.h"
#include "utils/Singleton.hxx"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,3,0)
#error Esp32HardwareTwai is only supported on ESP-IDF v4.3 and above.
#endif // IDF v4.3+

/// ESP32 Hardware TWAI (CAN) driver interface.
///
/// The ESP32 has a hardware TWAI controller that requires an external CAN
/// transceiver connected via two GPIO pins (RX and TX). SPI connected CAN
/// transceivers are not supported by this interface.
///
/// Example of usage (async API):
///```
/// Esp32HardwareTwai twai;
/// void setup() {
///   ...
///   twai.hw_init();
///   openmrn.begin();
///   openmrn.add_can_port_async("/dev/twai/twai0");
///   ...
/// }
///```
///
/// Example of usage (select API):
///```
/// Esp32HardwareTwai twai;
/// void setup() {
///   ...
///   twai.hw_init();
///   openmrn.begin();
///   openmrn.add_can_port_select("/dev/twai/twai0");
///   openmrn.start_executor_thread();
///   ...
/// }
///```
/// NOTE: For the select API it is necessary to start the executor thread in
/// the setup() method.
///
/// NOTE: The select API is not be usable without CONFIG_VFS_SUPPORT_SELECT
/// being enabled in sdkconfig, this option is disabled automatically when
/// CONFIG_LWIP_USE_ONLY_LWIP_SELECT is enabled. CONFIG_VFS_SUPPORT_SELECT is
/// enabled by default in arduino-esp32.
class Esp32HardwareTwai : public Singleton<Esp32HardwareTwai>
{
public:
    /// Constructor.
    ///
    /// @param rx is the GPIO pin connected to the CAN transceiver RX pin.
    /// @param tx is the GPIO pin connected to the CAN transceiver TX pin.
    /// @param report controls the periodic reporting of the TWAI driver
    /// statistics, default is enabled.
    /// @param rx_buffer_size is the number of @ref can_frame to queue before
    /// frames will be dropped/lost, default is defined in
    /// @var _sym_can_rx_buffer_size.
    /// @param tx_buffer_size is the number of @ref can_frame to queue before
    /// blocking will occur when transmitting, default is defined in
    /// @var _sym_can_tx_buffer_size.
    /// @param path is the VFS mount point for the TWAI driver, default is
    /// "/dev/twai".
    /// @param clock_out is the GPIO pin that can be used for an external clock
    /// pin, default is disabled (-1). When enabled this will have a pre-scaled
    /// clock signal.
    /// @param bus_status is the GPIO pin that can be used for a bus status
    /// indicator, default is disabled (-1). When enabled this pin will be set
    /// LOW (0v) when the TWAI driver is in a "Bus Off" state and will be set
    /// HIGH (3.3v) otherwise.
    ///
    /// NOTE: The CAN transceiver must internally loopback TX to RX, failure to
    /// do so will be interpreted as an arbitration loss or bit error.
    Esp32HardwareTwai(int rx, int tx
                    , bool report = true
                    , size_t rx_buffer_size = config_can_rx_buffer_size()
                    , size_t tx_buffer_size = config_can_tx_buffer_size()
                    , const char *path = "/dev/twai"
                    , int clock_out = GPIO_NUM_NC
                    , int bus_status = GPIO_NUM_NC);

    /// Destructor.
    ~Esp32HardwareTwai();

    /// Initializes the TWAI hardware and VFS adapter.
    ///
    /// NOTE: This must be called prior to adding the TWAI driver to the
    /// @ref SimpleCanStack.
    void hw_init();

private:
    /// Default constructor.
    Esp32HardwareTwai();

    /// GPIO pin connected to the external transceiver RX pin.
    const int rxPin_;

    /// GPIO pin connected to the external transceiver TX pin.
    const int txPin_;

    /// GPIO pin that generates an external clock signal.
    const int extClockPin_;

    /// GPIO pin connected to an external bus status indicator.
    const int busStatusPin_;

    /// VFS Mount point.
    const char *vfsPath_;

    DISALLOW_COPY_AND_ASSIGN(Esp32HardwareTwai);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32HardwareTwai;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_