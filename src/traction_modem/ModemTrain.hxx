/** @copyright
 * Copyright (c) 2025, Stuart Baker
 * All rights reserved
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
 * @file ModemTrain.hxx
 *
 * Implements a Traction Modem Train.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_TRAIN_HXX_
#define _TRACTION_MODEM_TRAIN_HXX_

#include "openlcb/TrainInterface.hxx"
#include "traction_modem/Defs.hxx"
#include "traction_modem/MemorySpace.hxx"
#include "traction_modem/Output.hxx"
#include "traction_modem/Link.hxx"

namespace traction_modem
{

// Forward declaration
class ModemTrainHwInterface;

/// ModemTrain definition.
class ModemTrain : public openlcb::TrainImpl
{
public:
    /// Constructor.
    /// @param service Service instance to bind this flow to.
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    /// @param hw_interface hardware specific interface to the modem train.
    ModemTrain(Service *service, TxInterface *tx_flow, RxInterface *rx_flow,
        ModemTrainHwInterface *hw_interface)
        : txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , link_(tx_flow, rx_flow)
        , linkManager_(service, &link_)
        , cvSpace_(service, &link_)
        , fuSpace_(service, &link_)
        , output_(tx_flow, rx_flow, hw_interface)
        , isActive_(false)
    {
    }

    /// Destructor.
    virtual ~ModemTrain()
    {
    }

    /// Start the flow of modem data.
    /// @param uart_fd file descriptor to send and receive data on
    void start(int uart_fd)
    {
        link_.start(uart_fd);
    }

    /// Get a reference to the train's transmit flow.
    /// @return reference to transmit flow
    TxInterface *get_tx_flow()
    {
        return txFlow_;
    }

    /// Get a reference to the train's receive flow.
    /// @return reference to receive flow
    RxInterface *get_rx_flow()
    {
        return rxFlow_;
    }

    /// Set the active state of the wireless control.
    /// @param is_active true if under wireless control, else false
    void set_is_active(bool is_active)
    {
        if (isActive_ != is_active)
        {
            isActive_ = is_active;
            txFlow_->send_packet(Defs::get_wireless_present_payload(isActive_));
        }
    }

    /// Get a reference to the CV memory space.
    /// @return CV memory space reference
    openlcb::MemorySpace *get_cv_space()
    {
        return &cvSpace_;
    }

    /// Get a reference to the firmware update memory space.
    /// @return firmware update memory space reference
    openlcb::MemorySpace *get_firmware_update_space()
    {
        return &fuSpace_;
    }

    // ====== Train interface =======

    /// Set train speed.
    /// @param speed speed to set the train to.
    void set_speed(openlcb::SpeedType speed) override
    {
        set_is_active(true);
        inEStop_ = false;
        lastSpeed_ = speed;
        txFlow_->send_packet(Defs::get_speed_set_payload(speed));
    }

    /// Returns the last set speed of the locomotive.
    /// @return last set speed of the locomotive.
    openlcb::SpeedType get_speed() override
    {
        return lastSpeed_;
    }

    /// Set emergency top active.
    void set_emergencystop() override
    {
        inEStop_ = true;
        lastSpeed_.set_mph(0); // keeps direction
        txFlow_->send_packet(Defs::get_estop_payload());
    }

    /// Get the current emergency stop state
    /// @return  true if current state is E-Stop, else false
    bool get_emergencystop() override
    {
        return inEStop_;
    }

    /// Sets the value of a function.
    /// @param address is a 24-bit address of the function to set. For legacy
    ///        DCC locomotives, see @ref TractionDefs for the address
    ///        definitions (0=light, 1-28= traditional function buttons).
    /// @param value is the function value. For binary functions, any non-zero
    ///        value sets the function to on, zero sets it to off.*/
    void set_fn(uint32_t address, uint16_t value) override
    {
        set_is_active(true);
        txFlow_->send_packet(Defs::get_fn_set_payload(address, value));
    }

    /// Get the current function value.
    /// @return the current value of the function
    uint16_t get_fn(uint32_t address) override
    {
        /// @todo Need to implement this.
        return 0;
    }

    /// @Get the legacy address.
    /// @return legacy address (typically DCC)
    uint32_t legacy_address() override
    {
        /// @todo What should this be? Should we do a CV1/17/18/29 read to get
        ///       this?
        return 883;
    }

    /// Get the type of legacy protocol in use.
    /// @return the legacy address type
    dcc::TrainAddressType legacy_address_type() override
    {
        /// @todo What should this be. Should we do a CV29 read to get this?
        return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }

private:
    /// True if the wireless is up and running.
    bool isRunning_ = false;
    /// UART fd to send traffic to the device.
    int fd_;
    /// last/current speed of the locomotive.
    openlcb::SpeedType lastSpeed_ = 0.0;
    /// Handles sending message frames.
    TxInterface *txFlow_;
    /// Handles receiving message frames.
    RxInterface *rxFlow_;
    /// Link status object.
    Link link_;
    /// Link Management object.
    LinkManager linkManager_;
    /// Space for CV read/write.
    CvSpace cvSpace_;
    /// Space for firmware updates.
    CvSpace fuSpace_;
    /// Output handler.
    Output output_;
    /// True if the last set was estop, false if it was a speed.
    bool inEStop_ = false;
    /// Is the wireless active.
    bool isActive_;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_TRAIN_HXX_