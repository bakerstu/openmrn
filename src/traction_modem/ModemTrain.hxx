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
 * @file Train.hxx
 *
 * Implements a Traction Modem Train.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_TRAIN_HXX_
#define _TRACTION_MODEM_TRAIN_HXX_

/// @todo Need to prune out the "hardware.hxx" dependencies from this file.
///       These need to be dispatched to hardware specific code somehow.
#include "traction_modem/Defs.hxx"
#if !defined(GTEST)
#include "hardware.hxx"
#endif

#include "executor/Dispatcher.hxx"
#include "openlcb/TrainInterface.hxx"
#include "traction_modem/TxFlow.hxx"
#include "traction_modem/RxFlow.hxx"

namespace traction_modem
{

class ModemTrain : public openlcb::TrainImpl
{
public:
    ModemTrain(Service *service)
        : txFlow_(service)
        , rxFlow_(service)
        , dispatcher_(service)
        , isActive_(false)
    {
        rxFlow_.set_listener(&cvSpace_);
    }

    void start(int uart_fd)
    {
        txFlow_.start(uart_fd);
        rxFlow_.start(uart_fd);
    }

    /// Set the active state of the wireless control.
    /// @param is_active true if under wireless control, else false
    void set_is_active(bool is_active)
    {
        if (isActive_ != is_active)
        {
            isActive_ = is_active;
            send_packet(Defs::get_wireless_present_payload(isActive_));
        }
    }

    openlcb::MemorySpace *get_cv_space()
    {
        return &cvSpace_;
    }

    // ====== Train interface =======

    void set_speed(openlcb::SpeedType speed) override
    {
        set_is_active(true);
        inEStop_ = false;
        lastSpeed_ = speed;
        send_packet(Defs::get_speed_set_payload(speed));
    }

    /** Returns the last set speed of the locomotive. */
    openlcb::SpeedType get_speed() override
    {
        return lastSpeed_;
    }

    /** Sets the train to emergency stop. */
    void set_emergencystop() override
    {
        inEStop_ = true;
        lastSpeed_.set_mph(0); // keeps direction
        send_packet(Defs::get_estop_payload());
    }

    bool get_emergencystop() override
    {
        return inEStop_;
    }

    /** Sets the value of a function.
     * @param address is a 24-bit address of the function to set. For legacy DCC
     * locomotives, see @ref TractionDefs for the address definitions (0=light,
     * 1-28= traditional function buttons).
     * @param value is the function value. For binary functions, any non-zero
     * value sets the function to on, zero sets it to off.*/
    void set_fn(uint32_t address, uint16_t value) override
    {
        set_is_active(true);
        send_packet(Defs::get_fn_set_payload(address, value));
        /// @todo The following switch statement is for hardware testing only.
        ///       In production software, the main MCU should have the on/off
        ///       logical by "output" number, and not by function number. The
        ///       main MCU should instruct the modem of the activity state.
        switch (address)
        {
            default:
                break;
#if !defined(GTEST)
            case 0:
                if (lastSpeed_.direction() == openlcb::Velocity::REVERSE)
                {
                    LOGIC_F0R_Pin::set(value ? 1 : 0);
                }
                break;
            case 1:
                LOGIC_F1_Pin::set(value ? 1 : 0);
                break;
            case 2:
                LOGIC_F2_Pin::set(value ? 1 : 0);
                break;
            case 3:
                LOGIC_F3_Pin::set(value ? 1 : 0);
                break;
            case 4:
                LOGIC_F4_Pin::set(value ? 1 : 0);
                break;
            // F5 and F6 also overwritten with input1 and input2 values when
            // button 7 or 8 are pressed
            case 5:
                LOGIC_F5_Pin::set(value ? 1 : 0);
                break;
            case 6:
                LOGIC_F6_Pin::set(value ? 1 : 0);
                break;
#endif // !defined(GTEST)
// Commented out since these pins cannot be defined as both inputs
// and outputs.
#if 0
            case 7:
                LOGIC_F7_Pin::set(value ? 1 : 0);
                break;
            case 8:
                LOGIC_F8_Pin::set(value ? 1 : 0);
                break;
#endif
#if !defined(GTEST)
            case 7:
            {
                bool input1 = INPUT1_Pin::get();
                LOGIC_F5_Pin::set(INPUT2_Pin::get());
                LOG(INFO, "INPUT1: %u", input1);
                break;
            }
            case 8:
            {
                bool input2 = INPUT2_Pin::get();
                LOGIC_F6_Pin::set(INPUT2_Pin::get());
                LOG(INFO, "INPUT2: %u", input2);
                break;
            }
#endif // !defined(GTEST)
        }
    }

    /** @returns the value of a function. */
    uint16_t get_fn(uint32_t address) override
    {
        /// @todo Need to implement this.
        return 0;
    }

    uint32_t legacy_address() override
    {
        /// @todo what should this be?
        return 883;
    }

    /** @returns the type of legacy protocol in use. */
    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }

    /// Register a message handler.
    /// @param interface interface to dispatch the messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    void register_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK)
    {
        dispatcher_.register_handler(interface, id, mask);
    }

private:
    inline void send_packet(Defs::Payload p)
    {
        auto *b = txFlow_.alloc();
        b->data()->payload = std::move(p);
        txFlow_.send(b);
    }

    bool isRunning_ = false;
    /// UART fd to send traffic to the device.
    int fd_;

    openlcb::SpeedType lastSpeed_ = 0.0;
    /// True if the last set was estop, false if it was a speed.
    bool inEStop_ = false;

    /// Handles sending message frames.
    TxFlow txFlow_;
    /// Handles receiving message frames.
    RxFlow rxFlow_;
    /// Handles incoming messages fro the RX Flow.
    DispatchFlow<Buffer<Message>, 2> dispatcher_;

    CvSpace cvSpace_{&txFlow_};
    bool isActive_;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_TRAIN_HXX_