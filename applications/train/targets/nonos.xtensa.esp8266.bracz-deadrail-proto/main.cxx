/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file main.cxx
 *
 * An application that blinks an LED.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#include <stdio.h>
#include <unistd.h>

#include "dcc/Defs.hxx"
#include "executor/StateFlow.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/esp8266/Esp8266Gpio.hxx"
#include "freertos_drivers/esp8266/TimerBasedPwm.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/TractionTrain.hxx"
#include "nmranet/TrainInterface.hxx"
#include "os/os.h"
#include "utils/ESPWifiClient.hxx"
#include "utils/GpioInitializer.hxx"
#include "utils/blinker.h"

#include "config.hxx"

extern "C" {
#include <gpio.h>
#include <osapi.h>
#include <user_interface.h>
extern void ets_delay_us(uint32_t us);
#define os_delay_us ets_delay_us
}

#include "nmranet/TrainInterface.hxx"
#include "hardware.hxx"

struct SpeedRequest
{
    SpeedRequest()
    {
        reset();
    }
    nmranet::SpeedType speed_;
    bool emergencyStop_;
    void reset()
    {
        speed_ = 0.0;
        emergencyStop_ = false;
    }
};

class SpeedController : public StateFlow<Buffer<SpeedRequest>, QList<2>>,
                        private DefaultConfigUpdateListener
{
public:
    SpeedController(Service *s, MotorControl mpar)
        : StateFlow<Buffer<SpeedRequest>, QList<2>>(s)
        , mpar_(mpar)
    {
        HW::MOT_A_HI_Pin::set_off();
        HW::MOT_B_HI_Pin::set_off();
        pwm_.enable();
    }

    void call_speed(nmranet::Velocity speed)
    {
        auto *b = alloc();
        b->data()->speed_ = speed;
        send(b, 1);
        /*
        long long period = USEC_TO_NSEC(1000);
        long long fill = speed_to_fill_rate(speed, period);
        if (speed.direction() == speed.FORWARD) {
            GPIO_OUTPUT_SET(2, 1);
            pwm_.old_set_state(0, period, period - fill);
        } else {
            GPIO_OUTPUT_SET(0, 1);
            pwm_.old_set_state(2, period, period - fill);
            }*/
    }

    void call_estop()
    {
        auto *b = alloc();
        b->data()->emergencyStop_ = true;
        send(b, 0);
    }

private:
    static constexpr bool invertLow_ = HW::invertLow;

    long long speed_to_fill_rate(nmranet::SpeedType speed, long long period)
    {
        int fill_rate = speed.mph();
        if (fill_rate >= 128)
            fill_rate = 128;
        // Let's do a 1khz
        long long fill = (period * fill_rate) >> 7;
        if (invertLow_)
            fill = period - fill;
        return fill;
    }

    Action entry() override
    {
        if (req()->emergencyStop_)
        {
            pwm_.pause();
            HW::MOT_A_HI_Pin::set_off();
            HW::MOT_B_HI_Pin::set_off();
            release();
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(1), STATE(eoff_enablelow));
        }
        // Check if we need to change the direction.
        bool desired_dir =
            (req()->speed_.direction() == nmranet::SpeedType::FORWARD);
        if (lastDirMotAHi_ != desired_dir)
        {
            pwm_.pause();
            HW::MOT_B_HI_Pin::set_off();
            HW::MOT_A_HI_Pin::set_off();
            return sleep_and_call(&timer_, MSEC_TO_NSEC(1), STATE(do_speed));
        }
        return call_immediately(STATE(do_speed));
    }

    Action do_speed()
    {
        // We set the pins explicitly for safety
        bool desired_dir =
            (req()->speed_.direction() == nmranet::SpeedType::FORWARD);
        int lo_pin;
        if (desired_dir)
        {
            HW::MOT_B_HI_Pin::set_off();
            HW::MOT_A_LO_Pin::set(invertLow_);
            lo_pin = HW::MOT_B_LO_Pin::PIN;
            HW::MOT_A_HI_Pin::set_on();
        }
        else
        {
            HW::MOT_A_HI_Pin::set_off();
            HW::MOT_B_LO_Pin::set(invertLow_);
            lo_pin = HW::MOT_A_LO_Pin::PIN;
            HW::MOT_B_HI_Pin::set_on();
        }

        long long fill = speed_to_fill_rate(req()->speed_, period_);
        pwm_.old_set_state(lo_pin, period_, fill);
        lastDirMotAHi_ = desired_dir;
        return release_and_exit();
    }

    Action eoff_enablelow()
    {
        // By shorting both motor outputs to ground we turn it to actively
        // brake.
        HW::MOT_A_LO_Pin::set(!invertLow_);
        HW::MOT_B_LO_Pin::set(!invertLow_);
        return exit();
    }

    SpeedRequest *req()
    {
        return message()->data();
    }

    void factory_reset(int fd) override
    {
        mpar_.pwm_frequency().write(
            fd, mpar_.pwm_frequency_options().defaultvalue());
    }
    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        auto config_freq = mpar_.pwm_frequency().read(fd);
        printf("pwm freq = %d\n", config_freq);
        period_ = 1000000000 / config_freq;
        return UPDATED;
    }

    MotorControl mpar_;
    long long period_ =
        1000000000 / MotorControl::pwm_frequency_options().defaultvalue();
    TimerBasedPwm pwm_;
    StateFlowTimer timer_{this};
    bool lastDirMotAHi_{false};
};

extern SpeedController g_speed_controller;

class ESPHuzzahTrain : public nmranet::TrainImpl
{
public:
    ESPHuzzahTrain()
    {
        HW::GpioInit::hw_init();
        HW::LIGHT_FRONT_Pin::set(false);
        HW::LIGHT_BACK_Pin::set(false);
    }

    void set_speed(nmranet::SpeedType speed) override
    {
        lastSpeed_ = speed;
        g_speed_controller.call_speed(speed);
        if (f0)
        {
            if (speed.direction() == nmranet::SpeedType::FORWARD)
            {
                HW::LIGHT_FRONT_Pin::set(true);
                HW::LIGHT_BACK_Pin::set(false);
            }
            else
            {
                HW::LIGHT_BACK_Pin::set(true);
                HW::LIGHT_FRONT_Pin::set(false);
            }
        }
    }
    /** Returns the last set speed of the locomotive. */
    nmranet::SpeedType get_speed() override
    {
        return lastSpeed_;
    }

    /** Sets the train to emergency stop. */
    void set_emergencystop() override
    {
        // g_speed_controller.call_estop();
        lastSpeed_.set_mph(0); // keeps direction
    }

    /** Sets the value of a function.
     * @param address is a 24-bit address of the function to set. For legacy DCC
     * locomotives, see @ref TractionDefs for the address definitions (0=light,
     * 1-28= traditional function buttons).
     * @param value is the function value. For binary functions, any non-zero
     * value sets the function to on, zero sets it to off.*/
    void set_fn(uint32_t address, uint16_t value) override
    {
        switch (address)
        {
            case 0:
                f0 = value;
                if (!value)
                {
                    HW::LIGHT_FRONT_Pin::set(false);
                    HW::LIGHT_BACK_Pin::set(false);
                }
                else if (lastSpeed_.direction() == nmranet::SpeedType::FORWARD)
                {
                    HW::LIGHT_FRONT_Pin::set(true);
                    HW::LIGHT_BACK_Pin::set(false);
                }
                else
                {
                    HW::LIGHT_BACK_Pin::set(true);
                    HW::LIGHT_FRONT_Pin::set(false);
                }
                break;
            case 1:
                /*if (value) {
                    analogWrite(2, 700);
                } else {
                    analogWrite(2, 100);
                    }*/
                f1 = value;
                HW::F1_Pin::set(!value);
                break;
        }
    }

    /** @returns the value of a function. */
    uint16_t get_fn(uint32_t address) override
    {
        switch (address)
        {
            case 0:
                return f0 ? 1 : 0;
                break;
            case 1:
                return f1 ? 1 : 0;
                break;
        }
        return 0;
    }

    uint32_t legacy_address() override
    {
        return 883;
    }

    /** @returns the type of legacy protocol in use. */
    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }

private:
    nmranet::SpeedType lastSpeed_ = 0.0;
    bool f0 = false;
    bool f1 = false;
};

const char kFdiXml[] =
    R"(<?xml version='1.0' encoding='UTF-8'?>
<?xml-stylesheet type='text/xsl' href='xslt/fdi.xsl'?>
<fdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/trunk/prototypes/xml/schema/fdi.xsd'>
<segment space='249'><group><name/>
<function size='1' kind='binary'>
<name>Light</name>
<number>0</number>
</function>
<function size='1' kind='momentary'>
<name>Blue</name>
<number>1</number>
</function>
</group></segment></fdi>)";

ESPHuzzahTrain trainImpl;
nmranet::ConfigDef cfg(0);

extern nmranet::NodeID NODE_ID;

namespace nmranet
{

extern const char *const CONFIG_FILENAME = "openlcb_config";
// The size of the memory space to export over the above device.
extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();
extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;

} // namespace nmranet

nmranet::SimpleTrainCanStack stack(&trainImpl, kFdiXml, NODE_ID);
SpeedController g_speed_controller(stack.service(), cfg.seg().motor_control());

extern "C" {
extern char WIFI_SSID[];
extern char WIFI_PASS[];
extern char WIFI_HUB_HOSTNAME[];
extern int WIFI_HUB_PORT;
}

class TestBlinker : public StateFlowBase
{
public:
    TestBlinker()
        : StateFlowBase(stack.service())
    {
        start_flow(STATE(doo));
        pwm_.enable();
    }

private:
    Action doo()
    {
        if (isOn_)
        {
            isOn_ = false;
            pwm_.old_set_state(2, USEC_TO_NSEC(1000), USEC_TO_NSEC(800));
        }
        else
        {
            isOn_ = true;
            pwm_.old_set_state(2, USEC_TO_NSEC(1000), USEC_TO_NSEC(200));
        }
        return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(doo));
    }

    TimerBasedPwm pwm_;
    StateFlowTimer timer_{this};
    bool isOn_{true};
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    gpio16_output_conf();
    gpio16_output_set(1);
    printf("pinout: B hi %d; B lo %d; A hi %d; A lo %d;\n",
           HW::MOT_B_HI_Pin::PIN, 
           HW::MOT_B_LO_Pin::PIN, 
           HW::MOT_A_HI_Pin::PIN, 
           HW::MOT_A_LO_Pin::PIN);
    resetblink(1);
    if (true)
        stack.create_config_file_if_needed(cfg.seg().internal_data(),
            nmranet::EXPECTED_VERSION, nmranet::CONFIG_FILE_SIZE);

    new ESPWifiClient(WIFI_SSID, WIFI_PASS, stack.can_hub(), WIFI_HUB_HOSTNAME,
                      WIFI_HUB_PORT, 1200, []()
        {
            resetblink(0);
            // This will actually return due to the event-driven OS
            // implementation of the stack.
            stack.loop_executor();
        });
    return 0;
}
