/** \copyright
 *
 *    Copyright (C) 2018  Robert Heller D/B/A Deepwoods Software
 *			51 Locke Hill Road
 *			Wendell, MA 01379-9728
 *
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
 *
 * \file LinuxGpio.hxx
 *
 * Defines GPIO pins using the Linux sysfs ABI.
 * 
 * \section HOWTOUSE How to use
 * 
 * You need to use the GPIO_PIN macro at the bottom, like this:
 * 
 * GPIO_PIN(LED1, GpioOutputSafeLow, 27);  // Defines LED1_Pin, for GPIO 27, initialized low. 
 * GPIO_PIN(CSLow, GpioOutputSafeHighInvert, 5);  // Defines CSLow_Pin, for GPIO 5, initialized high, with the set logic inverted. 
 * GPIO_PIN(Button1, GpioInputActiveLow, 20);  // Defines Button1_Pin, for GPIO 20, avtive low -- return true when shorted to ground. 
 *
 * Classes available for the second macro parameter are:
 * 
 *   - GpioOutputSafeLow        Output initialized low, true = high
 *   - GpioOutputSafeLowInvert  Output initialized low, true = low
 *   - GpioOutputSafeHigh       Output initialized high, true = high
 *   - GpioOutputSafeHighInvert Output initialized high, true = high
 *   - GpioInputActiveHigh      Input, high = true
 *   - GpioInputActiveLow       Input, low  = true
 * 
 * Be sure to use GpioInitializer to create an Initializer class:
 * 
 * typedef GpioInitializer<LED1_Pin, CSLow_Pin, Button1_Pin> GpioInit;
 * 
 * somewhere in main.cxx, and then in appl_main():
 * 
 * GpioInit::hw_init();
 * 
 * This makes sure the GPIO pins are properly set up (eg exported to /sys/class/gpio/).
 * Also, the process running the node needs to be in group gpio.
 * 
 * @author Robert Heller
 * @date 10 October 2018
 */

#ifndef __LINUXGPIO_HXX
#define __LINUXGPIO_HXX

#include "freertos_drivers/common/GpioWrapper.hxx"
#include "os/Gpio.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

template <class Defs, bool SAFE_VALUE, bool INVERT> 
         struct GpioOutputPin;
template <class Defs, bool ACTIVE_HIGH> struct GpioInputPin;

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
/// Uses Linux sysfs ABI
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <int PIN> class LinuxGpio : public Gpio
{
public:
    /// Sets the output state of the connected GPIO pin.
    ///
    /// @param new_state State to set the GPIO pin to.
    void write(Value new_state) const override
    {
        char valname[40];
        snprintf(valname,sizeof(valname),"/sys/class/gpio/gpio%d/value",PIN);
        int vfd = open(valname,O_WRONLY);
        if (vfd <  0) {
            LOG(WARNING, "LinuxGpio::write(): pin (%d) not exported!",PIN);
            return;
        }
        if (::write(vfd, new_state == Value::SET ? "1\n" : "0\n", 2) < 2) {
            LOG(WARNING, "LinuxGpio::write(): cannot set value of pin (%d)!",PIN);
        }
        close(vfd);
    }
    /// Reads the current state of the connected GPIO pin.
    /// @return @ref SET if currently high, @ref CLR if currently low.
    Value read() const override
    {
        char valname[40], c = '0';
        snprintf(valname,sizeof(valname),"/sys/class/gpio/gpio%d/value",PIN);
        int vfd = open(valname,O_RDONLY);
        if (vfd <  0) {
            LOG(WARNING, "LinuxGpio::read(): pin (%d) not exported!",PIN);
            return Value::CLR;
        }
        if (::read(vfd,&c,1) < 1) {
            LOG(WARNING, "LinuxGpio::read(): cannot get value of pin (%d)!",PIN);
        }
        close(vfd);
        return c == '1'? Value::SET : Value::CLR;
    }
    /// Sets output to HIGH.
    void set() const override
    {
        write(Value::SET);
    }
    
    /// Sets output to LOW.
    void clr() const override
    {
        write(Value::CLR);
    }
    
    /// Sets the direction of the connected GPIO pin.
    void set_direction(Gpio::Direction dir) const override
    {
        char dirname[40];
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN);
        int dfd = -1;
        if ((dfd = open(dirname,O_WRONLY)) < 0) {
            LOG(WARNING, "LinuxGpio::set_direction(): pin (%d) not exported!",PIN);
            return;
        }
        const char *dirmessage = dir == Gpio::Direction::DOUTPUT? "out\n" : "in\n";
        if (::write(dfd, dirmessage, strlen(dirmessage)) < (ssize_t)strlen(dirmessage)) {
            LOG(WARNING, "LinuxGpio::set_direction(): cannot set direction of pin (%d)!",PIN);
        }
        close(dfd);
        if (dir != direction()) {
            LOG(WARNING, "LinuxGpio::set_direction(): failed to set direction of pin (%d)!",PIN);
        }
    }

    /// Gets the GPIO direction.
    /// @return @ref DINPUT or @ref DOUTPUT
    Gpio::Direction direction() const override
    {
        char dirname[40], c = 'o';
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN);
        int dfd = -1;
        if ((dfd = open(dirname,O_RDONLY)) < 0) {
            LOG(WARNING, "LinuxGpio::direction(): pin (%d) not exported!",PIN);
            return Gpio::Direction::DOUTPUT;
        }
        if (::read(dfd,&c,1) < 1) {
            LOG(WARNING, "LinuxGpio::direction(): cannot get direction of pin (%d)!",PIN);
        }
        close(dfd);
        return (c == 'o')? Gpio::Direction::DOUTPUT: Gpio::Direction::DINPUT;
    }
private:
    template <class Defs, bool SAFE_VALUE, bool INVERT> 
          friend struct GpioOutputPin;
    template <class Defs, bool ACTIVE_HIGH> \
          friend struct GpioInputPin;
    static const LinuxGpio instance_;
};

template <int PIN_NUM>
const LinuxGpio<PIN_NUM> LinuxGpio<PIN_NUM>::instance_;

/// Parametric GPIO output class.
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param SAFE_VALUE is the initial value for the GPIO output pin.
/// @param INVERT inverts the high/low state of the pin when set.
template <class Defs, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Defs
{
public:
    using Defs::PIN_NUM;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        LOG(VERBOSE,
            "[LinuxGpio] Configuring output pin %d, default value: %d",
            PIN_NUM, SAFE_VALUE);
        FILE *fp = fopen("/sys/class/gpio/export","w");
        fprintf(fp,"%d\n",PIN_NUM);
        fclose(fp);
        char dirname[40];
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN_NUM);
        while (access(dirname,W_OK|R_OK) != 0) {
            // 50ms delay IS needed while kernel (udev) changes 
            // ownership of created GPIO directory
            // Note: when there are LOTS of GPIO lines exported, the
            // uDev ownership update might take awile -- keep sleeping
            usleep(50000); 
        }
        instance()->set_direction(Gpio::Direction::DOUTPUT);
    }
    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        instance()->write(SAFE_VALUE);
    }
    
    /// Toggles the state of the pin to the opposite of what it is currently.
    static void toggle()
    {
        instance()->write(!instance()->read());
    }
    
    /// Sets the output pin @param value if true, output is set to HIGH, if
    /// false, output is set to LOW.
    static void set(bool value)
    {
        if (INVERT) {
            instance()->write(!value);
        } else {
            instance()->write(value);
        }
    }
    
    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &LinuxGpio<PIN_NUM>::instance_;
    }
};

/// Defines a GPIO output pin, initialized to be an output pin with low level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLow : public GpioOutputPin<Defs, false>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with low
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLowInvert : public GpioOutputPin<Defs, false, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHighInvert : public GpioOutputPin<Defs, true, true>
{
};

/// Parametric GPIO input class.
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param ACTIVE_HIGH is true if the input is active high and false
/// if the input is active low
template <class Defs, bool ACTIVE_HIGH = true> struct GpioInputPin : public Defs
{
public:
        using Defs::PIN_NUM;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        LOG(VERBOSE, "[LinuxGpio] Configuring input pin %d, ACTIVE_HIGH: %d",
            PIN_NUM, ACTIVE_HIGH);
        FILE *fp = fopen("/sys/class/gpio/export","w");
        fprintf(fp,"%d\n",PIN_NUM);
        fclose(fp);
        char dirname[40];
        snprintf(dirname,sizeof(dirname),"/sys/class/gpio/gpio%d/direction",PIN_NUM);
        while (access(dirname,W_OK|R_OK) != 0) {
            // 50ms delay IS needed while kernel (udev) changes 
            // ownership of created GPIO directory
            // Note: when there are LOTS of GPIO lines exported, the
            // uDev ownership update might take awile -- keep sleeping
            usleep(50000); 
        }
        instance()->set_direction(Gpio::Direction::DINPUT);
    }   
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &LinuxGpio<PIN_NUM>::instance_;
    }
};

/// Defines a GPIO input pin active high
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputActiveHigh : public GpioInputPin<Defs, true>
{
};

/// Defines a GPIO input pin active low
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputActiveLow : public GpioInputPin<Defs, false>
{
};

/// Helper macro for defining GPIO pins on Linux-based microcontrollers (like the Raspberry Pi or Bagle Bone.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM)                \
struct NAME##Defs                                     \
{                                                     \
    static const int PIN_NUM = (int)NUM;\
public:                                               \
    static const int pin()                     \
    {                                                 \
        return PIN_NUM;                               \
    }                                                 \
};                                                    \
typedef BaseClass<NAME##Defs> NAME##_Pin


#endif // __LINUXGPIO_HXX

