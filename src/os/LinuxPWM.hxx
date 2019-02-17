/** \copyright
 *
 *    Copyright (C) 2019  Robert Heller D/B/A Deepwoods Software
 *			51 Locke Hill Road
 *			Wendell, MA 01379-9728
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
 * \file LinuxPWM.hxx
 *
 * Defines GPIO pins using the Linux sysfs ABI.
 * 
 * \section HOWTOUSE How to use                                        
 * TBD
 * 
 * @author Robert Heller
 * @date 10 October 2018
 */

#ifndef __LINUXPWM_HXX
#define __LINUXPWM_HXX

#include "freertos_drivers/common/PWM.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

template <int CHIP_NUM, int PIN_NUM> class LinuxPWM {
public:
    /// Number of the chip
    static constexpr const uint32_t CHIP = CHIP_NUM;
    /// Number of the pin
    static constexpr const uint32_t PIN  = PIN_NUM;
    
    /// Export pin
    static void export_pin()
    {
        char exportname[40];
        snprintf(exportname,sizeof(exportname),"/sys/class/pwm/pwmchip%d/export", CHIP);
        FILE *fp = fopen(exportname,"w");
        fprintf(fp,"%d\n",PIN);
        fclose(fp);
    }
    static void set_period(uint32_t nanos)
    {
        char periodname[60];
        snprintf(periodname,sizeof(periodname),"/sys/class/pwm/pwmchip%d/pwm%d/period",CHIP,PIN);
        FILE *fp = fopen(periodname,"w");
        fprintf(fp,"%d\n",nanos);
        fclose(fp);
    }
    static uint32_t get_period()
    {
        char periodname[60];
        uint32_t nanos;
        snprintf(periodname,sizeof(periodname),"/sys/class/pwm/pwmchip%d/pwm%d/period",CHIP,PIN);
        FILE *fp = fopen(periodname,"r");
        fscanf(fp,"%d",&nanos);
        fclose(fp);
        return nanos;
    }
    static void set_duty(uint32_t nanos)
    {
        char duty_cyclename[60];
        snprintf(duty_cyclename,sizeof(duty_cyclename),"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",CHIP,PIN);
        FILE *fp = fopen(duty_cyclename,"w");
        fprintf(fp,"%d\n",nanos);
        fclose(fp);
    }
    static uint32_t get_duty()
    {
        char duty_cyclename[60];
        uint32_t nanos;
        snprintf(duty_cyclename,sizeof(duty_cyclename),"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle",CHIP,PIN);
        FILE *fp = fopen(duty_cyclename,"r");
        fscanf(fp,"%d",&nanos);
        fclose(fp);
        return nanos;
    }
    static void enable() {
        char enablename[60];
        snprintf(enablename,sizeof(enablename),"/sys/class/pwm/pwmchip%d/pwm%d/enable",CHIP,PIN);
        int fd = open(enablename,O_WRONLY);
        write(fd,"1\n",2);
        close(fd);
    }
    static void disable() {
        char enablename[60];
        snprintf(enablename,sizeof(enablename),"/sys/class/pwm/pwmchip%d/pwm%d/enable",CHIP,PIN);
        int fd = open(enablename,O_WRONLY);
        write(fd,"0\n",2);
        close(fd);
    }
    static bool enabled() {
        char enablename[60], c;
        snprintf(enablename,sizeof(enablename),"/sys/class/pwm/pwmchip%d/pwm%d/enable",CHIP,PIN);
        int fd = open(enablename,O_RDONLY);
        read(fd,&c,1);
        close(fd);
        return (c == '1');
    }
    static uint32_t get_period_max() {return 0xffffffff;}
    static uint32_t get_period_min() {return 0;}
};

/// Creates an implementation of an os-independent PWM object from a
/// hardware-specific static PWM structure.

template <class PIN> class PwmWrapper : public PWM
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the data section.
    constexpr  PwmWrapper()
    {
    }
    
    void set_period(uint32_t counts)
    {
        PIN::set_period(counts);
    }
    uint32_t get_period()
    {
        return PIN::get_period();
    }
    void set_duty(uint32_t counts)
    {
        PIN::set_duty(counts);
    }
    uint32_t get_duty()
    {
        return PIN::get_duty();
    }
    uint32_t get_period_max()
    {
        return PIN::get_period_max();
    }
    uint32_t get_period_min()
    {
        return PIN::get_period_min();
    }
    void enable()
    {
        PIN::enable();
    }
    void disable()
    {
        PIN::disable();
    }
    bool enabled()
    {
        return PIN::enabled();
    }
    /// @return the static PWM object instance controlling this pin.
    static constexpr const PWM *instance()
    {
        return &instance_;
    }
    static PwmWrapper instance_;
};

template <class PIN> const PwmWrapper<PIN> PwmWrapper<PIN>::instance_;

template <class Base> struct PWMPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::export_pin();
        Base::disable();
        Base::set_period(0);
        Base::set_duty(0);
        Base::enable();
    }
    static void hw_set_to_safe()
    {
        Base::set_period(0);
        Base::set_duty(0);
        Base::enable();
    }
    static void set(bool unused) {}
    static void set_period(uint32_t nanos)
    {
        Base::set_period(nanos);
    }
    static uint32_t get_period()
    {
        return Base::get_period();
    }
    static void set_duty(uint32_t nanos)
    {
        Base::set_duty(nanos);
    }
    static uint32_t get_duty()
    {
        return Base::get_duty();
    }
    static void enable()
    {
        Base::enable();
    }
    static void disable()
    {
        Base::disable();
    }
    static bool enabled()
    {
        return Base::enabled();
    }
    static uint32_t get_period_max()
    {
        return Base::get_period_max();
    }
    static uint32_t get_period_min()
    {
        return Base::get_period_min();
    }
    /// @return the static PWM instance.
    static const PWM *instance()
    {
        return PwmWrapper<PWMPin<Base>>::instance();
    }
};

#define PWM_PIN(NAME, CHIP, NUM) \
    typedef PWMPin<LinuxPWM<CHIP, NUM>> NAME##_Pin

#endif // __LINUXPWM_HXX

