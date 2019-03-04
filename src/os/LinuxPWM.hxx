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
 * Defines PWM pins using the Linux sysfs ABI.
 * 
 * \section HOWTOUSE How to use
 * TBD
 * 
 * @author Robert Heller
 * @date 19 Feburary 2019
 */

#ifndef __LINUXPWM_HXX
#define __LINUXPWM_HXX

#include "freertos_drivers/common/PWM.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

class LinuxPWM : public PWM
{
public:
    LinuxPWM(int chip, int channel) : chip_(chip) , channel_(channel)
    {
        snprintf(pwmdir,sizeof(pwmdir),
                 "/sys/class/pwm/pwmchip%d/pwm%d",
                 chip_,channel_);
    }
    void exportPin()
    {
        char exportname[50];
        strcpy(exportname,pwmdir);
        strcat(exportname,"/period");
        if (access(exportname, F_OK) != -1)
        {
            set_period(1);
            return;
        }
        char *p = strrchr(exportname,'/');
        *p = '\0';
        p = strrchr(exportname,'/');
        p++;
        strcpy(p,"export");
        FILE *fp = fopen(exportname,"w");
        if (fp == NULL) {
            int err = errno;
            LOG_ERROR("Cannot open %s for write: %s\n",exportname,strerror(errno));
            exit(err);
        }
        fprintf(fp,"%d\n",channel_);
        fclose(fp);
        set_period(1);
    }       
    void set_period(uint32_t counts) override
    {
        set_sysfs_file_value("period",counts);
        if (counts > 0) {
            enable();
        } else {
            disable();
        }
    }
    uint32_t get_period() override
    {
        return get_sysfs_file_value("period");
    }
    void set_duty(uint32_t counts) override
    {
        set_sysfs_file_value("duty_cycle",counts);
        if (counts > 0) {
            enable();
        } else {
            disable();
        }
    }
    uint32_t get_duty() override
    {
        return get_sysfs_file_value("duty_cycle");
    }
    uint32_t get_period_max() override
    {
        return 0xffffffff;
    }
    uint32_t get_period_min() override
    {
        return 1;
    }
    void enable() {
        set_sysfs_file_value("enable",1);
    }
    void disable() {
        set_sysfs_file_value("enable",0);
    }
    bool enabled() {
        return get_sysfs_file_value("enable") == 1;
    }
private:
    const uint32_t chip_;
    const uint32_t channel_;
    char pwmdir[40];
    uint32_t get_sysfs_file_value(const char* basename) {
        char sysfname[60];
        strcpy(sysfname,pwmdir);
        strcat(sysfname,"/");
        strcat(sysfname,basename);
        uint32_t value;
        FILE *fp = fopen(sysfname,"r");
        if (fp == NULL) {
            int err = errno;
            LOG_ERROR("Cannot open %s for read: %s\n",sysfname,strerror(errno));
            exit(err);
        }
        fscanf(fp,"%d",&value);
        fclose(fp);
        return value;
    }
    void set_sysfs_file_value(const char* basename, uint32_t value) {
        char sysfname[60];
        strcpy(sysfname,pwmdir);
        strcat(sysfname,"/");
        strcat(sysfname,basename);
        FILE *fp = fopen(sysfname,"w");
        if (fp == NULL) {
            int err = errno;
            LOG_ERROR("Cannot open %s for write: %s\n",sysfname,strerror(errno));
            exit(err);
        }
        fprintf(fp,"%d\n",value);
        fclose(fp);
    }
};
        


#endif // __LINUXPWM_HXX

