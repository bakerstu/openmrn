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
        fprintf(fp,"%d\n",channel_);
        fclose(fp);
        set_period(1);
    }       
    void set_period(uint32_t counts) override
    {
        char periodname[60];
        strcpy(periodname,pwmdir);
        strcat(periodname,"/period");
        FILE *fp = fopen(periodname,"w");
        fprintf(fp,"%d\n",counts);
        fclose(fp);
        if (counts > 0) {
            enable();
        } else {
            disable();
        }
    }
    uint32_t get_period() override
    {
        char periodname[60];
        uint32_t counts;
        strcpy(periodname,pwmdir);
        strcat(periodname,"/period");
        FILE *fp = fopen(periodname,"r");
        fscanf(fp,"%d",&counts);
        fclose(fp);
        return counts;
    }
    void set_duty(uint32_t counts) override
    {
        char duty_cyclename[60];
        strcpy(duty_cyclename,pwmdir);
        strcat(duty_cyclename,"/duty_cycle");
        FILE *fp = fopen(duty_cyclename,"w");
        fprintf(fp,"%d\n",counts);
        fclose(fp);
        if (counts > 0) {
            enable();
        } else {
            disable();
        }
    }
    uint32_t get_duty() override
    {
        char duty_cyclename[60];
        uint32_t counts;
        strcpy(duty_cyclename,pwmdir);
        strcat(duty_cyclename,"/duty_cycle");
        FILE *fp = fopen(duty_cyclename,"r");
        fscanf(fp,"%d",&counts);
        fclose(fp);
        return counts;
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
        char enablename[60];
        strcpy(enablename,pwmdir);
        strcat(enablename,"/enable");
        int fd = open(enablename,O_WRONLY);
        write(fd,"1\n",2);
        close(fd);
    }
    void disable() {
        char enablename[60];
        strcpy(enablename,pwmdir);
        strcat(enablename,"/enable");
        int fd = open(enablename,O_WRONLY);
        write(fd,"0\n",2);
        close(fd);
    }
    bool enabled() {
        char enablename[60], c;
        strcpy(enablename,pwmdir);
        strcat(enablename,"/enable");
        int fd = open(enablename,O_RDONLY);
        read(fd,&c,1);
        close(fd);
        return (c == '1');
    }
private:
    const uint32_t chip_;
    const uint32_t channel_;
    char pwmdir[40];
};
        


#endif // __LINUXPWM_HXX

