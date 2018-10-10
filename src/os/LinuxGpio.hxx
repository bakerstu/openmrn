// -!- c++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Tue Oct 9 21:19:14 2018
//  Last Modified : <181010.0947>
//
//  Description	
//
//  Notes
//
//  History
//

/** \copyright
 *
 *    Copyright (C) 2018  Robert Heller D/B/A Deepwoods Software
 *			51 Locke Hill Road
 *			Wendell, MA 01379-9728
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * 
 *
 *
 * \file LinuxGpio.hxx
 * 
 * @author Robert Heller
 * @date 10 October 2018
 */

#ifndef __LINUXGPIO_HXX
#define __LINUXGPIO_HXX

#include "os/Gpio.hxx"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

/**
 * Gpio wrapper for a Linux GPIO bit using the sysfs interface.
 * 
 * 
 */

class LinuxGpio : public Gpio
{
public:
    
    constexpr LinuxGpio(const unsigned pin, const bool is_output)
          : pin_(pin)
          , isOutput_(is_output ? 1 : 0)
    {
        char buffer[128];
        FILE *fp = fopen("/sys/class/gpio/export","w");
        fprintf(fp,"%d\n",pin_);
        fclose(fp);
        snprintf(buffer,sizeof(buffer),"/sys/class/gpio/gpio%d/direction",pin_);
        fp = fopen(buffer,"w");
        fprintf(fp,(isOutput_ == 1) ? "out\n" : "in\n");
        fclose(fp);
        snprintf(buffer,sizeof(buffer),"/sys/class/gpio/gpio%d/value",pin_);
        fd = open(buffer, O_RDWR);
    }
    void write(Value new_state) const OVERRIDE
    {
        if (new_state == Value::SET) {
            ::write(fd,"1\n",2);
        } else {
            ::write(fd,"0\n",2);
        }
    }
    void set() const OVERRIDE
    {
        write(Value::SET);
    }

    void clr() const OVERRIDE
    {
        write(Value::CLR);
    }

    Value read() const OVERRIDE
    {
        char c;
        lseek(fd, 0L, SEEK_SET) ;
        ::read(fd,&c,1);
        return (c == '0')? Value::CLR : Value::SET;
    }

    void set_direction(Direction dir) OVERRIDE
    {
        char buffer[128];
        isOutput_ = (dir == Direction::OUTPUT) ? 1 : 0;
        snprintf(buffer,sizeof(buffer),"/sys/class/gpio/gpio%d/direction",pin_);
        FILE *fp = fopen(buffer,"w");
        fprintf(fp,(isOutput_ == 1) ? "out\n" : "in\n");
        fclose(fp);
    }

    Direction direction() const OVERRIDE
    {
        return isOutput_ ? Direction::OUTPUT : Direction::INPUT;
    }

private:
    /// Which pin
    const unsigned pin_;
    /// 1 if this GPIO is an output, 0 if it's an input.
    unsigned isOutput_ : 1;
    /// /sysfs file destriptor.
    int fd;
};
#endif // __LINUXGPIO_HXX

