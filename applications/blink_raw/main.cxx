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

#include "os/os.h"
#include "utils/blinker.h"
#include "console/Console.hxx"
#include "hardware.hxx"

#if (defined (TARGET_IS_CC3200) || defined (__linux__) || defined(PART_TM4C1294NCPDT)) && (!defined(NO_CONSOLE))
#define HAVE_CONSOLE
#endif

#ifdef HAVE_CONSOLE
Executor<1> executor("executor", 0, 2048);
#endif

class TinyLcd
{
public:
    void init()
    {
        usleep(2000);
        send_command(0x38);
        usleep(2000);
        send_command(0x28);
        usleep(2000);
        send_command(0x28);
        usleep(2000);
        send_command(0x0F);
        usleep(2000);
        send_command(0x01);
        usleep(2000);
        send_command(0x06);
    }

private:
    void send_nibble(uint8_t nibble)
    {
        LCD_D4_Pin::set(nibble & 0x1);
        LCD_D5_Pin::set(nibble & 0x2);
        LCD_D6_Pin::set(nibble & 0x4);
        LCD_D7_Pin::set(nibble & 0x8);
        LCD_E_Pin::set(true);
        usleep(2000);
        LCD_E_Pin::set(false);
    }

    void write(uint8_t value)
    {
        send_nibble(value);
        send_nibble(value << 4);
    }

    void send_command(uint8_t command)
    {
        LCD_RS_Pin::set(false);
        write(command);
    }
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    TinyLcd lcd;
    lcd.init();

    setblink(0);
#ifdef HAVE_CONSOLE
    new Console(&executor, Console::FD_STDIN, Console::FD_STDOUT, 2121);
#endif
    while (1)
    {
        setblink(1);
        usleep(500000);
        setblink(0);
        usleep(500000);
    }
    return 0;
}
