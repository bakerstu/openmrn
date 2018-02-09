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
#include "driverlib/sysctl.h"

#if (defined (TARGET_IS_CC3200) || defined (__linux__) || defined(PART_TM4C1294NCPDT)) && (!defined(NO_CONSOLE))
#define HAVE_CONSOLE
#endif

#ifdef HAVE_CONSOLE
Executor<1> executor("executor", 0, 2048);
#endif

inline void ClockDelay(unsigned d) {
    MAP_SysCtlDelay(d);
}

template<class HW> class ST7066UDisplay : public HW {
public:
    ST7066UDisplay() {}

    void init() {
        HW::RS_Pin::set(false);
        usleep(45000);
        set_nibble(0x3);
        usleep(5000);
        toggle_e();
        usleep(2000);
        toggle_e();
        usleep(2000);
        toggle_e();
        delay_37_usec();

        set_nibble(0x2);
        toggle_e();
        delay_37_usec();

        command(0x28);
        command(0x14);
        command(0x0F);
        command(0x06);
        clear();
        home();
    }

    /// Clears the screen.
    void clear() {
        command(CLEAR_SCREEN);
        usleep(2000);
    }

    /// Clears the DDRAM address counter and moves cursor to original position.
    void home() {
        command(HOME);
        usleep(2000);
    }

    /// @param display_on whether the display should be turned on
    /// @param cursor_on whether a cursor should be displayed
    /// @param blinking_on whether the cursor should be blinking
    void set_on_off(bool display_on, bool cursor_on, bool blinking_on) {
        uint8_t cmd = SET_DISPLAY_ON;
        if (display_on) cmd |= DISP_ON_DISPLAY;
        if (cursor_on) cmd |= DISP_ON_CURSOR;
        if (blinking_on) cmd |= DISP_ON_BLINKING;
        command(cmd);
    }

    /// Sets the display RAM address pointer.
    /// @param address desired address.
    void set_ac(uint8_t address) {
        command(SET_DD_ADDRESS | (address & 0x7F));
    }

    /// Sets the character generation RAM address pointer.
    /// @param address desired address.
    void set_cg_ac(uint8_t address) {
        command(SET_CG_ADDRESS | (address & 0x3F));
    }

    void write_text(uint8_t address, const char* data, size_t num) {
        set_ac(address);
        for (unsigned i = 0; i < num; ++i) {
            write_data(data[i]);
        }
    }

    void write_text(uint8_t address, const char* data) {
        write_text(address, data, strlen(data));
    }
    
private:
    enum Commands
    {
        CLEAR_SCREEN = 0x1,
        HOME = 0x2,
        SET_ENTRY_MODE = 0x4,
        SET_DISPLAY_ON = 0x8,
        DISP_ON_DISPLAY = 0x4,
        DISP_ON_CURSOR = 0x2,
        DISP_ON_BLINKING = 0x1,

        SET_CURSOR_SHIFT = 0x10,
        SET_FUNCTION = 0x20,
        SET_CG_ADDRESS = 0x40,
        SET_DD_ADDRESS = 0x80
    };

    /// Takes the low 4 bits and outputs it to the data port.
    void set_nibble(uint8_t d)
    {
        HW::D4_Pin::set(d & 1);
        HW::D5_Pin::set(d & 2);
        HW::D6_Pin::set(d & 4);
        HW::D7_Pin::set(d & 8);
    }

    /// Toggles the E bit
    void toggle_e() {
        HW::E_Pin::set(true);
        ClockDelay(40); // about 1.5 usec, minimum 0.3 usec.
        HW::E_Pin::set(false);
        ClockDelay(40); // about 1.5 usec, minimum 10 nsec
    }

    void write_byte(uint8_t data) {
        set_nibble(data >> 4);
        toggle_e();
        set_nibble(data);
        toggle_e();
    }

    void command(uint8_t cmd) {
        HW::RS_Pin::set(false);
        write_byte(cmd);
        delay_37_usec();
    }

    void write_data(uint8_t p) {
        HW::RS_Pin::set(true);
        write_byte(p);
        delay_37_usec();
    }

    void delay_37_usec() {
        ClockDelay(80*37);
    }
};

struct DisplayHW {
    typedef ::LCD_E_Pin E_Pin;
    typedef ::LCD_RS_Pin RS_Pin;
    typedef ::D4_Pin D4_Pin;
    typedef ::D5_Pin D5_Pin;
    typedef ::D6_Pin D6_Pin;
    typedef ::D7_Pin D7_Pin;
};

ST7066UDisplay<DisplayHW> display;

void display_test() {
    display.init();
    display.set_on_off(true, false, false);
    display.write_text(0, "hello");
    display.write_text(0x40, "world");
    while(1);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    setblink(0);
    
#ifdef HAVE_CONSOLE
    new Console(&executor, Console::FD_STDIN, Console::FD_STDOUT, 2121);
#endif
    usleep(1000000);
    display_test();
    while (1)
    {
        resetblink(1);
        usleep(500000);
        resetblink(0);
        usleep(500000);
    }
    return 0;
}
