extern "C" {
#include "ets_sys.h"
#include <gpio.h>
#include <os_type.h>
#include <osapi.h>
#include <user_interface.h>
}

#include "os/os.h"

#include "utils/blinker.h"

static int blinkerpin = 0;

extern "C" {

void resetblink(uint32_t pattern)
{
    if (pattern)
    {
        gpio_output_set(0, (1 << blinkerpin), 0, 0);
    }
    else
    {
        gpio_output_set((1 << blinkerpin), 0, 0, 0);
    }
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void usleep(useconds_t sleep_usec)
{
    extern void ets_delay_us(uint32_t us);
    ets_delay_us(sleep_usec);
}

//static os_event_t appl_task_event[1];

void ICACHE_FLASH_ATTR appl_task(os_event_t *e)
{
    static int argc = 0;
    static char **argv = {0};
    appl_main(argc, argv);
}

void ICACHE_FLASH_ATTR user_init()
{
    // init gpio subsytem
    gpio_init();
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
    gpio_output_set(0, 0, (1 << blinkerpin), 0);

    // static os_task_t appl_task_struct;
    // system_os_task(appl_task, USER_TASK_PRIO_0, appl_task_event, 1);

    appl_task(nullptr);
}


}  // extern "C"
