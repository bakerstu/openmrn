
#include <ti/devices/msp/msp.h>
// m0p/mspm0l130x.h>

#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "utils/blinker.h"

#define POWER_STARTUP_DELAY (16)

#define CPUCLK_FREQ 32000000

extern "C" {

void resetblink(uint32_t)
{ }
void setblink(uint32_t p)
{
    resetblink(p);
}

void diewith(uint32_t)
{
    while (1)
        ;
}

extern uint64_t g_time_msec;
uint64_t g_time_msec = 0;

extern uint64_t current_time_msec()
{
    return g_time_msec;
}

void hw_set_to_safe(void)
{ }

void hw_preinit(void)
{
    DL_GPIO_reset(GPIOA);

    DL_GPIO_enablePower(GPIOA);
    delay_cycles(POWER_STARTUP_DELAY);

    // SYSCFG_DL_GPIO_init(); // empty

    /* Module-Specific Initializations*/
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

    // Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    /*
     * Initializes the SysTick period to 1 ms,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(CPUCLK_FREQ / 1000);
}

extern int appl_main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    return appl_main(argc, argv);
}

// These are usually defined by freertos, but we are running in no-OS mode due
// to the small flash.

void SVC_Handler(void)
{ }

void SysTick_Handler(void)
{
    g_time_msec++;
}

void PendSV_Handler(void)
{ }

void usleep(uint32_t usecs)
{
    auto end = current_time_msec() + (usecs + 999) / 1000;
    while (current_time_msec() < end) {
        __WFI();
    }
}

// This is needed to avoid pulling in os.o, which has a dependency on freertos.
void ignore_fn() {}

} // extern C

