
#include <ti/devices/msp/msp.h>
// m0p/mspm0l130x.h>

#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

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
}

int main(void)
{
    while (1)
        ;
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

} // extern C

extern uint64_t g_time_msec = 0;

extern uint64_t current_time_msec()
{
    return g_time_msec;
}
