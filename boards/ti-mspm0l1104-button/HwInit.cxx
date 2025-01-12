
#include <ti/devices/msp/msp.h>
// m0p/mspm0l130x.h>

#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "utils/blinker.h"
#include "freertos_drivers/ti/MSPM0SpiPixelStrip.hxx"

#define POWER_STARTUP_DELAY (16)

#define CPUCLK_FREQ 32000000

uint8_t pix[3] = {3, 3, 3};

SpiPixelStrip strip0(SPI0, 1, pix);

extern "C" {

void resetblink(uint32_t p)
{
    pix[0] = p ? 30 : 3;
    strip0.update_sync();
}

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

static const DL_SPI_Config gSPI_0_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_15,
    .bitOrder    = DL_SPI_BIT_ORDER_LSB_FIRST,
    .chipSelectPin = DL_SPI_CHIP_SELECT_NONE,
};

static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

void setup_spi() {
    DL_SPI_setClockConfig(SPI0, (DL_SPI_ClockConfig *) &gSPI_0_clockConfig);
    DL_SPI_init(SPI0, (DL_SPI_Config *) &gSPI_0_config);
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     500000 = (32000000)/((1 + 31) * 2)
     *
     * Desired bit time is 0.27 to 0.4 usec per bit, which is 3.7MHz to 2.5MHz.
     * That is 12.8 to 8.64 multiplier for a 32MHz processor. We aim for
     * multiplier 10, which is SCR of 4.
     *
     *     (32000000)/((1 + 4) * 2) = 3.2MHz. That is 0.312 usec per bit.
     *
     * A 0 bit will be 0.31us high 0.62us low.
     * A 1 bit will be 0.62us high 0.31us low.
     */
    DL_SPI_setBitRateSerialClockDivider(SPI0, 4);
    DL_SPI_enable(SPI0);
}

void hw_preinit(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_enablePower(GPIOA);

    DL_SPI_reset(SPI0);
    DL_SPI_enablePower(SPI0);
    
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

    setup_spi();

    /// Pinmux setup.
    DL_GPIO_initPeripheralOutputFunctionFeatures(
        IOMUX_PINCM19 /*PA18*/, IOMUX_PINCM19_PF_SPI0_PICO,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
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

