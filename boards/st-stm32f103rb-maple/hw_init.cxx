#include "FreeRTOSConfig.h"
#include "os/OS.hxx"

#include <wirish/wirish.h>

extern "C" {

void setblink(uint32_t pattern);

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain()
{
    init();
    setblink(0);
}

void hw_init(void)
{
    // This is needed for proper bootup of the FreeRTOS scheduler.
    nvic_globalirq_disable();

    /* Set up the LED to blink  */
    //pinMode(BOARD_LED_PIN, OUTPUT);
}

uint32_t blinker_pattern;
uint32_t current_pattern;
#define BLINK_GPIO LPC_GPIO1
#define BLINK_BIT (1 << 23)

extern "C" {
extern void vPortClearInterruptMask( unsigned long ulNewMaskValue );
extern unsigned long ulPortSetInterruptMask( void );
}

void enable_fiq_only(void)
{
    /** @TODO(balazs.racz): we should call vPortClearInterruptMask(14) here,
     * assuming the blink-timer interrupt is set to priority 15. The code here
     * will block all FreeRTOS functions though. */
    ulPortSetInterruptMask();
    nvic_globalirq_enable();
}

void diewith(unsigned long);

// Used by libmaple
void _fail(const char* file, int line, const char* exp) {
    diewith(0x80CCACCA);
}



/** Initializes the timer responsible for the blinking hardware and sets
 *   initial pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 */
void setblink(uint32_t pattern)
{
/*    blinker_pattern = pattern;
    current_pattern = 0;
    BLINK_GPIO->FIODIR |= BLINK_BIT;
    BLINK_GPIO->FIOCLR = BLINK_BIT;

    // clock = raw clock
    LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & (~(0x3 << 4))) | (0x01 << 4);
    LPC_TIM1->TCR = 2;  // stop & reset timer
    LPC_TIM1->CTCR = 0; // timer mode
    // prescale to 1 ms per tick
    LPC_TIM1->PR = configCPU_CLOCK_HZ / 1000;
    LPC_TIM1->MR0 = 125;
    LPC_TIM1->MCR = 3; // reset and interrupt on match 0

    NVIC_SetPriority(TIMER1_IRQn, 0);
    NVIC_EnableIRQ(TIMER1_IRQn);

    LPC_TIM1->TCR = 1; // Timer go.*/
}

/** Updates the blinking pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 */
void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    // Makes a timer event trigger immediately.
    //LPC_TIM1->TC = LPC_TIM1->MR0 - 2;
}

/** Aborts the program execution and sets a particular blinking pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 *
 *  Never returns.
 */
void diewith(unsigned long pattern)
{
    enable_fiq_only();
    setblink(pattern);
    for (;;)
    {
    }
}

void TIMER1_IRQHandler(void)
{
    if (current_pattern & 1)
    {
//        BLINK_GPIO->FIOSET = BLINK_BIT;
    }
    else
    {
//     BLINK_GPIO->FIOCLR = BLINK_BIT;
    }
    current_pattern >>= 1;
    if (!current_pattern)
    {
        current_pattern = blinker_pattern;
    }

    // Clears IRQ.
//    LPC_TIM1->IR = 1;
//    NVIC_ClearPendingIRQ(TIMER1_IRQn);
}
}
