#include "mbed.h"
#include "utils/Hub.hxx"

#include "FreeRTOSConfig.h"

#include "os/OS.hxx"

const unsigned long cm3_cpu_clock_hz = 96000000;

extern DigitalOut d2;

DigitalOut d1(LED1);
DigitalOut d2(LED2);
DigitalOut d3(LED3);
DigitalOut d4(LED4);

static serial_t* stdio_serial;

//Serial st(USBTX, USBRX);

extern "C" {

extern serial_t stdio_uart;

void send_stdio_serial_message(const char* data);

void hw_init(void)
{
  // This is needed for proper bootup of the FreeRTOS scheduler.
  __disable_irq();

  d1 = 0;
  d2 = 1;
  d3 = 1;
  d4 = 0;
}

void send_stdio_serial_message(const char* data) {
  while (*data) {
    serial_putc(stdio_serial, *data++);
  }
}

class StdOutPipeMember : public HubPortInterface
{
public:
  void send(Buffer<HubData>* b, unsigned) override {
    const char* cb = (const char*)(b->data()->data());
    for (size_t i = 0; i < b->data()->size(); i++) {
      serial_putc(stdio_serial, cb[i]);
    }
    serial_putc(stdio_serial, '\n');
    b->unref();
  }
};

StdOutPipeMember stdout_pipem;
HubPortInterface* stdout_pipmember = &stdout_pipem;


void setblink(uint32_t pattern);

/** This function is called during boot.
    - after initializing BSS and DATA segments in memory
    - after the clock and PLL is setup but
    - BEFORE the static objects are initialized.
 */
void lowlevel_hw_init(void) {
  // Initializes the blinker routine.
  setblink(0);
  // Initializes the UART0 link that will allow us to send error messages to
  // the host even during boot time.
  stdio_serial = init_stdio_serial();
}

uint32_t blinker_pattern;
uint32_t current_pattern;
#define BLINK_GPIO LPC_GPIO1
#define BLINK_BIT (1<<23)

void enable_fiq_only(void) {
  __set_BASEPRI(1);
  __enable_irq();
}

void diewith(unsigned long);

/** Initializes the timer responsible for the blinking hardware and sets
 *   initial pattern.
 *
 *  \param pattern is a blinking pattern, each bit will be shifted to the
 *  output LED every 125 ms.
 */
void setblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    current_pattern = 0;
    BLINK_GPIO->FIODIR |= BLINK_BIT;
    BLINK_GPIO->FIOCLR = BLINK_BIT;

    // clock = raw clock
    LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & (~(0x3<<4))) | (0x01 << 4);
    LPC_TIM1->TCR = 2;  // stop & reset timer
    LPC_TIM1->CTCR = 0; // timer mode
    // prescale to 1 ms per tick
    LPC_TIM1->PR = configCPU_CLOCK_HZ / 1000;
    LPC_TIM1->MR0 = 125;
    LPC_TIM1->MCR = 3;  // reset and interrupt on match 0

    NVIC_SetPriority(TIMER1_IRQn, 0);
    NVIC_EnableIRQ(TIMER1_IRQn);

    LPC_TIM1->TCR = 1;  // Timer go.
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
    LPC_TIM1->TC = LPC_TIM1->MR0 - 2;
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
    send_stdio_serial_message("Diewith called.\n");
    for (;;)
    {
    }
}

void TIMER1_IRQHandler(void) {
  if (current_pattern & 1) {
    BLINK_GPIO->FIOSET = BLINK_BIT;
  } else {
    BLINK_GPIO->FIOCLR = BLINK_BIT;
  }
  current_pattern >>= 1;
  if (!current_pattern) {
    current_pattern = blinker_pattern;
  }

  // Clears IRQ.
  LPC_TIM1->IR = 1;
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
}

}
