#include "mbed.h"

DigitalOut d1(LED1);
DigitalOut d2(LED2);
DigitalOut d3(LED3);
DigitalOut d4(LED4);

static serial_t* stdio_serial;

//Serial st(USBTX, USBRX);

extern "C" {

extern serial_t stdio_uart;

void hw_init(void)
{
  // This is needed for proper bootup of the FreeRTOS scheduler.
  __disable_irq();

  d1 = 0;
  d2 = 1;
  d3 = 1;
  d4 = 0;

  init_stdio_serial();
  serial_putc(&stdio_uart, 'X');
  serial_putc(&stdio_uart, '\n');
  //st.printf("heeeeelo\n");

}

void send_stdio_serial_message(const char* data) {
  while (!*data) {
    serial_putc(stdio_serial, *data++);
  }
}

/** This function is called during boot.
    - after initializing BSS and DATA segments in memory
    - after the clock and PLL is setup but
    - BEFORE the static objects are initialized.
 */
void lowlevel_hw_init(void) {
  // Initializes the UART0 link that will allow us to send error messages to
  // the host even during boot time.
  stdio_serial = init_stdio_serial();
  send_stdio_serial_message("Foo.\n");
}

}
