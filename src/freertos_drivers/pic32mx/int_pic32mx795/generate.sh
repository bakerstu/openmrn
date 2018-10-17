#!/bin/bash

rm -f *.S int_defines.h

while read NUMDEF NAME ; do

  cp ../interrupt.S_template $NAME.S
  echo FREERTOS_ISR_wrapper $NUMDEF,$NAME >> $NAME.S
  echo extern const unsigned '__attribute__((section("rodata")))' ${NAME}_vector_number";" >> int_defines.h

  
done <<EOT
_UART_1_VECTOR uart1_interrupt
_UART_2_VECTOR uart2_interrupt
_UART_3_VECTOR uart3_interrupt
_UART_4_VECTOR uart4_interrupt
_UART_5_VECTOR uart5_interrupt
_UART_6_VECTOR uart6_interrupt
_CAN_1_VECTOR can1_interrupt
_CAN_2_VECTOR can2_interrupt
_USB_1_VECTOR usb_interrupt
EOT
