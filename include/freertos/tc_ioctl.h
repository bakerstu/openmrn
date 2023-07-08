/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file tc_ioctl.h
 * This file implements can specific ioctl() keys for serial devices.
 *
 * @author Balazs Racz
 * @date 28 Feb 2018
 */

#ifndef _FREERTOS_TC_IOCTL_H_
#define _FREERTOS_TC_IOCTL_H_

#include "freertos/stropts.h"

/** Magic number for this driver's ioctl calls */
#define TERMIOS_IOC_MAGIC ('T')

#define TCSBRK      IO(TERMIOS_IOC_MAGIC, 9)

#define TCPARNONE   IO(TERMIOS_IOC_MAGIC, 0xF0)
#define TCPARODD    IO(TERMIOS_IOC_MAGIC, 0xF1)
#define TCPAREVEN   IO(TERMIOS_IOC_MAGIC, 0xF2)
#define TCPARONE    IO(TERMIOS_IOC_MAGIC, 0xF3)
#define TCPARZERO   IO(TERMIOS_IOC_MAGIC, 0xF4)
/// Use 9-bit reception mode
#define TCNINEBITRX IO(TERMIOS_IOC_MAGIC, 0xF5)
/// One stop bit
#define TCSTOPONE   IO(TERMIOS_IOC_MAGIC, 0xF8) 
/// Two stop bits
#define TCSTOPTWO   IO(TERMIOS_IOC_MAGIC, 0xF9)

/// Argument is a Notifiable* pointer. This notifiable will be invoked when all
/// bytes have completed transferring and the transmit engine is idle.
#define TCDRAINNOTIFY   IOW(TERMIOS_IOC_MAGIC, 0xE0, 4)

/// Argument is the desired baud rate for the port.
#define TCBAUDRATE   IOW(TERMIOS_IOC_MAGIC, 0xE1, 4)

#endif // _FREERTOS_TC_IOCTL_H_

