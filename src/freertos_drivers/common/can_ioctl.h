/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file can_ioctl.h
 * This file implements can specific ioctl() keys.
 *
 * @author Stuart W. Baker
 * @date 2 November 2013
 */

#ifndef _can_ioctl_h_
#define _can_ioctl_h_

#if defined (__cplusplus)
extern "C" {
#endif

/** ioctl() structure to setup the RX/TX active callback.
 */
typedef struct can_active_callback
{
    void (*callback)(void*); /**< application callback to call */
    void *context; /**< application callback argument to pass */
} CanActiveCallback;

/** Magic number for this driver's ioctl calls */
#define CAN_IOC_MAGIC ('c')

/** read active ioctl */
#define CAN_IOC_READ_ACTIVE IO(CAN_IOC_MAGIC, 1)

/** write active ioctl */
#define CAN_IOC_WRITE_ACTIVE IO(CAN_IOC_MAGIC, 2)

#if defined (__cplusplus)
}
#endif

#endif /* _can_ioctl_h_ */
