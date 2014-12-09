/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file bootloader_hal.hxx
 *
 * Hardware support functions for bootloader operation.
 *
 * @author Balazs Racz
 * @date 8 Dec 2014
 */

#ifdef __cplusplus
extern "C" {
#endif
 
/** Initializes the hardware to a safe state of the outputs. This function may
 *  not assume anything about previous hardware state. Also the memory layout
 *  is not yet initialized. */
extern void hw_set_to_safe();

/** Called after hw_set_to_safe and after the bss and data segments are
 *  initialized. Initializes the processor state, CAN hardware etc. */
extern void hw_init();

/** @Returns true if the hardware state requests entry to the bootloader. This
 *  will typically read a GPIO pin for a bootloader switch. This function will
 *  run after hw_init. */
extern bool request_bootloader();

/** Checks if there is an incoming CAN frame from the hardware.
 *
 * @param frame will be loaded with the incoming frame.
 *
 * @returns true if a frame has arrived, false if no frame was loaded into
 * frame. */
extern bool read_can_frame(struct can_frame* frame);

/** Tries to send a can frame.
 *
 * @param frame is the frame to send.
 *
 * @returns true if the frame was sent, false if the hardware buffer was busy
 * and the operation should be re-tried later. */
extern bool try_send_can_frame(struct can_frame& frame);

#ifdef __cplusplus
}
#endif
