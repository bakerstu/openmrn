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
 * \file Bootloader.cxx
 *
 * A standalone NMRAnet stack with the sole purpose of reflashing a
 * microcontroller.
 *
 * @author Balazs Racz
 * @date 8 Dec 2014
 */

#include <string.h>
#include <unistd.h>

#include "freertos/bootloader_hal.h"
#include "nmranet/Defs.hxx"
#include "can_frame.h"

namespace nmranet
{

struct BootloaderState
{
    struct can_frame input_frame;
    struct can_frame output_frame;
    unsigned input_frame_full : 1;
    unsigned output_frame_full : 1;
} state_;
}
using namespace nmranet;

extern "C" {

/** @returns true if the application checksum currently in flash is correct. */
bool check_application_checksum()
{
    uint32_t checksum[CHECKSUM_COUNT];
    const void *flash_min;
    const void *flash_max;
    const struct app_header *app_header;
    get_flash_boundaries(&flash_min, &flash_max, &app_header);
    uint32_t pre_size = reinterpret_cast<const uint8_t *>(app_header) -
                        static_cast<const uint8_t *>(flash_min);
    checksum_data(flash_min, pre_size, checksum);
    if (memcmp(app_header->checksum_pre, checksum, sizeof(checksum)))
    {
        return false;
    }
    uint32_t post_size =
        app_header->app_size - (reinterpret_cast<const uint8_t *>(app_header) -
                                static_cast<const uint8_t *>(flash_min)) -
        sizeof(struct app_header);
    checksum_data(app_header + 1, post_size, checksum);
    if (memcmp(app_header->checksum_post, checksum, sizeof(checksum)))
    {
        return false;
    }
    return true;
}

void bootloader_entry()
{
    bootloader_hw_set_to_safe();
    bootloader_hw_init();
    if (!request_bootloader() && check_application_checksum())
    {
        return application_entry();
    }

    while (!read_can_frame(&state_.input_frame)) {
        usleep(10);
    }
    state_.output_frame = state_.input_frame;
    state_.output_frame.data[state_.output_frame.can_dlc++] = 0x55;
    try_send_can_frame(state_.output_frame);
}

} // extern "C"
