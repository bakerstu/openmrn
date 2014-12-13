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
#include "nmranet/CanDefs.hxx"
#include "nmranet/DatagramDefs.hxx"
#include "nmranet/MemoryConfig.hxx"
#include "can_frame.h"

namespace nmranet
{

struct BootloaderState
{
    struct can_frame input_frame;
    struct can_frame output_frame;
    unsigned input_frame_full : 1;
    unsigned output_frame_full : 1;
    unsigned request_reset : 1;
    NodeAlias alias;
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
    uint32_t post_size = app_header->app_size -
                         (reinterpret_cast<const uint8_t *>(app_header) -
                          static_cast<const uint8_t *>(flash_min)) -
                         sizeof(struct app_header);
    checksum_data(app_header + 1, post_size, checksum);
    if (memcmp(app_header->checksum_post, checksum, sizeof(checksum)))
    {
        return false;
    }
    return true;
}

void setup_can_frame()
{
    CLR_CAN_FRAME_RTR(state_.output_frame);
    CLR_CAN_FRAME_ERR(state_.output_frame);
    SET_CAN_FRAME_EFF(state_.output_frame);
    state_.output_frame.can_dlc = 0;
    state_.output_frame_full = 1;
}

void set_can_frame_global(Defs::MTI mti)
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
                        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
}

/** Sets the outgoing CAN frame to addressed, destination taken from the source
 * field of the incoming message. */
void set_can_frame_addressed(Defs::MTI mti)
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
                        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
    state_.output_frame.can_dlc = 2;
    uint32_t incoming_id = GET_CAN_FRAME_ID_EFF(state_.input_frame);
    NodeAlias incoming_alias = CanDefs::get_src(incoming_id);
    state_.output_frame.data[0] = (incoming_alias >> 8) & 0xf;
    state_.output_frame.data[1] = incoming_alias & 0xff;
}

/** Sets output frame dlc to 4; adds the given error code to bytes 2 and 3. */
void set_error_code(uint16_t error_code) {
    state_.output_frame.can_dlc = 4;
    state_.output_frame.data[2] = error_code >> 8;
    state_.output_frame.data[3] = error_code & 0xff;
}

void reject_datagram() {
    set_can_frame_addressed(Defs::MTI_DATAGRAM_REJECTED);
    set_error_code(Defs::ERROR_PERMANENT);
    state_.input_frame_full = 0;
}

void handle_memory_config_frame()
{
    uint8_t command = state_.input_frame.data[1];
    switch (command)
    {
        case MemoryConfigDefs::COMMAND_RESET:
        {
            set_can_frame_addressed(Defs::MTI_DATAGRAM_OK);
            state_.request_reset = 1;
            state_.input_frame_full = 0;
            return;
        }
    } // switch
    reject_datagram();
    return;
}

void handle_input_frame()
{
    if (IS_CAN_FRAME_ERR(state_.input_frame) ||
        IS_CAN_FRAME_RTR(state_.input_frame) ||
        !IS_CAN_FRAME_EFF(state_.input_frame))
    {
        state_.input_frame_full = 0;
        return;
    }
    uint32_t can_id = GET_CAN_FRAME_ID_EFF(state_.input_frame);
    int dlc = state_.input_frame.can_dlc;
    if ((can_id >> 12) == (0x1A000 | state_.alias) && dlc > 1)
    {
        // Datagram one frame.

        // Datagrams always need an answer. If we cannot render the answer,
        // let's not even try to parse the message.
        if (state_.output_frame_full) {
            return; // re-try.
        }
        if (state_.input_frame.data[0] == DatagramDefs::CONFIGURATION)
        {
            return handle_memory_config_frame();
        }
        else
        {
            reject_datagram();
            return;
        }
    }
    if (CanDefs::get_frame_type(can_id) == CanDefs::CONTROL_MSG)
    {
        // CAN control message. Ignore.
        state_.input_frame_full = 0;
        return;
    }
    // NMRAnet message
    state_.input_frame_full = 0;
    return;
}

void bootloader_entry()
{
    bootloader_hw_set_to_safe();
    bootloader_hw_init();
    if (!request_bootloader() && check_application_checksum())
    {
        return application_entry();
    }

    memset(&state_, 0, sizeof(state_));
    state_.alias = nmranet_alias();

    while (true)
    {
        if (!state_.input_frame_full && read_can_frame(&state_.input_frame))
        {
            state_.input_frame_full = 1;
        }
        if (state_.output_frame_full && try_send_can_frame(state_.output_frame))
        {
            state_.output_frame_full = 0;
        }
        if (state_.request_reset)
        {
            return bootloader_reboot();
        }
        if (state_.input_frame_full)
        {
            handle_input_frame();
        }

#ifdef __linux__
        usleep(10);
#endif
    } // while true
    try_send_can_frame(state_.output_frame);
}

} // extern "C"
