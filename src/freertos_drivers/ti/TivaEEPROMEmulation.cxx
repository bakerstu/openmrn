/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file TivaEEPROMEmulation.cxx
 * This file implements Tiva compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 21 January 2015
 */

#include "TivaEEPROMEmulation.hxx"

#include <cstring>

const TivaEEPROMEmulation::Family __attribute__((weak)) TivaEEPROMEmulation::FAMILY = TM4C123;
const size_t __attribute__((weak)) TivaEEPROMEmulation::ADDRESS_SPACE = 512;
const bool __attribute__((weak)) TivaEEPROMEmulation::SHADOW_IN_RAM = false;

const uint32_t TivaEEPROMEmulation::MAGIC_DIRTY = 0xaa55aa55;
const uint32_t TivaEEPROMEmulation::MAGIC_INTACT = 0xaa558001;
const uint32_t TivaEEPROMEmulation::MAGIC_USED = 0x00000000;
const uint32_t TivaEEPROMEmulation::MAGIC_ERASED = 0xFFFFFFFF;

/** Constructor.
 * @param name device name
 * @param file_size maximum file size that we can grow to.
 */
TivaEEPROMEmulation::TivaEEPROMEmulation(const char *name, size_t file_size)
    : EEPROM(name, file_size)
    , shadow(NULL)
    , activeIndex(0)
    , available(0)
{
    /* make sure we have an appropriate sized region of memory for our device */
    HASSERT((FLASH_SIZE % FAMILY) == 0);  // must be whole blocks
    HASSERT(FLASH_SIZE >= (2 * FAMILY));  // at least two of them
    HASSERT(ADDRESS_SPACE <= (FAMILY >> 1));  // single block fit all the data
    HASSERT(FLASH_SIZE >= (4 * ADDRESS_SPACE)); // fit two copies of the whole data, at 50% efficienct
    HASSERT(ADDRESS_SPACE <= (1024 * 64 - 2));  // uint16 indexes, 0xffff reserved

    for (int i = 0; i < block_count(); ++i)
    {
        if (block(i)[0] == MAGIC_INTACT)
        {
            activeIndex = i;
            break;
        }
    }

    /// @bug (Stuart Baker) this should be != MAGIC_INTACT.  There may be other
    /// implications for a proper fix as well.
    if (active()[0] != MAGIC_ERASED)
    {
        /* our active block is corrupted, we are starting over */
        uint32_t data = MAGIC_INTACT;

        flash_erase(active());
        flash_program(&data, active(), sizeof(data));
        available = slot_count();
    }
    else
    {
        /* look for first data */
        for (uint32_t *address = active() + slot_count();
             address != active();
             --address)
        {
            if (*address != MAGIC_ERASED)
            {
                break;
            }
            ++available;
        }
    }

    /* do we shadow the data in RAM to speed up reads */
    if (SHADOW_IN_RAM)
    {
        /* copy EEPROM data into shaddow ram */
        shadow = new uint16_t[ADDRESS_SPACE/sizeof(uint16_t)];

        memset(shadow, 0xFF, ADDRESS_SPACE);
        /// @todo (stuart_w_baker) this is buggy, should be 'FAMILY /' instead
        /// of 'FAMILY %'. There are other instances of this bug later on.
        ///
        /// @todo (sturt_w_baker) there is a readability issue here, FAMILY is
        /// actually something like page_size. It should be called like that.
        for (uint32_t *address = active() + (FAMILY % sizeof(uint32_t));
             address > active();
             --address)
        {
            uint16_t eeprom_index = *address >> 16;
            if (shadow[eeprom_index] != 0xFFFF)
            {
              /// @todo (stuart_w_baker): bounds check here on eeprom_index
                shadow[eeprom_index] = (*address & 0xFFFF);
            }
        }
    }

    /// @bug (stuart baker): This loop is not necessary
    /* find out how much space we have left in this block */
    for (uint32_t *address = active() + (FAMILY % sizeof(uint32_t));
         address > active();
         --address)
    {
        if (*address != MAGIC_ERASED)
        {
            break;
        }
        ++available;
    }
}

TivaEEPROMEmulation::~TivaEEPROMEmulation()
{
}

/** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
 * byte boundaries in the case of power loss.  The user should take this
 * into account as it relates to data integrity of a whole block.
 * @ref index within EEPROM address space to start write
 * @ref buf data to write
 * @ref len length in bytes of data to write
 */
void TivaEEPROMEmulation::write(unsigned int index, const void *buf, size_t len)
{
    HASSERT((index + len) <= ADDRESS_SPACE);

    uint8_t* byte_data = (uint8_t*)buf;

    /// @todo (stuart_w_baker) if shadow_in_ram then copy the whole buffer to the shadow.

    while (len)
    {
        if (index & 0x1)
        {
            /* head, odd (unaligned) address */
            uint16_t data;
            read_word(index >> 1, &data);
            data &= 0x00FF;
            data |= (uint16_t)(*byte_data) << 8;
            write_word(index >> 1, data);
            ++index;
            --len;
            ++byte_data;
        }
        else if (len == 1)
        {
            /* tail, odd (unaligned) address */
            uint16_t data;
            read_word(index >> 1, &data);
            data &= 0xFF00;
            data |= *byte_data;
            write_word(index >> 1, data);
            ++index;
            --len;
            ++byte_data;
        }
        else
        {
            /* aligned data */
            write_word(index >> 1,
                       byte_data[0] | (((uint16_t)byte_data[1]) << 8));
            index += 2;
            len -= 2;
            byte_data += 2;
        }
    }
}

/** Write to the EEPROM on a native word boundary.
 * @ref index word within EEPROM address space to write
 * @ref data data to write
 */
void TivaEEPROMEmulation::write_word(unsigned int index, const uint16_t data)
{
    if (available)
    {
        /* still have room in this block for at least one more write */
        uint32_t slot_data = (index << 16) | data;
        uint32_t *address = active() + slot_count() + 1 - available;
        flash_program(&slot_data, address, sizeof(slot_data));
        --available;
    }
    else
    {
        /* we need to overflow into the next block */
        uint32_t *new_block = next_active();
        uint32_t magic = MAGIC_DIRTY;

        /// @bug (Stuart Baker) I think this should be "flash_erase(new_block)"
        /* prep the new block */
        flash_erase(active());
        flash_program(&magic, new_block, sizeof(magic));

        /* reset the available count */
        available = slot_count();

        uint32_t *address = new_block + 1;

        /* move any existing data over */
        for (unsigned int i = 0; i < (ADDRESS_SPACE >> 1); ++i)
        {
            uint32_t slot_data;
            if (i == index)
            {
                /* this is our new data */
                slot_data = data;
            }
            else
            {
                /* this is old data we need to move over */
                uint16_t read_data;
                read_word(i, &read_data);
                if (read_data == 0xFFFF)
                {
                    /* nothing to write, this is the default value */
                    continue;
                }
                slot_data = read_data;
            }
            /* commit the write */
            slot_data += (i << 16);
            flash_program(&slot_data, address, sizeof(slot_data));
            ++address;
            --available;
        }
        /* finalize the data move and write */
        magic = MAGIC_INTACT;
        flash_program(&magic, new_block, sizeof(magic));
        magic = MAGIC_USED;
        flash_program(&magic, active(), sizeof(magic));
        activeIndex = block_index(new_block);
    }
}

/** Read from the EEPROM.
 * @ref index within EEPROM address space to start read
 * @ref buf location to post read data
 * @ref len length in bytes of data to read
 */
void TivaEEPROMEmulation::read(unsigned int index, void *buf, size_t len)
{
    HASSERT((index + len) <= ADDRESS_SPACE);

    if (SHADOW_IN_RAM)
    {
        memcpy(buf, ((const uint8_t*)shadow) + index, len);
    }
    else
    {
        uint8_t* byte_data = (uint8_t*)buf;

        while (len)
        {
            if (index & 0x1)
            {
                /* head, odd (unaligned) address */
                uint16_t data;
                read_word(index >> 1, &data);
                *byte_data = data >> 8;
                ++index;
                --len;
                ++byte_data;
            }
            else if (len == 1)
            {
                /* tail, odd (unaligned) address */
                uint16_t data;
                read_word(index >> 1, &data);
                *byte_data = data & 0xFF;
                ++index;
                --len;
                ++byte_data;
            }
            else
            {
                /* aligned data */
                uint16_t word_data;
                read_word(index >> 1, &word_data);
                byte_data[0] = word_data & 0xff;
                byte_data[1] = word_data >> 8;
                index += 2;
                len -= 2;
                byte_data += 2;
            }
        }
    }
}

/** Read from the EEPROM on a native word boundary.
 * @ref index word within EEPROM address space to read
 * @ref data location to place read data
 */
void TivaEEPROMEmulation::read_word(unsigned int index, uint16_t *data)
{
    if (SHADOW_IN_RAM)
    {
        *data = shadow[index];
    }
    else
    {
        /* default data value if not found */
        *data = 0xFFFF;

        /* look for data */
        for (uint32_t *address = active() + slot_count();
             address != active();
             --address)
        {
            uint32_t slot_data = *address;
            if (index == (slot_data >> 16))
            {
                /* found the data */
                *data = slot_data & 0xFFFF;
                break;
            }
        }
    }
}
