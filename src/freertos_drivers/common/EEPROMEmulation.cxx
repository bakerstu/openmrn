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
 * \file EEPROMEmulation.cxx
 * This file implements a generic EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 24 June 2015
 */

#include "EEPROMEmulation.hxx"

#include <cstring>

const size_t EEPROMEmulation::HEADER_BLOCK_COUNT = 3;

const uint32_t EEPROMEmulation::MAGIC_DIRTY = 0xaa55aa55;
const uint32_t EEPROMEmulation::MAGIC_INTACT = 0xaa558001;
const uint32_t EEPROMEmulation::MAGIC_USED = 0x00000000;
const uint32_t EEPROMEmulation::MAGIC_ERASED = 0xFFFFFFFF;

/** Constructor.
 * @param name device name
 * @param file_size maximum file size that we can grow to.
 */
EEPROMEmulation::EEPROMEmulation(const char *name, size_t file_size)
    : EEPROM(name, file_size)
    , shadow_in_ram(false)
    , shadow(nullptr)
    , activeIndex(0)
    , available(0)
{
    /* make sure we have an appropriate sized region of memory for our device */
    HASSERT(FLASH_SIZE >= (2 * SECTOR_SIZE));  // at least two of them
    HASSERT((FLASH_SIZE % SECTOR_SIZE) == 0);  // and nothing remaining
    HASSERT(file_size <= (SECTOR_SIZE >> 1));  // single block fit all the data
    HASSERT(file_size <= (1024 * 64 - 2));  // uint16 indexes, 0xffff reserved
    HASSERT(BLOCK_SIZE >= 4); // we don't support block sizes less than 4 bytes
    HASSERT((BLOCK_SIZE % 4) == 0); // block size must be on 4 byte boundary
}

/** Mount the EEPROM file.
 */
void EEPROMEmulation::mount()
{
    /* look for an active block that is not used up */
    for (int i = 0; i < sector_count(); ++i)
    {
        if (*block(MAGIC_DIRTY_INDEX,  sector(i)) == MAGIC_DIRTY  &&
            *block(MAGIC_INTACT_INDEX, sector(i)) == MAGIC_INTACT &&
            *block(MAGIC_USED_INDEX,   sector(i)) == MAGIC_ERASED )
        {
            activeIndex = i;
            break;
        }
    }

    if (*block(MAGIC_DIRTY_INDEX,  active()) != MAGIC_DIRTY  ||
        *block(MAGIC_INTACT_INDEX, active()) != MAGIC_INTACT ||
        *block(MAGIC_USED_INDEX,   active()) != MAGIC_ERASED )
    {
        /* our active block is corrupted, we are starting over */
        uint32_t data[4] = {MAGIC_DIRTY, 0, 0, 0};

        flash_erase(active());
        flash_program(data, block(0, active()), BLOCK_SIZE);

        data[0] = MAGIC_INTACT;
        flash_program(data, block(1, active()), BLOCK_SIZE);

        available = slot_count();
    }
    else
    {
        /* look for first data block */
        for (uint32_t *address = slot_last(active());
             address != magic_last(active());
             address -= (BLOCK_SIZE / sizeof(uint32_t)))
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
        /* copy EEPROM data into shadow ram */
        shadow = new uint8_t[file_size()];

        /* prime the shadow RAM with the EEPROM data */
        read(0, shadow, file_size());

        /* turn on shadowing */
        shadow_in_ram = true;
    }
}

/** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
 * BLOCK_SIZE boundaries in the case of power loss.  The user should take this
 * into account as it relates to data integrity of a whole block.
 * @ref index within EEPROM address space to start write
 * @ref buf data to write
 * @ref len length in bytes of data to write
 */
void EEPROMEmulation::write(unsigned int index, const void *buf, size_t len)
{
    HASSERT((index + len) <= file_size());

    uint8_t* byte_data = (uint8_t*)buf;

    if (shadow_in_ram)
    {
        memcpy(shadow + index, byte_data, len);
    }

    while (len)
    {
        /* get the least significant address bits */
        unsigned int lsa = index & (BYTES_PER_BLOCK - 1);
        if (lsa)
        {
            /* head, (unaligned) address */
            uint8_t data[BYTES_PER_BLOCK];
            size_t write_size = len < (BYTES_PER_BLOCK - lsa) ?
                                len : (BYTES_PER_BLOCK - lsa);
            read_block(index / BYTES_PER_BLOCK, data);

            if (memcmp(data + lsa, byte_data, write_size) != 0)
            {
                /* at least some data has changed */
                memcpy(data + lsa, byte_data, write_size);
                write_block(index / BYTES_PER_BLOCK, data);
            }

            index     += write_size;
            len       -= write_size;
            byte_data += write_size;
        }
        else if (len < BYTES_PER_BLOCK)
        {
            /* tail, (unaligned) address */
            uint8_t data[BYTES_PER_BLOCK];
            read_block(index / BYTES_PER_BLOCK, data);

            if (memcmp(data, byte_data, len) != 0)
            {
                /* at least some data has changed */
                memcpy(data, byte_data, len);
                write_block(index / BYTES_PER_BLOCK, data);
            }

            len = 0;
        }
        else
        {
            /* aligned data */
            uint8_t data[BYTES_PER_BLOCK];
            read_block(index / BYTES_PER_BLOCK, data);

            if (memcmp(data, byte_data, BYTES_PER_BLOCK) != 0)
            {
                /* at least some data has changed */
                memcpy(data, byte_data, BYTES_PER_BLOCK);
                write_block(index / BYTES_PER_BLOCK, data);
            }

            index     += BYTES_PER_BLOCK;
            len       -= BYTES_PER_BLOCK;
            byte_data += BYTES_PER_BLOCK;
        }
    }
}

/** Write to the EEPROM on a native block boundary.
 * @ref index block within EEPROM address space to write
 * @ref data data to write, array size must be @ref BYTES_PER_BLOCK large
 */
void EEPROMEmulation::write_block(unsigned int index, const uint8_t data[])
{
    if (available)
    {
        /* still have room in this block for at least one more write */
        uint32_t slot_data[BLOCK_SIZE / sizeof(uint32_t)];
        for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
        {
            slot_data[i] = (index << 16) | 
                           (data[(i * 2) + 1] << 8) |
                           (data[(i * 2) + 0] << 0);
        }
        uint32_t *address = block(MAGIC_COUNT + slot_count() - available,
                                  active());
        flash_program(slot_data, address, sizeof(slot_data));
        --available;
    }
    else
    {
        /* we need to overflow into the next block */
        uint32_t *new_block = next_active();
        uint32_t magic[4] = {MAGIC_DIRTY, 0, 0, 0};

        /* prep the new block */
        flash_erase(new_block);
        flash_program(magic,
                      block(MAGIC_DIRTY_INDEX, new_block),
                      BLOCK_SIZE);

        /* reset the available count */
        available = slot_count();

        uint32_t *address = new_block +
                            (MAGIC_COUNT * (BLOCK_SIZE / sizeof(uint32_t)));

        /* move any existing data over */
        for (unsigned int i = 0; i < (file_size() / BYTES_PER_BLOCK); ++i)
        {
            uint32_t slot_data[BLOCK_SIZE / sizeof(uint32_t)];
            if (i == index)
            {
                for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
                {
                    slot_data[i] = (index << 16) | 
                                   (data[(i * 2) + 1] << 8) |
                                   (data[(i * 2) + 0] << 0);
                }
            }
            else
            {
                /* this is old data we need to move over */
                uint8_t read_data[BYTES_PER_BLOCK];
                if (!read_block(i, read_data))
                {
                    /* nothing to write, this is the default "erased" value */
                    continue;
                }
                for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
                {
                    slot_data[i] = (index << 16) | 
                                   (read_data[(i * 2) + 1] << 8) |
                                   (read_data[(i * 2) + 0] << 0);
                }
            }
            /* commit the write */
            flash_program(slot_data, address, sizeof(slot_data));
            address += BLOCK_SIZE / sizeof(uint32_t);
            --available;
        }
        /* finalize the data move and write */
        magic[0] = MAGIC_INTACT;
        flash_program(magic, block(MAGIC_INTACT_INDEX, new_block), BLOCK_SIZE);
        magic[0] = MAGIC_USED;
        flash_program(magic, block(MAGIC_USED_INDEX, active()), BLOCK_SIZE);
        activeIndex = sector_index(new_block);
    }
}

/** Read from the EEPROM.
 * @ref index within EEPROM address space to start read
 * @ref buf location to post read data
 * @ref len length in bytes of data to read
 */
void EEPROMEmulation::read(unsigned int index, void *buf, size_t len)
{
    HASSERT((index + len) <= file_size());

    if (shadow_in_ram)
    {
        memcpy(buf, shadow + index, len);
    }
    else
    {
        uint8_t* byte_data = (uint8_t*)buf;

        while (len)
        {
            /* get the least significant address bits */
            unsigned int lsa = index & (BYTES_PER_BLOCK - 1);
            if (lsa)
            {
                /* head, (unaligned) address */
                uint8_t data[BYTES_PER_BLOCK];
                size_t read_size = len < (BYTES_PER_BLOCK - lsa) ?
                                   len : (BYTES_PER_BLOCK - lsa);

                read_block(index / BYTES_PER_BLOCK, data);
                memcpy(byte_data, data + lsa, read_size);

                index     += read_size;
                len       -= read_size;
                byte_data += read_size;
            }
            else if (len < BYTES_PER_BLOCK)
            {
                /* tail, (unaligned) address */
                uint8_t data[BYTES_PER_BLOCK];

                read_block(index / BYTES_PER_BLOCK, data);
                memcpy(byte_data, data, len);

                len = 0;
            }
            else
            {
                /* aligned data */
                uint8_t data[BYTES_PER_BLOCK];
                read_block(index / BYTES_PER_BLOCK, data);
                memcpy(byte_data, data, BYTES_PER_BLOCK);

                index     += BYTES_PER_BLOCK;
                len       -= BYTES_PER_BLOCK;
                byte_data += BYTES_PER_BLOCK;
            }
        }
    }
}

/** Read from the EEPROM on a native block boundary.
 * @ref index bock within EEPROM address space to read
 * @ref data location to place read data, array size must be @ref
 *           BYTES_PER_BLOCK large
 * @ref return true if any of the data is not "erased", else return false
 */
bool EEPROMEmulation::read_block(unsigned int index, uint8_t data[])
{
    if (shadow_in_ram)
    {
        memcpy(data, shadow + (index * BYTES_PER_BLOCK), BYTES_PER_BLOCK);
    }
    else
    {
        /* default data value if not found */
        memset(data, 0xFF, BYTES_PER_BLOCK);

        /* look for data */
        for (uint32_t *address = slot_last(active());
             address != magic_last(active());
             address -= (BLOCK_SIZE / sizeof(uint32_t)))
        {
            if (index == (*address >> 16))
            {
                /* found the data */
                for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
                {
                    data[(i * 2) + 0] = (address[i] >> 0) & 0xFF; 
                    data[(i * 2) + 1] = (address[i] >> 8) & 0xFF; 
                }
                return true;
            }
        }
    }

    return false;
}
