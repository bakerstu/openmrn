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
{
    /* make sure we have an appropriate sized region of memory for our device */
    HASSERT(EEPROMEMU_FLASH_SIZE >= (2 * SECTOR_SIZE));  // at least two of them
    HASSERT((EEPROMEMU_FLASH_SIZE % SECTOR_SIZE) == 0);  // and nothing remaining
    HASSERT(file_size <= (SECTOR_SIZE >> 1));  // single block fit all the data
    HASSERT(file_size <= (1024 * 64 - 2));  // uint16 indexes, 0xffff reserved
    HASSERT(BLOCK_SIZE >= 4); // we don't support block sizes less than 4 bytes
    HASSERT(BLOCK_SIZE <= MAX_BLOCK_SIZE); // this is how big our buffers are.
    HASSERT((BLOCK_SIZE % 4) == 0); // block size must be on 4 byte boundary
}

/** Mount the EEPROM file.
 */
void EEPROMEmulation::mount()
{
    /* look for an active block that is not used up */
    for (unsigned i = 0; i < sector_count(); ++i)
    {
        if (*block(i, MAGIC_DIRTY_INDEX) == MAGIC_DIRTY  &&
            *block(i, MAGIC_INTACT_INDEX) == MAGIC_INTACT &&
            *block(i, MAGIC_USED_INDEX) == MAGIC_ERASED )
        {
            activeSector_ = i;
            break;
        }
    }

    if (*block(activeSector_, MAGIC_DIRTY_INDEX) != MAGIC_DIRTY  ||
        *block(activeSector_, MAGIC_INTACT_INDEX) != MAGIC_INTACT ||
        *block(activeSector_, MAGIC_USED_INDEX) != MAGIC_ERASED )
    {
        /* our active block is corrupted, we are starting over */
        uint32_t data[4] = {MAGIC_DIRTY, 0, 0, 0};

        flash_erase(activeSector_);
        flash_program(activeSector_, MAGIC_DIRTY_INDEX, data, BLOCK_SIZE);

        data[0] = MAGIC_INTACT;
        flash_program(activeSector_, MAGIC_INTACT_INDEX, data, BLOCK_SIZE);

        availableSlots_ = slot_count();
    }
    else
    {
        /* look for first data block */
        for (unsigned block_index = rawBlockCount_ - 1;
             block_index >= MAGIC_COUNT; --block_index)
        {
            if (*block(activeSector_, block_index) != MAGIC_ERASED)
            {
                break;
            }
            ++availableSlots_;
        }
    }

    /* do we shadow_ the data in RAM to speed up reads */
    if (SHADOW_IN_RAM)
    {
        /* copy EEPROM data into shadow_ ram */
        shadow_ = new uint8_t[file_size()];

        /* prime the shadow_ RAM with the EEPROM data */
        read(0, shadow_, file_size());

        /* turn on shadowing */
        shadowInRam_ = true;
    }
}

/** Write to the EEPROM.  NOTE!!! This is not necessarily atomic across
 * BLOCK_SIZE boundaries in the case of power loss.  The user should take this
 * into account as it relates to data integrity of a whole block.
 * @param index within EEPROM address space to start write
 * @param buf data to write
 * @param len length in bytes of data to write
 */
void EEPROMEmulation::write(unsigned int index, const void *buf, size_t len)
{
    HASSERT((index + len) <= file_size());

    uint8_t* byte_data = (uint8_t*)buf;
    uint8_t* shadow_data = (uint8_t*)buf;
    unsigned shadow_index = index;
    size_t shadow_len = len;

    while (len)
    {
        /* get the least significant address bits */
        unsigned int lsa = index & (BYTES_PER_BLOCK - 1);
        if (lsa)
        {
            /* head, (unaligned) address */
            uint8_t data[MAX_BLOCK_SIZE];
            size_t write_size = len < (BYTES_PER_BLOCK - lsa) ?
                                len : (BYTES_PER_BLOCK - lsa);
            read_fblock(index / BYTES_PER_BLOCK, data);

            if (memcmp(data + lsa, byte_data, write_size) != 0)
            {
                /* at least some data has changed */
                memcpy(data + lsa, byte_data, write_size);
                write_fblock(index / BYTES_PER_BLOCK, data);
            }

            index     += write_size;
            len       -= write_size;
            byte_data += write_size;
        }
        else if (len < BYTES_PER_BLOCK)
        {
            /* tail, (unaligned) address */
            uint8_t data[MAX_BLOCK_SIZE];
            read_fblock(index / BYTES_PER_BLOCK, data);

            if (memcmp(data, byte_data, len) != 0)
            {
                /* at least some data has changed */
                memcpy(data, byte_data, len);
                write_fblock(index / BYTES_PER_BLOCK, data);
            }

            len = 0;
        }
        else
        {
            /* aligned data */
            uint8_t data[MAX_BLOCK_SIZE];
            read_fblock(index / BYTES_PER_BLOCK, data);

            if (memcmp(data, byte_data, BYTES_PER_BLOCK) != 0)
            {
                /* at least some data has changed */
                memcpy(data, byte_data, BYTES_PER_BLOCK);
                write_fblock(index / BYTES_PER_BLOCK, data);
            }

            index     += BYTES_PER_BLOCK;
            len       -= BYTES_PER_BLOCK;
            byte_data += BYTES_PER_BLOCK;
        }
    }

    if (shadowInRam_)
    {
        memcpy(shadow_ + shadow_index, shadow_data, shadow_len);
    }

    updated_notification();
}

/** Write to the EEPROM on a native block boundary.
 * @param index block within EEPROM address space to write
 * @param data data to write, array size must be @ref BYTES_PER_BLOCK large
 */
void EEPROMEmulation::write_fblock(unsigned int index, const uint8_t data[])
{
    if (availableSlots_)
    {
        /* still have room in this sector for at least one more write */
        uint32_t slot_data[MAX_BLOCK_SIZE / sizeof(uint32_t)];
        for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
        {
            slot_data[i] = (index << 16) |
                           (data[(i * 2) + 1] << 8) |
                           (data[(i * 2) + 0] << 0);
        }
        flash_program(activeSector_, rawBlockCount_ - availableSlots_, slot_data, BLOCK_SIZE);
        --availableSlots_;
    }
    else
    {
        /* we need to overflow into the next sector */
        unsigned new_sector = next_active();
        uint32_t magic[4] = {MAGIC_DIRTY, 0, 0, 0};

        /* prep the new block */
        flash_erase(new_sector);
        flash_program(new_sector, MAGIC_DIRTY_INDEX, magic, BLOCK_SIZE);

        /* reset the available count */
        unsigned available_slots = slot_count();

        /* move any existing data over */
        for (unsigned int fblock = 0; fblock < (file_size() / BYTES_PER_BLOCK); ++fblock)
        {
            uint32_t slot_data[MAX_BLOCK_SIZE / sizeof(uint32_t)];
            if (fblock == index) // the new data to be written
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
                uint8_t read_data[MAX_BLOCK_SIZE];
                if (!read_fblock(fblock, read_data))
                {
                    /* nothing to write, this is the default "erased" value */
                    continue;
                }
                for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
                {
                    slot_data[i] = (fblock << 16) |
                                   (read_data[(i * 2) + 1] << 8) |
                                   (read_data[(i * 2) + 0] << 0);
                }
            }
            /* commit the write */
            flash_program(new_sector, rawBlockCount_ - available_slots, slot_data, BLOCK_SIZE);
            --available_slots;
        }
        /* finalize the data move and write */
        magic[0] = MAGIC_INTACT;
        flash_program(new_sector, MAGIC_INTACT_INDEX, magic, BLOCK_SIZE);
        magic[0] = MAGIC_USED;
        flash_program(activeSector_, MAGIC_USED_INDEX, magic, BLOCK_SIZE);
        activeSector_ = new_sector;
        availableSlots_ = available_slots;
    }
}

/** Read from the EEPROM.
 * @param offset within EEPROM address space to start read
 * @param buf location to post read data
 * @param len length in bytes of data to read
 */
void EEPROMEmulation::read(unsigned int offset, void *buf, size_t len)
{
    HASSERT((offset + len) <= file_size());

    if (shadowInRam_)
    {
        memcpy(buf, shadow_ + offset, len);
        return;
    }

    uint8_t *byte_data = (uint8_t *)buf;
    memset(byte_data, 0xff, len); // default if data not found

    for (unsigned block_index = slot_first();
         block_index < rawBlockCount_ - availableSlots_;
         ++block_index)
    {
    	const uint32_t *address = block(activeSector_, block_index);
        unsigned slot_offset = (address[0] >> 16) * BYTES_PER_BLOCK;
        // Check if slot overlaps with desired data.
        if (offset + len <= slot_offset)
        {
            continue;
        }
        if (slot_offset + BYTES_PER_BLOCK <= offset)
        {
            continue;
        }
        // Reads the block
        uint8_t data[MAX_BLOCK_SIZE];
        for (unsigned int i = 0; i < BLOCK_SIZE / sizeof(uint32_t); ++i)
        {
            data[(i * 2) + 0] = (address[i] >> 0) & 0xFF;
            data[(i * 2) + 1] = (address[i] >> 8) & 0xFF;
        }
        // Copies the right part into the output buffer.
        unsigned slotofs, bufofs;
        if (slot_offset < offset)
        {
            slotofs = offset - slot_offset;
            bufofs = 0;
        }
        else
        {
            slotofs = 0;
            bufofs = slot_offset - offset;
        }
        unsigned copylen = BYTES_PER_BLOCK - slotofs;
        if (slot_offset + BYTES_PER_BLOCK > offset + len)
        {
            HASSERT(copylen >= (slot_offset + BYTES_PER_BLOCK) - (offset + len));
            copylen -= (slot_offset + BYTES_PER_BLOCK) - (offset + len);
        }
        memcpy(byte_data + bufofs, data + slotofs, copylen);
    }
}

/** Read from the EEPROM on a native block boundary.
 * @param index block within EEPROM address space to read
 * @param data location to place read data, array size must be @ref
 *           BYTES_PER_BLOCK large
 * @param return true if any of the data is not "erased", else return false
 */
bool EEPROMEmulation::read_fblock(unsigned int index, uint8_t data[])
{
    if (shadowInRam_)
    {
        memset(data, 0xff, BYTES_PER_BLOCK);
        if (memcmp(data, shadow_ + (index * BYTES_PER_BLOCK), BYTES_PER_BLOCK) !=
            0)
        {
            memcpy(data, shadow_ + (index * BYTES_PER_BLOCK), BYTES_PER_BLOCK);
            return true;
        }
        return false;
    }
    else
    {
        /* default data value if not found */
        memset(data, 0xFF, BYTES_PER_BLOCK);

        /* look for data */
        for (unsigned raw_block = slot_last();
             raw_block >= slot_first();
             --raw_block)
        {
        	const uint32_t* address = block(activeSector_, raw_block);
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
