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
 * \file Node.cxx
 * This file imlements Node level methods of the device file system.
 *
 * @author Stuart W. Baker
 * @date 31 January 2015
 */

#include "Devtab.hxx"

#include <algorithm>

#include "can_ioctl.h"
#include "executor/Notifiable.hxx"

/** Open method */
int Node::open(File *, const char *, int, int) OVERRIDE {
    OSMutexLock l(&lock_);
    if (references_++ == 0)
    {
        enable();
    }
    return 0;
}

/** Close method */
int Node::close(File *) OVERRIDE {
    OSMutexLock l(&lock_);
    if (--references_ <= 0)
    {
        disable();
        references_ = 0;
    }
    return 0;
}



/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int NonBlockNode::ioctl(File *file, unsigned long int key, unsigned long data)
{
    /* sanity check to be sure we have a valid key for this device */
    HASSERT(IOC_TYPE(key) == CAN_IOC_MAGIC);

    // Will be called at the end if non-null.
    Notifiable* n = nullptr;

    if (IOC_SIZE(key) == NOTIFIABLE_TYPE) {
        n = reinterpret_cast<Notifiable*>(data);
    }

    switch (key)
    {
        default:
            return -EINVAL;
        case CAN_IOC_READ_ACTIVE:
            portENTER_CRITICAL();
            if (!has_rx_buffer_data())
            {
                swap(n, readableNotify_);
            }
            portEXIT_CRITICAL();
            break;
        case CAN_IOC_WRITE_ACTIVE:
            portENTER_CRITICAL();
            if (!has_tx_buffer_space())
            {
                swap(n, writableNotify_);
            }
            portEXIT_CRITICAL();
            break;
    }
    if (n)
    {
        n->notify();
    }
    return 0;
}
