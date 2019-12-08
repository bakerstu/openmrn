/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file CC32xxHelper.cxx
 * Common helper routines for the TI SimpleLink API.
 *
 * @author Balazs Racz
 * @date 30 December 2016
 */

#define SUPPORT_SL_R1_API

#include "freertos_drivers/ti/CC32xxHelper.hxx"

#include "utils/logging.h"
#include "utils/macros.h"
#include "simplelink.h"
#include "fs.h"

volatile int g_last_sl_result;

void SlCheckResult(int result, int expected)
{
    g_last_sl_result = result;
    HASSERT(result == expected);
}

void SlCheckError(int result)
{
    g_last_sl_result = result;
    HASSERT(result >= 0);
}

int SlDeleteFile(const void* filename)
{
    const uint8_t* name = (const uint8_t*)filename;
    int ret = sl_FsDel(name, 0);
    if (ret == SL_ERROR_FS_FILE_HAS_NOT_BEEN_CLOSE_CORRECTLY)
    {
        ret = sl_FsOpen( name, SL_FS_WRITE, nullptr);
        LOG(INFO, "recreate %s: %d", name, ret);
        if (ret >= 0)
        {
            ret = sl_FsClose(ret, nullptr, nullptr, 0);
            LOG(INFO, "recreate-close %s: %d", name, ret);
        }
        ret = sl_FsDel(name, 0);
        LOG(INFO, "redelete %s: %d", name, ret);
    }
    return ret;
}
