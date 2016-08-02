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
 * \file fcntl.h
 * This file extends the fcntl.h devines
 *
 * @author Stuart W. Baker
 * @date 1 February 2015
 */

#ifndef _EXTENDED_FCNTL_H_
#define _EXTENDED_FCNTL_H_

#include <sys/fcntl.h>

#ifdef __cplusplus
extern "C" {
#endif

/// ioctl parameter space for our custom defined ioctls in the freertos
/// drivers.
#define F_FREERTOS_SPECIFIC_BASE 1000

/// ioctl to set the length of the internal fifo buffer.
#define F_SETPIPE_SZ (F_FREERTOS_SPECIFIC_BASE + 0)


#ifndef FREAD
/// Workaround for missing header defines on some newlib versions.
#define FREAD 1
#endif

#ifndef FWRITE
/// Workaround for missing header defines on some newlib versions.
#define FWRITE 2
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EXTENDED_FCNTL_H_ */
