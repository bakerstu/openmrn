/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file dirent.h
 * This file implements the missing dirent.h functionality.
 *
 * @author Stuart W. Baker
 * @date 21 January 2018
 */

#ifndef _EXTENDED_DIRENT_H_
#define _EXTENDED_DIRENT_H_

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** DIR typedef */
typedef uintptr_t DIR;

/** Directory entry structure */
struct dirent
{
    ino_t d_ino;    /**< file serial number */
    char  d_name[]; /**< filename string of entry */
};

/** Close a directory.
 * @param @dirp directory pointer to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int closedir(DIR *dirp);

/** Open a directory.
 * @param name directory path
 * @return pointer to the open directory on success, NULL on error
 */
DIR *opendir(const char *name);

/** Read the next entry in a directory.
 * @param dirp directory pointer to read.
 * @return pointer to a struct dirent representing the next directory entry
 */
struct dirent *readdir(DIR *dirp);

#ifdef __cplusplus
}
#endif

#endif /* _EXTENDED_DIRENT_H_ */

