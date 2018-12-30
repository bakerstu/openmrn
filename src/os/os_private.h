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
 * @file os_private.cxx
 * This file is a bit of a hack and should only used under extreme caution.
 * its purpose is to allow alternate niche platform support for the OS API's
 *
 * @author Stuart W. Baker
 * @date 29 December 2018
 */

#ifndef _OS_OS_PRIVATE_H_
#define _OS_OS_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__FreeRTOS__)
/// Task list entriy
typedef struct task_list
{
    xTaskHandle task; ///< list entry data
    char * name; ///< name of task 
    size_t unused; ///< number of bytes left unused in the stack
    struct task_list *next; ///< next link in the list
} TaskList;

/// Add a thread to the task list for tracking.
/// @param task_new metadata for new task
void add_thread_to_task_list(TaskList *task_new);

/// Delete a thread from the task list for tracking.
/// @param task_handle FreeRTOS task handle to delete
void del_thread_from_task_list(TaskHandle_t task_handle);
#endif // __FreeRTOS__)

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _OS_OS_PRIVATE_H_

