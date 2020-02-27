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
 * \file CC32xxHelper.hxx
 * Common helper routines for the TI SimpleLink API.
 *
 * @author Balazs Racz
 * @date 30 December 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXHELPER_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXHELPER_HXX_

/**
 * Tests that a SimpleLink request has succeeded. Performs internal debugging
 * when it failed and crashes.
 *
 * @param result is the returned value from the simplelink driver
 * @param expected is the expected return code.
 */
void SlCheckResult(int result, int expected = 0);

/**
 * Tests that a SimpleLink request has succeeded (return >= 0). Performs
 * internal debugging when it failed and crashes.
 *
 * @param result is the returned value from the simplelink driver
 */
void SlCheckError(int result);

/**
 * Deletes a file from sflash. This routine works around the error
 * SL_FS_FILE_HAS_NOT_BEEN_CLOSE_CORRECTLY which prevents deleting a file.
 * @param filename serial-flash file name to delete.
 * @return the result code from the last delete operation. It is OK for this to
 * be non zero.
 */
int SlDeleteFile(const void* filename);


#endif // _FREERTOS_DRIVERS_TI_CC32XXHELPER_HXX_

