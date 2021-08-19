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
 * @file CC32x0SFSPIFFS.hxx
 * This file implements a SPIFFS FLASH driver specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 1 January 2018
 */

#ifndef _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32X0SFSPIFFS_HXX_
#define _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32X0SFSPIFFS_HXX_

#include "../cc32x0sf/TiSPIFFS.hxx"

static constexpr unsigned CC32xxSF_ERASE_PAGE_SIZE = 2 * 1024;

using CC32x0SFSPIFFS = TiSPIFFS<CC32xxSF_ERASE_PAGE_SIZE>;

#endif // _FREERTOS_DRIVERS_SPIFFS_CC3220SF_CC32X0SFSPIFFS_HXX_

