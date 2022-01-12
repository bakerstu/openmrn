/** @copyright
 * Copyright (c) 2021, Balazs Racz
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
 * @file TM4C129xSPIFFS.cxx
 * This file implements a SPIFFS FLASH driver specific to TI TM4C129x.
 *
 * @author Balazs Racz
 * @date 19 Aug 2021
 */

// This define is needed to call any ROM_xx function in the driverlib.
#define TARGET_IS_TM4C129_RA1

#define TI_DUAL_BANK_FLASH
//#define TISPIFFS_LOCK_BASEPRI_FF  // no crash
#define TISPIFFS_LOCK_NONE

#include "spiffs.h"
#include "inc/hw_types.h"
#include "driverlib/flash.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/cpu.h"

#include "TM4C129xSPIFFS.hxx"
#include "../cc32x0sf/TiSPIFFSImpl.hxx"

/// Explicit instantion of the template so that the functions get compiled and
/// into this .o file and the linker would find them.
template class TiSPIFFS<TM4C129x_ERASE_PAGE_SIZE>;
