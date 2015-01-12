/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file 11cxx_async_can.hxx
 * This file implements an asynchronous CAN driver for the LPC11Cxx
 * microcontrollers, using the builtin ROM drivers.
 *
 * @author Balazs Racz
 * @date 14 Dec 2013
 */

#ifndef _FREERTOS_DRIVERS_NXP_11CXX_ASYNC_CAN_HXX_
#define _FREERTOS_DRIVERS_NXP_11CXX_ASYNC_CAN_HXX_
#ifdef TARGET_LPC11Cxx

#include "utils/Hub.hxx"

namespace lpc11cxx
{

// Creates a CAN driver for the LPC11CXX, and adds it to the specified pipe.
void CreateCanDriver(CanHubFlow* parent);

} // namespace lpc11cxx

#endif // TARGET_LPC11Cxx
#endif // _FREERTOS_DRIVERS_NXP_11CXX_ASYNC_CAN_HXX_
