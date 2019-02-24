/** @copyright
 * Copyright (c) 2018, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * @file TempFile.cxx
 *
 * Helper classes for creating temporary files for testing and mock
 * implementations.
 *
 * @author Stuart Baker
 * @date 21 August 2018
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L
#endif

#if defined(__MACH__)
#define _DARWIN_C_SOURCE // mkdtemp
#endif

#include "os/TempFile.hxx"

/// @todo mingw does not seem to have an mkdtemp call.
#if !defined(__FreeRTOS__) && !defined(__WINNT__)
TempDir::TempDir()
{
#if defined(__linux__) || defined(__MACH__)
    dirName_ = "/tmp/openmrntmpdirXXXXXX";
#else
    dirName_ = "./openmrntmpdirXXXXXX";
#endif
    dirName_.c_str();
    HASSERT(mkdtemp(&dirName_[0]));
}
#endif

//
// TempFile::TempFile()
//
TempFile::TempFile(const TempDir& dir, const string& basename)
{
    fileName_ = dir.name() + "/" + basename + ".XXXXXX";
    fileName_.c_str();
    fd_ = mkstemp((char*)fileName_.c_str());
}
