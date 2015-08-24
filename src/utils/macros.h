/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file macros.h
 *
 * Useful macros shared across the entire codebase.
 *
 * @author Balazs Racz
 * @date 19 July 2013
 */

#ifndef _UTILS_MACROS_H_
#define _UTILS_MACROS_H_

#ifdef __cplusplus
#include <vector>
#include <map>
#include <string>
#include <utility>

using std::vector;
using std::map;
using std::string;
using std::pair;
#endif

#include <stdlib.h>   // for abort

#if defined(__EMSCRIPTEN__) 
#if defined(EXPECT_DEATH)
#undef EXPECT_DEATH
#endif
#define EXPECT_DEATH(x...) 
#endif

#ifdef __FreeRTOS__

/**
   Hard assertion facility. These checks will remain in production code, and
   they should guard logic errors that make it impossible to continue the
   running of the program, and termination/reboot is preferred.

   An example would be to check a pointer being not-NULL before dereferencing
   it. The resulting fault is typically much harder to debug than an assert.
 */
#define HASSERT(x) do { if (!(x)) abort(); } while(0)


#define DIE(MSG) abort()


#else

#include <assert.h>
#include <stdio.h>

#ifdef NDEBUG
#define HASSERT(x) do { if (!(x)) { fprintf(stderr, "Assertion failed in file " __FILE__ " line %d: assert(" #x ")", __LINE__); abort();} } while(0)
#else
#define HASSERT(x) do { assert(x); } while(0)
#endif

#define DIE(MSG) do { fprintf(stderr, "Crashed in file " __FILE__ " line %d: " MSG, __LINE__); abort(); } while(0)

#endif

#ifdef NDEBUG

/** Debug assertion facility. Will terminate the program if the program was
   compiled without NDEBUG symbol.
 */
#define DASSERT(x) 

#else

#define DASSERT(x) HASSERT(x)

#endif


/**
   Removes default copy-constructor and assignment added by C++.

   This macro should be used in the private part of all classes that are not
   meant to be copied (which is almost all classes), to avoid bugs resulting
   from unintended passing of the objects by value.
 */
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);   \
  void operator=(const TypeName&)

/** Function attribute for virtual functions declaring that this funciton is
 * overriding a funciton that should be virtual in the base class. Supported by
 * GCC >= 4.7, so currently empty */
#define OVERRIDE 

/** Returns the number of elements in a statically defined array (of static
 *  size) */
#define ARRAYSIZE(a) (sizeof(a) / sizeof(a[0]))

/// Adds a constructor to the current class that proxies every argument to the
/// base constructor.
///
/// @param CURRENT_CLASS the name of the current class
/// @param BASE_CLASS the name of the immediate base class
#define INHERIT_CONSTRUCTOR(CURRENT_CLASS, BASE_CLASS)                         \
    template <typename... Args>                                                \
    explicit CURRENT_CLASS(Args... args)                                       \
        : BASE_CLASS(args...)                                                  \
    {                                                                          \
    }

/// Adds a constexpr constructor to the current class that proxies every
/// argument to the base constructor.
///
/// @param CURRENT_CLASS the name of the current class
/// @param BASE_CLASS the name of the immediate base class
#define INHERIT_CONSTEXPR_CONSTRUCTOR(CURRENT_CLASS, BASE_CLASS)               \
    template <typename... Args>                                                \
    explicit constexpr CURRENT_CLASS(Args... args)                             \
        : BASE_CLASS(args...)                                                  \
    {                                                                          \
    }

#define C_STATIC_ASSERT(expr, name) \
    typedef unsigned char __static_assert_##name[expr ? 0 : -1];

#endif // _UTILS_MACROS_H_
