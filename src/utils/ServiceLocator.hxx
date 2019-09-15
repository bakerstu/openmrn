/** \copyright
 * Copyright (c) 2019, John Socha-Leialoha
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
 * \file ServiceLocator.hxx
 * This file implements a very simple service locator where you can register
 * and retrieve pointers by name.
 *
 * @author John Socha-Leialoha
 * @date 15 September 2019
 */

#ifndef _UTILS_SERVICELOCATOR_HXX_
#define _UTILS_SERVICELOCATOR_HXX_

#include <map>
#include <string>

/**
 * Add this as a base class for any service you want to be able to register
 */
class RegisterableService
{
};

/**
 * This base class provides a relatively simple map between a name and a pointer.
 * We need this base class because we can't have this in the template.
 */
class ServiceLocatorBase
{
protected:
    static std::map<std::string, RegisterableService *> services;
};

/**
 * Simple serice locator based on name. Note, this is not really type safe.
 * So if you register a pointer with a name and then retrieve it using that
 * name, but a different type, it will almost certainly lead to a crash.
 * @typename Reference is the type of the class you're registering
 */
template <typename Reference> class ServiceLocator : ServiceLocatorBase
{
public:
    /**
     * Register a pointer with a name
     * @param name of the service that you want to register
     * @param service is a pointer that you want to register
     */
    static void add_service(std::string name, Reference *service)
    {
        services[name] = service;
    }

    /**
     * Retrieves a pointer to the registered service, by name.
     * @param name of the service to retrieve
     * @return the retrieved pointer, which is not guaranteed to be of the
     * requested type
     */
    static Reference *get_service(std::string name)
    {
        auto it = services.find(name);
        if (it == services.end())
        {
            return nullptr;
        }

        RegisterableService *service = it->second;
        return static_cast<Reference *>(service);
    }
};

#endif // _UTILS_SERVICELOCATOR_HXX_

