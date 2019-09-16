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
 * This is implemented by the RegisterableService template below and is used by
 * the service locator to help with type safety.
 */
class RegisterableInterface
{
public:
    virtual const char *get_service_name() = 0;
};

/**
 * Simple serice locator based on name. This is not intended to be used
 * directly as it is not type safe. Instead, use the RegisterableService
 * template class below as the base class for your services you want to
 * register. Then use the static methods in that template.
 */
class ServiceLocator
{
public:
    /**
     * Register a pointer with a name
     * @param name of the service that you want to register
     * @param service is a pointer that you want to register
     */
    static void add_service(std::string name, RegisterableInterface *service)
    {
        services[name] = service;
    }

    /**
     * Retrieves a pointer to the registered service, by name.
     * @param name of the service to retrieve
     * @return the retrieved pointer, which is not guaranteed to be of the
     * requested type
     */
    static RegisterableInterface *get_service(std::string name)
    {
        auto it = services.find(name);
        if (it == services.end())
        {
            return nullptr;
        }

        return it->second;
    }

protected:
    static std::map<std::string, RegisterableInterface *> services;
};

template <typename RegisterableType>
class RegisterableService : public RegisterableInterface
{
public:
    static RegisterableType *get_service()
    {
        RegisterableInterface *service =
            ServiceLocator::get_service(get_type_name());
        return static_cast<RegisterableType *>(service);
    }

    static void register_service(RegisterableType *service)
    {
        ServiceLocator::add_service(get_type_name(), service);
    }

private:
    const char *get_service_name() override
    {
        return get_type_name();
    }

    static const char *get_type_name()
    {
        return __PRETTY_FUNCTION__;
    }
};

#endif // _UTILS_SERVICELOCATOR_HXX_

