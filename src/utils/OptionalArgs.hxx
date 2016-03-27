/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file OptionalArgs.hxx
 *
 * Constexpr structure for storing a list of optional arguments to a function
 * that needs to be optimized at compile time.
 *
 * @author Balazs Racz
 * @date 24 November 2015
 */

#ifndef _UTILS_OPTIONALARGS_HXX_
#define _UTILS_OPTIONALARGS_HXX_

/// Used as an argument to the get(Fetcher) function in the OptionalArg
/// implementation to select which entry to retrieve.
template <typename DATA_TYPE, int N> class Fetcher
{
public:
    constexpr Fetcher()
    {
    }
};

/// Used as an argument to the constructor of the OptionalArg implementation to
/// represent that a specific argument has to be overridden with a given value.
template <typename DATA_TYPE, int N, DATA_TYPE defval> class Specifier
{
public:
    constexpr Specifier(const DATA_TYPE d)
        : d_(d)
    {
    }

    typedef Fetcher<DATA_TYPE, N> FetcherType;
    typedef DATA_TYPE data_type;
    static constexpr DATA_TYPE default_value()
    {
        return defval;
    }

    DATA_TYPE d_;
};

#define DECLARE_OPTIONALARG(SpecName, function_name, DataType, N, DEF)         \
    using SpecName = Specifier<DataType, N, (DEF)>;                            \
    using SpecName##Get = Fetcher<DataType, N>;                                \
    static constexpr int check_arguments_are_valid(const SpecName s)           \
    {                                                                          \
        return 0;                                                              \
    }

#define DEFINE_OPTIONALARG(SpecName, function_name, DataType)                  \
    constexpr DataType function_name() const                                   \
    {                                                                          \
        return get(SpecName##Get());                                           \
    }

/// Declares that a recursive class template is coming.
template <class Decl, typename... Args> class OptionalArg;

/// Terminates the class recursion template.
///
/// The constructor is used to check that the customer has specified only known
/// parameters.
///
/// The get function throws an error deterministically, because if the compiler
/// gets this deep in the get function, then none of the actual data carrying
/// components matched on the type. This means that the customer is trying to
/// fetch something we didn't store.
template <class Decl> class OptionalArg<Decl> : public Decl
{
public:
    template <typename... Args>
    constexpr OptionalArg(Args... args)
        : check_(check_all_args(args...))
    {
    }

    template <class F> constexpr F get(const F f) const
    {
        return tried_to_get_unknown_argument() ? f : F();
    }

private:
    using Decl::check_arguments_are_valid;
    static constexpr int check_arguments_are_valid(const OptionalArg &a)
    {
        return 0;
    }

    template <typename A, typename... Args>
    static constexpr int check_all_args(const A a, Args... args)
    {
        return check_arguments_are_valid(a) + check_all_args(args...);
    }

    static constexpr int check_all_args()
    {
        return 0;
    }

    static bool tried_to_get_unknown_argument(); // unimplemented.

    const int check_;
};

/// Template recursion entry. We have as many instances of this class in the
/// inheritance stack as the number of data elements we need to carry. Each of
/// these classes stores one single element in the private variable d_.
///
/// The constructor picks out the specifier for the current entry to fill in
/// the current storage. All arguments are forwarded to the base class. This
/// allows arbitrary order of the specified arguments; it is the responsibility
/// of the innermost class to check for spurious arguments.
///
/// The get method either recognizes the Fetcher argument as referring to the
/// current entry, or forwards it to the parent class.
template <typename Decl, typename Specifier, typename... TArgs>
class OptionalArg<Decl, Specifier, TArgs...>
    : public OptionalArg<Decl, TArgs...>
{
public:
    typedef Specifier specifier_type;
    typedef typename Specifier::FetcherType fetcher_type;
    typedef typename Specifier::data_type data_type;
    using Base = OptionalArg<Decl, TArgs...>;

    template <typename... Args>
    constexpr OptionalArg(Args... args)
        : Base(args...)
        , d_(GetFromArgs(args...))
    {
    }

    constexpr OptionalArg()
        : d_(GetFromArgs())
    {
    }

    constexpr data_type get(const fetcher_type) const
    {
        return d_;
    }

    /// Needed due to templated base class; the public inheritance is not
    /// enough.
    using Base::get;

private:
    /// This template gets instantiated when the first argument is for us.
    template <typename... Args>
    static constexpr data_type GetFromArgs(
        const specifier_type spec, Args... args)
    {
        return spec.d_;
    }

    /// This template gets instantiated for a copy constructor: when the
    /// argument is already an OptionalArg (or reference to it).
    template <typename U, typename... Args>
    static constexpr typename std::enable_if<
        std::is_convertible<typename std::add_lvalue_reference<U>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        data_type>::type
    GetFromArgs(const U me, Args... args)
    {
        return me.get(fetcher_type());
    }

    /// This template gets instantiated only if the argument is not an
    /// OptionalArg and not for the current entry. Then we just ignore the
    /// first arg.
    template <typename T, typename... Args>
    static constexpr typename std::enable_if<
        !std::is_convertible<typename std::add_lvalue_reference<T>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        data_type>::type
    GetFromArgs(const T t, Args... args)
    {
        return GetFromArgs(args...);
    }

    // If we've run out of all arguments, we take the default value.
    static constexpr data_type GetFromArgs()
    {
        return specifier_type::default_value();
    }

    data_type d_;
};

#endif // _UTILS_OPTIONALARGS_HXX_
