/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file PersistentGpio.hxx
 *
 * A wrapper around GPIO (output) objects that stores last set state in EEPROM
 * and restores it from there upon boot.
 *
 * @author Balazs Racz
 * @date 7 June 2017
 */

#ifndef _OS_PERSISTENTGPIO_HXX_
#define _OS_PERSISTENTGPIO_HXX_

#include "utils/StoredBitSet.hxx"
#include "os/Gpio.hxx"

extern StoredBitSet *g_gpio_stored_bit_set;

class PersistentGpio : public Gpio
{
public:
    constexpr PersistentGpio(const Gpio *impl, const unsigned bit_ofs)
        : impl_(impl)
        , bit_(bit_ofs)
    {
    }

    void restore() const {
        impl_->write(g_gpio_stored_bit_set->get_bit(bit_));
    }
    
    void write(Value new_state) const OVERRIDE
    {
        impl_->write(new_state);
        g_gpio_stored_bit_set->set_bit(bit_, new_state).lock_and_flush();
    }

    void set() const OVERRIDE
    {
        impl_->set();
        g_gpio_stored_bit_set->set_bit(bit_, true).lock_and_flush();
    }

    void clr() const OVERRIDE
    {
        impl_->clr();
        g_gpio_stored_bit_set->set_bit(bit_, false).lock_and_flush();
    }

    Value read() const OVERRIDE
    {
        return impl_->read();
    }

    void set_direction(Direction dir) const OVERRIDE
    {
        // NOTE: not sure if it's a good idea to let the direction be modified,
        // since upon boot we will restore something else. We don't persist the
        // direction.
        return impl_->set_direction(dir);
    }

    Direction direction() const OVERRIDE
    {
        return impl_->direction();
    }

private:
    /// Implementation of the gpio object.
    const Gpio *impl_;
    /// Implementation of the gpio object.
    const unsigned bit_;
};

#endif // _OS_PERSISTENTGPIO_HXX_
