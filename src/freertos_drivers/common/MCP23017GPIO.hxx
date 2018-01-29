/** @copyright
 * Copyright (c) 2017 Stuart W Baker
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
 * @file MCP23017GPIO.hxx
 * This file implements a GPIO driver for the MCP23017 I/O expander.
 *
 * @author Stuart W. Baker
 * @date 7 October 2017
 */

#ifndef _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_
#define _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_

#include "os/Gpio.hxx"
#include "os/OS.hxx"

class MCP23017GPIO;

/** I/O expansion driver Base to avoid template bloat.
 */
class MCP23017Base : public OSThread
{
public:
    /** Initialize I2C device settings.  Typically called in hw_postinit(), not
     * hw_preinit() or hw_init().
     * @param i2c_name spi interface that the MCP23017 is on
     * @param i2c_address 7-bit I2C address, successive devices on the same
     *                    I2C bus must have sequencial addresses
     */
    void init(const char *i2c_name, uint8_t i2c_address);

    /** Handle an interrupt.  Called by user provided interrupt handler.
     */
    void interrupt_handler();

protected:
    /** Constructor.
     * @param interrupt_enable callback to enable the interrupt
     * @param interrupt_disable callback to disable the interrupt
     * @param interrupt_lockout_time the time in nanoseconds to enable the
     *                               next interrupt (optional debounce)
     */
    MCP23017Base(void (*interrupt_enable)(), void (*interrupt_disable)(),
                 long long interrupt_lockout_time = 0)

        : OSThread()
        , interrupt_enable(interrupt_enable)
        , interrupt_disable(interrupt_disable)
        , sem_()
        , intLockTime_(interrupt_lockout_time)
        , fd_(-1)
        , i2cAddress_(0)
    {
    }

    /** Destructor.
     */
    ~MCP23017Base()
    {
    }

private:
    /** I2C registers. */
    enum Registers
    {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        IOCON,
        IOCON_ALT,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
    };

    /** Writes a GPIO output pin (set or clear to a specific state).
     * @param new_state the desired output state. See @ref Value.
     * @param bit bit value (0 - 15) to act on
     */
    void write(Gpio::Value new_state, uint8_t bit)
    {
        if (new_state)
        {
            set(bit);
        }
        else
        {
            clr(bit);
        }
    }

    /** Retrieves the current @ref Value of a GPIO input pin.
     * @return @ref SET if currently high, @ref CLR if currently low.
     * @param bit bit value (0 - 15) to act on
     */
    virtual Gpio::Value read(uint8_t bit) = 0;

    /** Sets the GPIO output pin to high.
     * @param bit bit value (0 - 15) to act on
     */
    virtual void set(uint8_t bit) = 0;

    /** Clears the GPIO output pin to low.
     * @param bit bit value (0 - 15) to act on
     */
    virtual void clr(uint8_t bit) = 0;

    /** Sets the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    virtual void set_direction(Gpio::Direction dir, uint8_t bit) = 0;

    /** Gets the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    virtual Gpio::Direction direction(uint8_t bit) = 0;

    /** User entry point for the created thread.
     * @return exit status
     */
    void *entry() override; /**< entry point to thread */

    /** Enable interrupt callback.
     */
    void (*interrupt_enable)();

    /** Disable interrupt callback.
     */
    void (*interrupt_disable)();

    /** Number of MCP23017 chips on a single bus.
     * @return number of chips
     */
    virtual uint8_t chips() = 0;

    /** Set the local input port data.
     * @param data data to set
     * @param index chip index to access
     */
    virtual void set_data_in(uint16_t data, uint8_t index) = 0;

    /** Get the local output latch data.
     * @param index chip index that the data is for
     * @return output latch data
     */
    virtual uint16_t get_data_out(uint8_t index) = 0;

    /** Get the local I/O direction.
     * @param index chip index that the data is for
     * @return I/O direction
     */
    virtual uint16_t get_direction(uint8_t index) = 0;

    /** Write to an I2C register.
     * @param reg register to write
     * @param value register value to write
     * @param index chip index that the data is for
     */
    void reg_write(Registers reg, uint8_t value, uint8_t index);

    /** Read from an I2C register.
     * @param reg register to read
     * @return value read from the register
     * @param index chip index to access
     */
    uint8_t reg_read(Registers reg, uint8_t index);

    /** semaphore for notifying the thread from an ISR */
    OSSem sem_;

    /** time to lockout interrupts for debounce */
    long long intLockTime_;

    /** file descriptor for the I2C interface */
    int fd_;

    /** I2C address of the device */
    uint8_t i2cAddress_;

    /** Allow access to MCP23017Base from MCP23017GPIO */
    friend class MCP23017GPIO;

    DISALLOW_COPY_AND_ASSIGN(MCP23017Base);
};

/** Specialization of the MCP23017 base class templated for the number of
 * devices on a single I2C bus (16 I/O lines ber device).
 */
template <uint8_t N = 1> class MCP23017 : public MCP23017Base
{
public:
    /** Constructor.
     * @param interrupt_enable callback to enable the interrupt
     * @param interrupt_disable callback to disable the interrupt
     * @param interrupt_lockout_time the time in nanoseconds to enable the
     *                               next interrupt (optional debounce)
     */
    MCP23017(void (*interrupt_enable)(), void (*interrupt_disable)(),
             long long interrupt_lockout_time = 0)
        : MCP23017Base(interrupt_enable, interrupt_disable,
                       interrupt_lockout_time)
    {
        HASSERT(N > 0 && N <= 8);

        for (uint8_t i = 0; i < N; ++i)
        {
            dataIn_[i] = 0;
            dataOut_[i] = 0;
            direction_[i] = 0xFFFF;
        }
    }

    /** Destructor.
     */
    ~MCP23017()
    {
    }

private:
    /** Retrieves the current @ref Value of a GPIO input pin.
     * @return @ref SET if currently high, @ref CLR if currently low.
     * @param bit bit value (0 - 15) to act on
     */
    Gpio::Value read(uint8_t bit) override
    {
        int index = bit / 16;
        int shift = bit % 16;

        return (dataIn_[index] & (0x1 << shift)) ? Gpio::SET : Gpio::CLR;
    }   

    /** Sets the GPIO output pin to high.
     * @param bit bit value (0 - 15) to act on
     */
    void set(uint8_t bit) override
    {
        int index = bit / 16;
        int shift = bit % 16;

        portENTER_CRITICAL();
        dataOut_[index] |= 0x1 << shift;
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Clears the GPIO output pin to low.
     * @param bit bit value (0 - 15) to act on
     */
    void clr(uint8_t bit) override
    {
        int index = bit / 16;
        int shift = bit % 16;

        portENTER_CRITICAL();
        dataOut_[index] &= ~(0x1 << shift);
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Sets the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    void set_direction(Gpio::Direction dir, uint8_t bit) override
    {
        int index = bit / 16;
        int shift = bit % 16;

        portENTER_CRITICAL();
        if (dir == Gpio::Direction::INPUT)
        {
            direction_[index] |= 0x1 << shift;
        }
        else
        {
            direction_[index] &= ~(0x1 << shift);
        }
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Gets the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    Gpio::Direction direction(uint8_t bit) override
    {
        int index = bit / 16;
        int shift = bit % 16;

        return (direction_[index] & (0x1 << shift)) ? Gpio::Direction::INPUT :
                                                      Gpio::Direction::OUTPUT;
    }

    /** Number of MCP23017 chips on a single bus.
     * @return number of chips
     */
    uint8_t chips() override
    {
        return N;
    }

    /** Set the local input port data.
     * @param data data to set
     * @param index chip index that the data is for
     */
    void set_data_in(uint16_t data, uint8_t index) override
    {
        dataIn_[index] = data;
    }

    /** Get the local output latch data.
     * @param index chip index that the data is for
     * @return output latch data
     */
    uint16_t get_data_out(uint8_t index) override
    {
        return dataOut_[index];
    }

    /** Get the local I/O direction.
     * @param index chip index that the data is for
     * @return I/O direction
     */
    uint16_t get_direction(uint8_t index) override
    {
        return direction_[index];
    }

    /** local copy of the I/O expansion input port bits */
    uint16_t dataIn_[N];

    /** local copy of the I/O expansion output latch bits */
    uint16_t dataOut_[N];

    /** local copy of the I/O direction */
    uint16_t direction_[N];

    DISALLOW_COPY_AND_ASSIGN(MCP23017);
};

/** Single GPIO instance using an MCP23017 device.
 */
class MCP23017GPIO : public Gpio
{
public:
    MCP23017GPIO(MCP23017Base *instance, uint8_t bit)
        : Gpio()
        , instance_(instance)
        , bit_(bit)
    {
        HASSERT(bit < (instance_->chips() * 16));
    }

    /** Writes a GPIO output pin (set or clear to a specific state).
     * @param new_state the desired output state. See @ref Value.
     */
    void write(Value new_state) const override
    {
        instance_->write(new_state, bit_);
    }

    /** Retrieves the current @ref Value of a GPIO input pin.
     * @return @ref SET if currently high, @ref CLR if currently low.
     */
    Value read() const override
    {
        return instance_->read(bit_);
    }   

    /** Sets the GPIO output pin to high.
     */
    void set() const override
    {
        instance_->set(bit_);
    }

    /** Clears the GPIO output pin to low.
     */
    void clr() const override
    {
        instance_->clr(bit_);
    }

    /** Sets the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     */
    void set_direction(Direction dir) const override
    {
        instance_->set_direction(dir, bit_);
    }

    /** Gets the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     */
    Direction direction() const override
    {
        return instance_->direction(bit_);
    }

private:
    /** reference to the chip instance */
    MCP23017Base *instance_;

    /** bit number representative of the bit */
    uint8_t bit_;

    DISALLOW_COPY_AND_ASSIGN(MCP23017GPIO);
};

#endif /* _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_ */
