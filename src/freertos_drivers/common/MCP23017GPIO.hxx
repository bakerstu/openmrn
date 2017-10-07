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

/** I/O expansion driver.
 */
class MCP23017 : public OSThread
{
public:
    /** Constructor.
     * @param name name of the I2C device in the file system
     * @param interrupt_enable callback to enable the interrupt
     * @param interrupt_disable callback to disable the interrupt
     */
    MCP23017(const char *name,
             void (*interrupt_enable)(), void (*interrupt_disable)())

        : OSThread()
        , interrupt_enable(interrupt_enable)
        , interrupt_disable(interrupt_disable)
        , sem_()
        , fd_(-1)
        , data_(0)
        , dataShaddow_(0)
        , direction_(0xFFFF)
        , directionShaddow_(0xFFFF)
        , i2cAddress_(0)
    {
    }

    /** Destructor.
     */
    ~MCP23017()
    {
    }

    /** Initialize I2C device settings.  Typically called in hw_postinit(), not
     * hw_preinit() or hw_init().
     * @param i2c_name spi interface that the MCP23017 is on
     * @param i2c_address 7-bit I2C address
     */
    void init(const char *i2c_name, uint8_t i2c_address);

    /** Handle an interrupt.  Called by user provided interrupt handler.
     */
    void interrupt_handler();

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
    Gpio::Value read(uint8_t bit)
    {
        return (data_ & (0x1 << bit)) ? Gpio::SET : Gpio::CLR;
    }   

    /** Sets the GPIO output pin to high.
     * @param bit bit value (0 - 15) to act on
     */
    void set(uint8_t bit)
    {
        portENTER_CRITICAL();
        dataShaddow_ |= 0x1 << bit;
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Clears the GPIO output pin to low.
     * @param bit bit value (0 - 15) to act on
     */
    void clr(uint8_t bit)
    {
        portENTER_CRITICAL();
        dataShaddow_ &= ~(0x1 << bit);
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Sets the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    void set_direction(Gpio::Direction dir, uint8_t bit)
    {
        portENTER_CRITICAL();
        if (dir == Gpio::Direction::INPUT)
        {
            directionShaddow_ |= 0x1 << bit;
        }
        else
        {
            directionShaddow_ &= ~(0x1 << bit);
        }
        portEXIT_CRITICAL();
        sem_.post();
    }

    /** Gets the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     * @param bit bit value (0 - 15) to act on
     */
    Gpio::Direction direction(uint8_t bit)
    {
        return (direction_ & (0x1 << bit)) ? Gpio::Direction::INPUT :
                                             Gpio::Direction::OUTPUT;
    }

private:
    enum
    {
        OUTPUT = 0,
        INPUT = 1,
    };

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

    /** User entry point for the created thread.
     * @return exit status
     */
    void *entry() override; /**< entry point to thread */

    void (*interrupt_enable)(); /**< enable interrupt callback */
    void (*interrupt_disable)(); /**< disable interrupt callback */

    /** Write to an I2C register.
     * @param reg register to write
     * @param value register value to write
     */
    void register_write(Registers reg, uint8_t value);

    /** Read from an I2C register.
     * @param reg register to read
     * @return value read from the register
     */
    uint8_t register_read(Registers reg);

    /** semaphore for notifying the thread from an ISR */
    OSSem sem_;

    /** file descriptor for the I2C interface */
    int fd_;

    union
    {
        /** local copy of the I/O expansion bits */
        uint16_t data_;
        struct
        {
            uint8_t dataA_;
            uint8_t dataB_;
        };
    };

    /** local copy of the I/O expansion bits */
    uint16_t dataShaddow_;

    /** local copy of the I/O direction */
    uint16_t direction_;

    /** local copy of the I/O direction */
    uint16_t directionShaddow_;

    /** I2C address of the device */
    uint8_t i2cAddress_;

};

class MCP23017GPIO : public Gpio
{
public:
    MCP23017GPIO(MCP23017 *instance, uint8_t bit)
        : Gpio()
        , instance_(instance)
        , bit_(bit)
    {
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
    MCP23017 *instance_;

    /** bit number representative of the bit */
    uint8_t bit_;
};

#endif /* _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_ */
