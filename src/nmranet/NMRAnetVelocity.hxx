/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file NMRAnetVelocity.hxx
 * This file provides an implementation of velocity in NMRAnet terms.
 *
 * @author Stuart W. Baker
 * @date 2 August 2013
 */

#include <math.h>
#include <stdint.h>

extern "C" {
/* These come from the ieeehalfprecision.c */
int singles2halfp(void *target, void *source, int numel);
int halfp2singles(void *target, void *source, int numel);
}

/** Conversion factor for MPH. */
#define MPH_FACTOR 0.44704  

namespace NMRAnet
{

/** This type represents how velocity is seen on the wire (16 bit float).
 */
typedef uint16_t float16_t;

/** This class provides a mechanism for working with velocity in different
 *  forms.  A single precision floating point value is used internally to store
 *  velocity, but this class provides the ability to easily work with
 *  different velocity formats including DCC 14/28/128 speed step formats.
 *  NMRAnet velocity is represented as a floating point meters/sec.  The sign
 *  represents direction where negative is reverse and positive is forward.
 *  Sign is always preserved even when the result is 0.  For example,
 *  -7 + 7 = -0 and 7 - 7 = +0.
 */
class Velocity
{
public:
    /** define an enumeration for direction
     */
    enum
    {
        FORWARD = 0, /**< forward direction */
        REVERSE,     /**< reverse direction */
    };

    /** Basic constructor.
     * @param value starting value for Velocity.
     */
    Velocity(float value = 0.0)
        : velocity(value)
    {
    }

    /** Constructor that takes the 16 bit wire format
     * @param value starting value for Velocity as IEEE half precision float.
     */
    Velocity(float16_t value)
        : velocity(halfp2singles(&velocity, &value, 1))
    {
    }

    /** Copy constructor. */
    Velocity(const Velocity& old_velocity)
        : velocity(old_velocity.velocity)
    {
    }

    /** Destructor does nothing. */
    ~Velocity() {}

    /** Return the speed independent of direction.
     * @return speed absolute value of velocity
     */
    float speed()
    {
        return fabsf(velocity);
    }

    /** Return the direction independent of speed.
     * @return direction FORWARD or REVERSE
     */
    int direction()
    {
        if (velocity == -0 || velocity < 0)
        {
            return REVERSE;
        }
        return FORWARD;
    }
    
    /** Set the direction to forward. */
    void forward()
    {
        if (velocity == -0 || velocity < 0)
        {
            velocity = -velocity;
        }
    }
    
    /** Set the direction to reverse. */
    void reverse()
    {
        if (velocity != -0 && velocity >= 0)
        {
            velocity = -velocity;
        }
    }
    
    /** Convert the native meters/sec representation into mile per hour.
     * @return velocity represented as miles per hour
     */
    float mph()
    {
        int sign = get_sign(velocity);

        return zero_adjust(velocity * MPH_FACTOR, sign);
    }
    
    /** Get the speed in DCC 128 speed step format.
     *  The mapping from meters/sec is strait forward.  First convert to
     *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
     *  126 miles/hour.
     *
     *  bit 7:  direction
     *  bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_128()
    {
        uint8_t result;
        float tmp = (speed() * MPH_FACTOR) + 0.5;
        
        if (tmp == 0)
        {
            result = 0;
        }
        else if (tmp > 126)
        {
            result = 127;
        }
        else
        {
            result = (uint8_t)(tmp + 1);
        }
        
        result |= get_sign(velocity) == -1 ? 0x00 : 0x80;
        return result;
    }

    /** Set the speed from DCC 128 speed step format.
     *  The mapping from meters/sec is strait forward.  First convert to
     *  miles/hour, then each speed step represents 1 mile/hour.  Saturate at
     *  126 miles/hour.
     *
     *  @param value bit 7:  direction
     *               bits 6..0:  0 = stopped, 1 = estop, 2 - 127 = speed steps 1 - 126
     */
    void set_dcc_128(uint8_t value)
    {
        if ((value & 0x7F) <= 1)
        {
            velocity = 0;
        }
        else
        {
            velocity = (value & 0x07F) - 1;
            velocity /= MPH_FACTOR;
        }
        
        if ((value & 0x80) == 0)
        {
            velocity = -velocity;
        }
    }
    
    /** Get the speed in DCC 28 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  bit 7..6:  fixed at b'01'
     *  bit 5:  direction
     *  bits 4:  speed step least significant bit
     *  bits 3..0:  speed step significatn bits 4..1
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_28()
    {
        uint8_t result;
        float tmp = ((speed() * MPH_FACTOR * 28) / 128) + 0.5;
        
        if (tmp == 0)
        {
            result = 0;
        }
        else if (tmp > 28)
        {
            result = 31;
        }
        else
        {
            result = (uint8_t)(tmp + 3);
        }
        
        result |= result & 0x01 ? 0xA0 : 0x80;
        
        result >>= 1;

        result |= get_sign(velocity) == -1 ? 0x00 : 0x20;
        return result;
    }
    
    /** Set the speed from DCC 28 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  @param value bit 7..6:  fixed at b'01'
     *               bit 5:  direction
     *               bits 4:  speed step least significant bit
     *               bits 3..0:  speed step significatn bits 4..1
     */
    void set_dcc_28(uint8_t value)
    {
        value <<= 1;
        
        value |= value & 0x20 ? 0x01 : 0x00;
        
        if ((value & 0x1F) <= 3)
        {
            velocity = 0;
        }
        else
        {
            velocity = (value & 0x01F) - 3;
            velocity *= 128;
            velocity /= (28 * MPH_FACTOR);
        }
        
        if ((value & 0x40) == 0)
        {
            velocity = -velocity;
        }
    }

    /** Get the speed in DCC 14 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  bit 7..6:  fixed at b'01'
     *  bit 5:  direction
     *  bits 4:  reserved 0 for headlight
     *  bits 3..0:  0 = stopped, 4 - 31 = speed steps 1 - 28 
     *  @return DCC encoded speed steps
     */
    uint8_t get_dcc_14()
    {
        uint8_t result;
        float tmp = ((speed() * MPH_FACTOR * 14) / 128) + 0.5;
        
        if (tmp == 0)
        {
            result = 0;
        }
        else if (tmp > 14)
        {
            result = 15;
        }
        else
        {
            result = (uint8_t)(tmp + 1);
        }
        
        result |= 0x40;

        result |= get_sign(velocity) == -1 ? 0x00 : 0x20;
        return result;
    }
    
    /** Set the speed from DCC 14 speed step format.
     *  This is a decimation of the 128 speed step mode.
     *
     *  @param value bit 7..6:  fixed at b'01'
     *               bit 5:  direction
     *               bits 4:  reserved 0 for headlight
     *               bits 3..0:  0 = stopped, 4 - 31 = speed steps 1 - 28 
     */
    void set_dcc_14(uint8_t value)
    {
        if ((value & 0x0F) <= 1)
        {
            velocity = 0;
        }
        else
        {
            velocity = (value & 0x0F) - 1;
            velocity *= 128;
            velocity /= (14 * MPH_FACTOR);
        }
        
        if ((value & 0x20) == 0)
        {
            velocity = -velocity;
        }
    }
    
    /** Get a wire version of the velocity.
     * @return IEEE half precision floating point representation of velocity
     */
    float16_t get_wire()
    {
        float16_t result;
        singles2halfp(&result, &velocity, 1);
        return result;
    }
    
    /** Set the value based on the wire version of velocity.
     * @param value IEEE half precision floating point representation of velocity
     */
    void set_wire(float16_t value)
    {
        halfp2singles(&velocity, &value, 1);
    }
    
    /** Overloaded addition operator. */
    Velocity operator + (const Velocity& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity + v.velocity, sign));
    }

    /** Overloaded addition operator. */
    Velocity operator + (const float& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity + v, sign));
    }

    /** Overloaded addition operator. */
    Velocity operator + (const double& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity + v, sign));
    }

    /** Overloaded addition operator. */
    Velocity operator + (const int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity + v, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator + (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity + v, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const Velocity& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity - v.velocity, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const float& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity - v, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const double& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity - v, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity - v, sign));
    }

    /** Overloaded subtraction operator. */
    Velocity operator - (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity - v, sign));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const Velocity& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity * v.velocity, sign));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const float& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity * v, sign));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const double& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity * v, sign));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity * v, sign));
    }

    /** Overloaded multiplication operator. */
    Velocity operator * (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity * v, sign));
    }

    /** Overloaded division operator. */
    Velocity operator / (const Velocity& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity / v.velocity, sign));
    }

    /** Overloaded division operator. */
    Velocity operator / (const float& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity / v, sign));
    }

    /** Overloaded division operator. */
    Velocity operator / (const double& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity / v, sign));
    }

    /** Overloaded division operator. */
    Velocity operator / (const int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity / v, sign));
    }

    /** Overloaded division operator. */
    Velocity operator / (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        return Velocity(zero_adjust(velocity / v, sign));
    }

    /** Overloaded pre-increement operator. */
    Velocity& operator ++ ()
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + 1, sign);
        return *this;
    }

    /** Overloaded post-increment operator. */
    Velocity operator ++ (int)
    {
        int sign = get_sign(velocity);

        Velocity result(*this);
        velocity = zero_adjust(velocity + 1, sign);
        return result;
    }

    /** Overloaded pre-decreement operator. */
    Velocity& operator -- ()
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - 1, sign);
        return *this;
    }

    /** Overloaded post-decrement operator. */
    Velocity operator -- (int)
    {
        int sign = get_sign(velocity);

        Velocity result(*this);
        velocity = zero_adjust(velocity - 1, sign);
        return result;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const Velocity& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + v.velocity, sign);
        return *this;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const float& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + v, sign);
        return *this;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const double& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + v, sign);
        return *this;
    }

    /** Overloaded addition equals operator. */
    Velocity& operator += (const int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + v, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator += (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity + v, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const Velocity& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - v.velocity, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const float& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - v, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const double& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - v, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - v, sign);
        return *this;
    }

    /** Overloaded subtraction equals operator. */
    Velocity& operator -= (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity - v, sign);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const Velocity& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity * v.velocity, sign);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const float& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity * v, sign);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const double& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity * v, sign);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity * v, sign);
        return *this;
    }

    /** Overloaded multiplication equals operator. */
    Velocity& operator *= (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity * v, sign);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const Velocity& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity / v.velocity, sign);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const float& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity / v, sign);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const double& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity / v, sign);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity / v, sign);
        return *this;
    }

    /** Overloaded division equals operator. */
    Velocity& operator /= (const unsigned int& v)
    {
        int sign = get_sign(velocity);

        velocity = zero_adjust(velocity / v, sign);
        return *this;
    }

private:
    /** Floating point representation of velocity. */
    float velocity;
    
    float zero_adjust(float value, int sign)
    {
        if (sign == -1 && value == 0)
        {
            return -0;
        }
    }
    
    int get_sign(float value)
    {
        if (value == -0 || value < 0)
        {
            return -1;
        }
        return 1;
    }
};


};
