/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file GPIOGeneric.hxx
 *
 * Generic implementation header for GPIO.
 *
 * @author Stuart Baker
 * @date 1 July 2015
 */

#include "GPIOGeneric.hxx"

#include "os/os.h"

Gpio *Gpio::first = nullptr;

/** Constructor.
 * @param GPIO number
 * @param mode GPIO mode settings
 * @param safe default "safe" value, may go unused for input only pins
 */
Gpio::Gpio(unsigned number, Mode mode, Value safe)
        : pin(number)
        , bit(0)
        , safeValue(safe == CLR ? 0 : 1)
        , invert(mode & INVERT ? 1 : 0)
        , next(nullptr)
{
}

/** Destructor.
 */
Gpio::~Gpio()
{
    /* we use the schedualer lock as a poor man's mutex.  This sequence allows
     * for static or dynamic construction.
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    Gpio *current = first;
    Gpio *last = nullptr;
    while (current)
    {
        /* make sure we are not constructing this I/O twice */
        if (current->pin == pin)
        {
            if (last)
            {
                last->next = next;
            }
            else
            {
                first = next;
            }
            break;
        }
        last = current;
        current->next = last;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }
}


/** Add the GPIO number to our list of known GPIO.
 */
void Gpio::track()
{
    /* we use the schedualer lock as a poor man's mutex.  This sequence allows
     * for static or dynamic construction.
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    /* make sure we are not constructing this I/O twice */
    Gpio *current = first;
    while (current)
    {
        HASSERT(current->pin != pin);
        current = current->next;
    }

    /* register GPIO in the system */
    next = first;
    first = this;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }
}

/** Find a GPIO by referencing its number.
 * @return Gpio instance pointer if found, else nullptr if not found
 */
Gpio *Gpio::find(unsigned number)
{
    /* we use the schedualer lock as a poor man's mutex.  This sequence allows
     * for static or dynamic construction.
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    Gpio *current = first;
    while (current)
    {
        /* make sure we are not constructing this I/O twice */
        if (current->pin == number)
        {
            break;
        }
        current = current->next;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }

    return current;
}
