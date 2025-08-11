/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file FeatherCan.hxx
 *
 * Wrapper for Arduino CAN driver for the Feather M4 CAN board using the
 * external ACANFD_FeatherM4CAN library.
 *
 * @author Balazs Racz
 * @date 29 July 2025
 */

#ifndef _ARDUINO_SAM_FEATHERM4CAN_HXX_
#define _ARDUINO_SAM_FEATHERM4CAN_HXX_

#ifndef ARDUINO_FEATHER_M4_CAN
#error "This code only works on the Arduino Feather M4 CAN (SAME51)"
#endif

#include <ACANFD_FeatherM4CAN-from-cpp.h>

#include "freertos_drivers/common/Can.hxx"

/// Wrapper for Arduino CAN driver for the Feather M4 CAN board using the
/// external ACANFD_FeatherM4CAN library.
/// Usage:
///
/// // These are required to get the right CAN setup working.
/// #define CAN0_MESSAGE_RAM_SIZE (0)
/// #define CAN1_MESSAGE_RAM_SIZE (1728)
/// #include <ACANFD_FeatherM4CAN.h>
///
/// #include <OpenMRNLite.h>
/// FeatherM4Can can_driver;
///
/// void setup() {
///   ...
///   HASSERT(can_driver.begin());
///   openmrn.add_can_port(&can_driver);
///   openmrn.begin();
///   ...
/// }
///
class FeatherM4Can : public openmrn_arduino::Can
{
public:
    FeatherM4Can(ACANFD_FeatherM4CAN &can = can1)
        : openmrn_arduino::Can("")
        , can_(&can)
    {
    }

    /// Initializes the CAN-bus driver.
    /// @return true if successful prints and returns false on error.
    bool begin()
    {
        const uint32_t errorCode = can1.beginFD(get_settings());

        if (0 == errorCode)
        {
            return true;
        }
        else
        {
            Serial.print("Error can configuration: 0x");
            Serial.println(errorCode, HEX);
            return false;
        }
    }

    ACANFD_FeatherM4CAN_Settings get_settings()
    {
        return ACANFD_FeatherM4CAN_Settings(
            ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz, 125 * 1000,
            DataBitRateFactor::x1);
    }

    /// Prints the settings to Serial. Should be called after begin().
    void print_settings()
    {
        auto settings = get_settings();

        Serial.print("Bit Rate prescaler: ");
        Serial.println(settings.mBitRatePrescaler);
        Serial.print("Arbitration Phase segment 1: ");
        Serial.println(settings.mArbitrationPhaseSegment1);
        Serial.print("Arbitration Phase segment 2: ");
        Serial.println(settings.mArbitrationPhaseSegment2);
        Serial.print("Arbitration SJW: ");
        Serial.println(settings.mArbitrationSJW);
        Serial.print("Actual Arbitration Bit Rate: ");
        Serial.print(settings.actualArbitrationBitRate());
        Serial.println(" bit/s");
        Serial.print("Arbitration Sample point: ");
        Serial.print(settings.arbitrationSamplePointFromBitStart());
        Serial.println("%");
        Serial.print("Exact Arbitration Bit Rate ? ");
        Serial.println(settings.exactArbitrationBitRate() ? "yes" : "no");
        Serial.print("Data Phase segment 1: ");
        Serial.println(settings.mDataPhaseSegment1);
        Serial.print("Data Phase segment 2: ");
        Serial.println(settings.mDataPhaseSegment2);
        Serial.print("Data SJW: ");
        Serial.println(settings.mDataSJW);
        Serial.print("Actual Data Bit Rate: ");
        Serial.print(settings.actualDataBitRate());
        Serial.println(" bit/s");
        Serial.print("Data Sample point: ");
        Serial.print(settings.dataSamplePointFromBitStart());
        Serial.println("%");
        Serial.print("Exact Data Bit Rate ? ");
        Serial.println(settings.exactDataBitRate() ? "yes" : "no");

        Serial.print("Message RAM required minimum size: ");
        Serial.print(can().messageRamRequiredMinimumSize());
        Serial.println(" words");
    }

    ACANFD_FeatherM4CAN &can()
    {
        return *can_;
    }

    /// @return number of CAN frames available for read (input frames).
    int available() override
    {
        return can().availableFD0() ? 1 : 0;
    }

    /// @return number of CAN frames available for write (space in output
    /// buffer).
    int availableForWrite() override
    {
        return can().sendBufferNotFullForIndex(0) ? 1 : 0;
    }

    /// Reads a frame if there is one available.
    /// @param frame will be filled with the input CAN frame.
    /// @return 0 or 1 depending on whether a frame was read or not.
    int read(struct can_frame *frame) override
    {
        CANFDMessage fdframe;
        if (!can().receiveFD0(fdframe))
        {
            return 0;
        }
        if (fdframe.type != CANFDMessage::CAN_DATA)
        {
            return 0;
        }
        CLR_CAN_FRAME_RTR(*frame);
        CLR_CAN_FRAME_ERR(*frame);
        if (fdframe.ext)
        {
            SET_CAN_FRAME_ID_EFF(*frame, fdframe.id);
            SET_CAN_FRAME_EFF(*frame);
        }
        else
        {
            SET_CAN_FRAME_ID(*frame, fdframe.id);
            CLR_CAN_FRAME_EFF(*frame);
        }
        frame->can_dlc = std::min(8u, (unsigned)fdframe.len);
        frame->data64 = fdframe.data64[0];
        return 1;
    }

    /// Send a frame if there is space available.
    /// @param frame the output CAN frame.
    /// @return 0 or 1 depending on whether the write happened or not.
    int write(const struct can_frame *frame) override
    {
        if (availableForWrite() == 0)
        {
            return 0;
        }
        CANFDMessage fdframe;
        // Populates type.
        if (IS_CAN_FRAME_RTR(*frame))
        {
            fdframe.type = CANFDMessage::CAN_REMOTE;
        }
        else if (IS_CAN_FRAME_ERR(*frame))
        {
            return 1; // drop to the floor.
        }
        else
        {
            fdframe.type = CANFDMessage::CAN_DATA;
        }
        // Populates ID.
        if (IS_CAN_FRAME_EFF(*frame))
        {
            fdframe.ext = true;
            fdframe.id = GET_CAN_FRAME_ID_EFF(*frame);
        }
        else
        {
            fdframe.ext = false;
            fdframe.id = GET_CAN_FRAME_ID(*frame);
        }
        fdframe.len = frame->can_dlc;
        fdframe.data64[0] = frame->data64;
        const uint32_t send_status = can().tryToSendReturnStatusFD(fdframe);
        return (send_status == 0) ? 1 : 0;
    }

private:
    // Not used.
    void enable() override
    {
    }
    // Not used.
    void disable() override
    {
    }
    // Not used.
    void tx_msg() override
    {
    }

    /// Pointer to the underlying library's CAN implementation.
    ACANFD_FeatherM4CAN *can_;
}; // class FeatherM4Can

#endif // _ARDUINO_SAM_FEATHERM4CAN_HXX_
