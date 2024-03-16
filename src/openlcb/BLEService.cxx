/** @copyright
 * Copyright (c) 2024, Stuart Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file BLEService.hxx
 *
 * OpenLCB BLE Service definition.
 *
 * @author Stuart Baker
 * @date 2 March 2024
 */

#include "openlcb/BLEService.hxx"

namespace openlcb
{

// 0ff45220-84a9-4daf-83e7-da4c828d1851
const uint8_t BLEService::GATT_SERVICE_UUID_OPENLCB[16] =
{
    0x51, 0x18, 0x8D, 0x82, 0x4C, 0xDA,
    0xE7, 0x83,
    0xAF, 0x4D,
    0xA9, 0x84,
    0x20, 0x52, 0xF4, 0x0F,
};

// b029e1ba-47bc-4d8a-a28e-fbe167fc06cb
const uint8_t BLEService::GATT_CHAR_UUID_DATA_IN[16] =
{
    0xCB, 0x06, 0xfC, 0x67, 0xE1, 0xFB,
    0x8E, 0xA2,
    0x8A, 0x4D,
    0xBC, 0x47,
    0xBA, 0xE1, 0x29, 0xB0,
};

// c4fed917-55d9-485d-bf30-2eb3c095a90f
const uint8_t BLEService::GATT_CHAR_UUID_DATA_OUT[16] =
{
    0x0F, 0xA9, 0x95, 0xC0, 0xB3, 0x2E,
    0x30, 0xBF,
    0x5D, 0x48,
    0xD9, 0x55,
    0x17, 0xD9, 0xFE, 0xC4,
};

uint8_t BLEService::dataIn_[200];
uint8_t BLEService::dataOut_[200];
uint8_t BLEService::dataInCCCD_[2] = {0x00, 0x00};
uint8_t BLEService::dataOutCCCD_[2] = {0x00, 0x00};


} // openlcb
