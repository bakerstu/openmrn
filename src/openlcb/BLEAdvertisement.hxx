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
 * @file BLEAdvertisement.hxx
 *
 * OpenLCB BLE Advertisement definition.
 *
 * @author Stuart Baker
 * @date 9 March 2024
 */

#ifndef _OPENLCB_BLEADVERTISEMENT_HXX_
#define _OPENLCB_BLEADVERTISEMENT_HXX_

#include "ble/Advertisement.hxx"
#include "openlcb/Defs.hxx"

namespace openlcb
{

/// OpenLCB BLE advertisement defintion.
class BLEAdvertisement : public ble::Advertisement
{
public:
    /// Constructor.
    /// @param node_id Device node ID that will be used in the advertisement.
    /// @param node_name node name that will be used in the advertisement.
    /// @param pip supported OpenLCB protocols
    BLEAdvertisement(NodeID node_id, const char *node_name, uint32_t pip);
};

} // namespace openlcb

#endif // _OPENLCB_BLEADVERTISEMENT_HXX_
