/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file Can.hxx
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#ifndef _Can_hxx_
#define _Can_hxx_

#include "Devtab.hxx"
#include "os/OS.hxx"
#include "nmranet_can.h"

/** Number of receive CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_RX_BUFFER_SIZE;

/** Number of transmit CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_TX_BUFFER_SIZE;

/** Private data for a can device */
class Can : public Node {
 protected:
  /** Constructor
   * @param name device name in file system
   */
  Can(const char *name)
      : Node(),
        txQ(os_mq_create(CAN_TX_BUFFER_SIZE, sizeof(struct can_frame))),
        rxQ(os_mq_create(CAN_RX_BUFFER_SIZE, sizeof(struct can_frame))),
        overrunCount(0),
        read_callback(NULL),
        write_callback(NULL),
        readContext(NULL),
        writeContext(NULL),
        mutex(),
        devtab(name, &ops, this) {}

  /** Destructor.
   */
  ~Can() {
    /** @todo (Stuart Baker) for completeness we should destroy the
     * txQ and rxQ here.
     */
  }

  virtual void enable() = 0;  /**< function to enable device */
  virtual void disable() = 0; /**< function to disable device */
  virtual void tx_msg() = 0;  /**< function to try and transmit a message */

  os_mq_t txQ;                    /**< transmit queue */
  os_mq_t rxQ;                    /**< receive queue */
  unsigned int overrunCount;      /**< overrun count */
  void (*read_callback)(void *);  /**< callback for read active notify */
  void (*write_callback)(void *); /**< callback for write active notify */
  void *readContext;  /**< callback argument for read active notify */
  void *writeContext; /**< callback argument for write active notify */

 private:
  /** Open a device.
  * @param file new file reference to this device
  * @param path file or device name
  * @param flags open flags
  * @param mode open mode
  * @return 0 upon success, negative errno upon failure
  */
  static int open(File *file, const char *path, int flags, int mode);

  /** Close a device.
  * @param file file reference for this device
  * @param node node reference for this device
  * @return 0 upon success, negative errno upon failure
  */
  static int close(File *file, Node *node);

  /** Read from a file or device.
  * @param file file reference for this device
  * @param buf location to place read data
  * @param count number of bytes to read
  * @return number of bytes read upon success, -1 upon failure with errno
  * containing the cause
  */
  static ssize_t read(File *file, void *buf, size_t count);

  /** Write to a file or device.
  * @param file file reference for this device
  * @param buf location to find write data
  * @param count number of bytes to write
  * @return number of bytes written upon success, -1 upon failure with errno
  * containing the cause
  */
  static ssize_t write(File *file, const void *buf, size_t count);

  /** Request an ioctl transaction
  * @param file file reference for this device
  * @param node node reference for this device
  * @param key ioctl key
  * @param data key data
  */
  static int ioctl(File *file, Node *node, unsigned long int key,
                   unsigned long data);

  OSMutex mutex;     /**< mutual exclusion for the device */
  Devtab devtab;     /**< device tabel entry for this instance */
  static Devops ops; /**< device operations for CAN */

  /** Default constructor.
   */
  Can();

  DISALLOW_COPY_AND_ASSIGN(Can);
};

#endif /* _Can_hxx_ */
