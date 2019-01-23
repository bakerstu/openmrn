/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32CANDRIVER_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32CANDRIVER_HXX_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>

/** \brief CAN Node Bus speed */
typedef enum {
	CAN_SPEED_100KBPS = 100,  /**< \brief CAN Node runs at 100kBit/s. */
	CAN_SPEED_125KBPS = 125,  /**< \brief CAN Node runs at 125kBit/s. */
	CAN_SPEED_200KBPS = 200,  /**< \brief CAN Node runs at 200kBit/s. */
	CAN_SPEED_250KBPS = 250,  /**< \brief CAN Node runs at 250kBit/s. */
	CAN_SPEED_500KBPS = 500,  /**< \brief CAN Node runs at 500kBit/s. */
	CAN_SPEED_800KBPS = 800,  /**< \brief CAN Node runs at 800kBit/s. */
	CAN_SPEED_1000KBPS = 1000 /**< \brief CAN Node runs at 1000kBit/s. */
} CAN_speed_t;

/**
 * \brief CAN frame type (standard/extended)
 */
typedef enum {
	CAN_frame_std = 0, /**< Standard frame, using 11 bit identifer. */
	CAN_frame_ext = 1  /**< Extended frame, using 29 bit identifer. */
} CAN_frame_format_t;

/**
 * \brief CAN RTR
 */
typedef enum {
	CAN_no_RTR = 0, /**< No RTR frame. */
	CAN_RTR = 1     /**< RTR frame. */
} CAN_RTR_t;

/** \brief Frame information record type */
typedef union {
	uint32_t U; /**< \brief Unsigned access */
	struct {
		uint8_t DLC : 4;               /**< \brief [3:0] DLC, Data length container */
		unsigned int unknown_2 : 2;    /**< \brief \internal unknown */
		CAN_RTR_t RTR : 1;             /**< \brief [6:6] RTR, Remote Transmission Request */
		CAN_frame_format_t FF : 1;     /**< \brief [7:7] Frame Format, see# CAN_frame_format_t*/
		unsigned int reserved_24 : 24; /**< \brief \internal Reserved */
	} B;
} CAN_FIR_t;

/** \brief CAN Frame structure */
typedef struct {
	CAN_FIR_t FIR;  /**< \brief Frame information record*/
	uint32_t MsgID; /**< \brief Message ID */
	union {
		uint8_t u8[8];   /**< \brief Payload byte access*/
		uint32_t u32[2]; /**< \brief Payload u32 access*/
	} data;
} CAN_frame_t;

/**
 * \brief Initialize the CAN Module
 *
 * \return 0 CAN Module had been initialized
 */
QueueHandle_t CAN_init(gpio_num_t tx_pin_id, gpio_num_t rx_pin_id, CAN_speed_t speed, uint8_t rx_buffer_size);

void CAN_deinit();

/**
 * \brief Send a can frame
 *
 * \param	p_frame	Pointer to the frame to be send, see #CAN_frame_t
 * \return  0 Frame has been written to the module
 */
int CAN_write_frame(const CAN_frame_t *p_frame);

/**
 * \brief Starts the CAN Module
 *
 * \return 0 CAN Module was started
 */
int CAN_start(void);

/**
 * \brief Stops the CAN Module
 *
 * \return 0 CAN Module was stopped
 */
int CAN_stop(void);

/**
 * \brief Stops and resets the CAN controller.
 * 
 * \return 0 CAN controller was reset.
 */
int CAN_reset(void);

#endif /* _FREERTOS_DRIVERS_ESP32_ESP32CANDRIVER_HXX_ */