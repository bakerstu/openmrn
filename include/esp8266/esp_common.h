#ifndef _ESP_COMMON_H_
#define _ESP_COMMON_H_

// Prevent including the freertosconfig
//#define INC_FREERTOS_H

#include "c_types.h"

typedef uint8_t  u8_t;
typedef int8_t   s8_t;
typedef uint16_t u16_t;
typedef int16_t  s16_t;
typedef uint32 u32_t;
typedef int32  s32_t;

#include "spiffs_config.h"
#include "espressif/esp_spiffs.h"
#include <spi_flash.h>

#endif //  _ESP_COMMON_H_
