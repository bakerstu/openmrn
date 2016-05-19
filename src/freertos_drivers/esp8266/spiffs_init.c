#include "espressif/esp_spiffs.h"


#define FS1_FLASH_SIZE      (128*1024)
#define FS2_FLASH_SIZE      (128*1024)

#define FS1_FLASH_ADDR      (1024*1024)
#define FS2_FLASH_ADDR      (1280*1024)

#define SECTOR_SIZE         (4*1024) 
#define LOG_BLOCK           (SECTOR_SIZE)
#define LOG_PAGE            (128)

#define FD_BUF_SIZE         32*4
#define CACHE_BUF_SIZE      (LOG_PAGE + 32)*8


void ICACHE_FLASH_ATTR spiffs_init()
{
    struct esp_spiffs_config config;

    config.phys_size = 128 * 1024;
    config.phys_addr = 1024 * 1024;
    config.phys_erase_block = 4*1024;
    config.log_block_size = 4*1024;
    config.log_page_size = 128;
    config.fd_buf_size = FD_BUF_SIZE * 2;
    config.cache_buf_size = CACHE_BUF_SIZE;

    int r = esp_spiffs_init(&config);
    printf("spiffs: %d\n", r);
}
