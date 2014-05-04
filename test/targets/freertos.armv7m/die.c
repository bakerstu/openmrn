#include "FreeRTOSConfig.h"

/** Never returns. used for fake targets like an emulator. Real boards should
 * do some kind of LED flashing here. */
void diewith(unsigned long pattern) {
    while(1);
}
