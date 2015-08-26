#include "os/os.h"
#include "utils/blinker.h"

int appl_main(int argc, char*argv[]) {
    resetblink(0xF280);
    while(true) {}
    return 0;
}
