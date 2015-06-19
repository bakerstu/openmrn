#include <stdio.h>
#include <string>
using std::string;

#include "config.hxx"
#include "definitions.hxx"

#include "utils/StringPrintf.cxx"

nmranet::ConfigDef def(0);

int main(int argc, char *argv[]) {
    string cdi;
    def.config_renderer().render_cdi(&cdi);

    printf("%s", cdi.c_str());

    return 0;
}
