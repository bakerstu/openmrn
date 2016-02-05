#include <stdio.h>
#include <string>
using std::string;

#include "config.hxx"

#include "utils/StringPrintf.cxx"

nmranet::ConfigDef def(0);

int main(int argc, char *argv[])
{
    string cdi;
    def.config_renderer().render_cdi(&cdi);

    printf("%s", cdi.c_str());

    std::vector<unsigned> event_offsets;
    def.handle_events([&event_offsets](unsigned o)
        {
            event_offsets.push_back(o);
        });
    printf("<!-- events: ");
    for (unsigned o : event_offsets) {
        printf("%u,", o);
    }
    printf("-->\n");

    return 0;
}
