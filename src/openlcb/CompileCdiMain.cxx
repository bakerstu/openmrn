#include <stdio.h>
#include <string>
using std::string;

#include "openlcb/ConfigRepresentation.hxx"
#include "config.hxx"

#include "utils/StringPrintf.cxx"
#include "utils/FileUtils.hxx"

bool raw_render = false;

// openlcb::ConfigDef def(0);

RENDER_CDI(openlcb, ConfigDef, "CDI", 1);

template <int N> void render_all_cdi()
{
    printf("// skipping config %d\n", N);
    render_all_cdi<N - 1>();
}

template <typename CdiType>
void render_cdi_helper(const CdiType &t, string ns, string name)
{
    string payload;
    t.config_renderer().render_cdi(&payload);
    if (raw_render)
    {
        string filename = name + ".xmlout";
        printf("Writing %d bytes to %s\n", (int)payload.size(),
            filename.c_str());
        write_string_to_file(filename, payload);
    }
    else
    {
        printf("namespace %s {\n\nextern const char %s_DATA[];\n", ns.c_str(),
            name.c_str());
        printf("// This is a C++11 raw string.\n");
        printf("const char %s_DATA[] = R\"xmlpayload(%s)xmlpayload\";\n",
            name.c_str(), payload.c_str());
        printf("extern const size_t %s_SIZE;\n", name.c_str());
        printf("const size_t %s_SIZE = sizeof(%s_DATA);\n", name.c_str(),
            name.c_str());
        printf("\n}  // namespace %s\n\n", ns.c_str());
    }
}

int main(int argc, char *argv[])
{
    if (argc > 1 && string(argv[1]) == "-r")
    {
        raw_render = true;
    }
    else
    {
        printf(R"(
/* Generated code based off of config.hxx */

#include <cstdint>
#include <unistd.h>

)");
    }

    render_all_cdi<10>();
    /*    render_all_cdi<9>();
        render_all_cdi<8>();
        render_all_cdi<7>();
        render_all_cdi<6>();
        render_all_cdi<5>();
        render_all_cdi<4>();
        render_all_cdi<3>();
        render_all_cdi<2>();
        render_all_cdi<1>();*/

    std::vector<unsigned> event_offsets;
    openlcb::ConfigDef def(0);
    def.handle_events([&event_offsets](unsigned o)
        {
            event_offsets.push_back(o);
        });
    printf("namespace openlcb {\nextern const uint16_t CDI_EVENT_OFFSETS[] = {\n  ");
    for (unsigned o : event_offsets)
    {
        printf("%u, ", o);
    }
    printf("0};\n}  // namespace openlcb\n");
    return 0;
}
