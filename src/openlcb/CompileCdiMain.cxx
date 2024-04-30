#include <stdio.h>
#include <string>
using std::string;

#include "openlcb/ConfigRepresentation.hxx"
#include "config.hxx"

#include "utils/StringPrintf.cxx"
#include "utils/FileUtils.cxx"

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
        // Adds trailing zero to the file written.
        payload.push_back(0);
        // Writes the file.
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
        printf("extern const size_t %s_SIZE = sizeof(%s_DATA);\n", name.c_str(),
            name.c_str());
        printf("extern const size_t %s_END_OFFSET = %u;\n", name.c_str(),
               (unsigned)t.end_offset());
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

    // Internally calls all smaller numbered instances all the way down to 1.
    render_all_cdi<20>();

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
