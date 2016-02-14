#include <stdio.h>
#include <string>
using std::string;

#include "nmranet/ConfigRepresentation.hxx"
#include "config.hxx"

#include "utils/StringPrintf.cxx"

//nmranet::ConfigDef def(0);

RENDER_CDI(nmranet, ConfigDef, "CDI", 1);

template <typename CdiType>
void render_cdi_helper(const CdiType &t, string ns, string name)
{
    string payload;
    t.config_renderer().render_cdi(&payload);
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

int main(int argc, char *argv[])
{
    printf(R"(
/* Generated code based off of config.hxx */

#include <cstdint>
#include <unistd.h>

)");

    CdiRenderHelper<20>::render_cdi();

    /*
      string cdi;
    def.config_renderer().render_cdi(&cdi);

    printf("%s", cdi.c_str());

    std::vector<unsigned> event_offsets;
    def.handle_events([&event_offsets](unsigned o)
        {
            event_offsets.push_back(o);
        });
    printf("<!-- events: ");
    for (unsigned o : event_offsets)
    {
        printf("%u,", o);
    }
    printf("-->\n");
    */
    return 0;
}
