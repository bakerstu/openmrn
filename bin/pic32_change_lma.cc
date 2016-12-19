#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using std::string;
using std::vector;

char linebuf[1000];

string next_line()
{
    string ret;
    char *l = fgets(linebuf, sizeof(linebuf), stdin);
    if (!l)
    {
        exit(0);
    }
    ret = l;
    return ret;
}

int main(int argc, char *argv[])
{
    vector<string> shift;
    shift.push_back("--change-section-lma %s-0x80000000");
    if (argc > 1) {
        shift.clear();
        for (int i = 1; i < argc; ++i) {
            shift.push_back(argv[i]);
        }
    }
    //fprintf(stderr, "num shift %u\n", (unsigned)shift.size());
    while (true)
    {
        string line = next_line();
        if (line.find(" 9f") == string::npos &&
            line.find(" 9d") == string::npos)
        {
            continue;
        }
        string section_name = line.substr(4, line.find(' ', 4) - 4);
        for (unsigned i = 0; i < shift.size(); ++i) {
            const string& sh = shift[i];
            printf(sh.c_str(), section_name.c_str());
            printf(" ");
        }
    }
    return 0;
}
