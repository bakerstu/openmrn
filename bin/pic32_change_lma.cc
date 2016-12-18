#include <stdio.h>
#include <stdlib.h>
#include <string>

using std::string;

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
    while (true)
    {
        string line = next_line();
        if (line.find(" bf") == string::npos &&
            line.find(" bd") == string::npos)
        {
            continue;
        }
        string section_name = line.substr(4, line.find(' ', 4) - 4);
        printf("--change-section-lma %s-0xA0000000 ", section_name.c_str());
    }
    return 0;
}
