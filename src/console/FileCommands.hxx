/** @copyright
 * Copyright (c) 2018, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file FileCommands.hxx
 * This console commands for basic file system operations.
 *
 * @author Stuart W. Baker
 * @date 20 January 2018
 */

#ifndef _CONSOLE_FILECOMMANDS_HXX_
#define _CONSOLE_FILECOMMANDS_HXX_

#include "console/Console.hxx"

/// Container for all the file system operations.
class FileCommands
{
public:
    /// Constructor.
    /// @param console console instance to add the commands to
    FileCommands(Console *console)
    {
        console->add_command("cat", cat_command);
        console->add_command("echo", echo_command);
    }

private:

    /// Concatinate files.
    /// @param fp file pointer to console
    /// @param argc number of arguments including the command itself
    /// @param argv array of arguments starting with the command itself
    /// @param context unused
    /// @return COMMAND_OK
    static Console::CommandStatus cat_command(FILE *fp, int argc,
                                              const char *argv[], void *context)
    {
        if (argc == 0)
        {
            fprintf(fp, "concatenate files and print on the standard output\n");
            return Console::COMMAND_OK;
        }

        for (int i = 1; i < argc; ++i)
        {
            int fd = ::open(argv[i], O_RDONLY | O_NONBLOCK);
            if (fd < 0)
            {
                fprintf(fp, "%s: %s: No such file or directory\n",
                        argv[0], argv[i]);
                continue;
            }
            ssize_t result;
            do
            {
                char buf[32];
                result = ::read(fd, buf, sizeof(buf));
                if (result > 0)
                {
                    fprintf(fp, "%.*s", result, buf);
                }
            } while (result > 0);

            close(fd);

            fprintf(fp, "\n");
        }

        return Console::COMMAND_OK;
    }

    /// Echo string.
    /// @param fp file pointer to console
    /// @param argc number of arguments including the command itself
    /// @param argv array of arguments starting with the command itself
    /// @param context unused
    /// @return COMMAND_OK
    static Console::CommandStatus echo_command(FILE *fp, int argc,
                                               const char *argv[],
                                               void *context)
    {
        if (argc == 0)
        {
            fprintf(fp, "display a line of text\n");
            return Console::COMMAND_OK;
        }

        if (argc == 2)
        {
            fprintf(fp, "%s\n", argv[1]);
            return Console::COMMAND_OK;
        }
        else if (argc == 4)
        {
            if (!strcmp(argv[2], ">"))
            {
                int fd = ::open(argv[3], O_WRONLY | O_CREAT);
                if (fd < 0)
                {
                    fprintf(fp, "%s: No such file or directory\n", argv[3]);
                }
                else
                {
                    ::write(fd, argv[1], strlen(argv[1]));
                    close(fd);
                    return Console::COMMAND_OK;
                }
            }
        }

        return Console::COMMAND_ERROR;
    }


#if 0
    /// "cat" console command
    class CatFlow : public Console::CommandFlow
    {
    public:
        /// Constructor.
        /// @param console console instance to add the commands to
        CatFlow(Console *console)
            : CommandFlow(console, "cat")
        {
        }

    private:
        StateFlowBase::Action entry() override
        {
            int fd = ::open
            fprintf(fp, "param: ");
            return wait_for_line_and_call(STATE(input));
        }

        StateFlowBase::Action input()
        {
            EXPECT_EQ(CommandFlow::argc, 1);
            EXPECT_TRUE(!strcmp(argv[0], "example"));
            return record_status_and_exit(Console::COMMAND_OK);
        }

        DISALLOW_COPY_AND_ASSIGN(CatFlow);
    };

    /// "cat" command instance
    CatFlow cat;
#endif

    DISALLOW_COPY_AND_ASSIGN(FileCommands);
};

#endif // _CONSOLE_FILECOMMANDS_HXX_
