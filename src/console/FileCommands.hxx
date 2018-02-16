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

#include <dirent.h>

#include "console/Console.hxx"

/// Container for all the file system operations.
/// This class can be used by intantiating an instance of FileCommands and
/// passing to the constructor a @ref Console instance reference.  The commands
/// implemented by FileCommands will be added to the @ref Console instance.
class FileCommands
{
public:
    /// Constructor.
    /// @param console console instance to add the commands to
    FileCommands(Console *console)
    {
        console->add_command("cat", cat_command);
        console->add_command("echo", echo_command);
        console->add_command("rm", rm_command);
        console->add_command("ls", ls_command);
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
                    ssize_t wr_size = strlen(argv[1]);
                    ssize_t result = ::write(fd, argv[1], wr_size);
                    close(fd);
                    return result == wr_size ? Console::COMMAND_OK :
                                               Console::COMMAND_ERROR;
                }
            }
        }

        return Console::COMMAND_ERROR;
    }

    /// Remove files or directories.
    /// @param fp file pointer to console
    /// @param argc number of arguments including the command itself
    /// @param argv array of arguments starting with the command itself
    /// @param context unused
    /// @return COMMAND_OK
    static Console::CommandStatus rm_command(FILE *fp, int argc,
                                             const char *argv[], void *context)
    {
        if (argc == 0)
        {
            fprintf(fp, "remove files or directories\n");
            return Console::COMMAND_OK;
        }

        if (argc < 2)
        {
            return Console::COMMAND_ERROR;
        }

        for (int i = 1; i < argc; ++i)
        {
            if (::unlink(argv[i]) == 0)
            {
                continue;
            }
            if (errno == ENOENT)
            {
                fprintf(fp,
                        "%s: cannot remove '%s': No such file or directory\n",
                        argv[0], argv[i]);
            }
        }

        return Console::COMMAND_OK;
    }

    /// List directory contents.
    /// @param fp file pointer to console
    /// @param argc number of arguments including the command itself
    /// @param argv array of arguments starting with the command itself
    /// @param context unused
    /// @return COMMAND_OK
    static Console::CommandStatus ls_command(FILE *fp, int argc,
                                             const char *argv[], void *context)
    {
        if (argc == 0)
        {
            fprintf(fp, "list directory contents\n");
            return Console::COMMAND_OK;
        }

        if (argc != 2)
        {
            return Console::COMMAND_ERROR;
        }

        struct stat stat;
        if (::stat(argv[1], &stat) != 0)
        {
            if (errno == ENOENT)
            {
                fprintf(fp,
                        "%s: cannot access '%s': No such file or directory\n",
                        argv[0], argv[1]);
            }
            return Console::COMMAND_ERROR;
        }

        if (S_ISDIR(stat.st_mode))
        {
            DIR *dir = opendir(argv[1]);
            HASSERT(dir);
            struct dirent *dirent;
            do
            {
                dirent = readdir(dir);
                if (dirent)
                {
                    HASSERT(::stat(dirent->d_name, &stat) == 0);
                    ls_printline(fp, dirent->d_name, &stat);
                }
            } while (dirent);
            closedir(dir);
        }
        else if (S_ISREG(stat.st_mode) || S_ISLNK(stat.st_mode))
        {
            ls_printline(fp, argv[1], &stat);
        }

        return Console::COMMAND_OK;
    }

    /// Helper method to print a single "ls" line.
    /// @param fp file pointer to console
    /// @param name name of the entry
    /// @param stat status information about the entry
    static void ls_printline(FILE *fp, const char *name, struct stat *stat)
    {
        char type;
        type = S_ISDIR(stat->st_mode) ? 'd' : '-';
        type = S_ISLNK(stat->st_mode) ? 'l' : type;

        fprintf(fp, "%c%c%c%c%c%c%c%c%c%c %5ld %s\n", type,
                stat->st_mode & S_IRUSR ? 'r' : '-',
                stat->st_mode & S_IWUSR ? 'w' : '-',
                stat->st_mode & S_IXUSR ? 'x' : '-',
                stat->st_mode & S_IRGRP ? 'r' : '-',
                stat->st_mode & S_IWGRP ? 'w' : '-',
                stat->st_mode & S_IXGRP ? 'x' : '-',
                stat->st_mode & S_IROTH ? 'r' : '-',
                stat->st_mode & S_IWOTH ? 'w' : '-',
                stat->st_mode & S_IXOTH ? 'x' : '-',
                stat->st_size, name);
    }

    DISALLOW_COPY_AND_ASSIGN(FileCommands);
};

#endif // _CONSOLE_FILECOMMANDS_HXX_
