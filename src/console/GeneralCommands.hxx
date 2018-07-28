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
 * @file GeneralCommands.hxx
 * General commands.
 *
 * @author Stuart W. Baker
 * @date 29 April 2018
 */

#ifndef _CONSOLE_GENERALCOMMANDS_HXX_
#define _CONSOLE_GENERALCOMMANDS_HXX_

#include "console/Console.hxx"

/// Revision information
extern const char *REVISIONS[];

/// Container for all the general commands.
/// This class can be used by intantiating an instance of GeneralCommands and
/// passing to the constructor a @ref Console instance reference.  The commands
/// implemented by GeneralCommands will be added to the @ref Console instance.
class GeneralCommands
{
public:
    /// Constructor.
    /// @param console console instance to add the commands to
    GeneralCommands(Console *console)
    {
        console->add_command("version", version_command);
    }

private:
    /// Concatinate files.
    /// @param fp file pointer to console
    /// @param argc number of arguments including the command itself
    /// @param argv array of arguments starting with the command itself
    /// @param context unused
    /// @return COMMAND_OK
    static Console::CommandStatus version_command(FILE *fp, int argc,
                                                  const char *argv[],
                                                  void *context)
    {
        if (argc == 0)
        {
            fprintf(fp, "print all compiler and repository revisions\n");
            return Console::COMMAND_OK;
        }

        if (REVISIONS[0] == nullptr)
        {
            fprintf(fp, "%s: No revision history\n", argv[0]);
        }

        for (int i = 0; REVISIONS[i] != nullptr; ++i)
        {
            fprintf(fp, "%s\n", REVISIONS[i]);
        }

        return Console::COMMAND_OK;
    }

    DISALLOW_COPY_AND_ASSIGN(GeneralCommands);
};

#endif // _CONSOLE_GENERALCOMMANDS_HXX_
