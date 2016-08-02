/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file Console.cxx
 * This file provides an implementation of an interactive text console.
 *
 * @author Stuart W. Baker
 * @date 10 May 2014
 */

#include "console/Console.hxx"

#if defined (CONSOLE_NETWORKING)
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

/*
 * Console::Console()
 */
Console::Console(ExecutorBase *executor, uint16_t port)
    : Service(executor)
    , help("help", help_command, this, &helpMark)
    , helpMark("?", help_command, this)
#if defined (CONSOLE_NETWORKING)
    , listen(this, port)
#endif
{
    add_command("quit", quit_command, this);
}

/*
 * Console::Console()
 */
Console::Console(ExecutorBase *executor, int fd_in, int fd_out, int port)
    : Service(executor)
    , help("help", help_command, this, &helpMark)
    , helpMark("?", help_command, this)
#if defined (CONSOLE_NETWORKING)
    , listen(this, port)
#endif
{
    open_session(fd_in, fd_out);
    add_command("quit", quit_command, this);
}

/*
 * Console::open_session()
 */
void Console::open_session(int fd_in, int fd_out)
{
#ifdef HAVE_BSDSOCKET
    fcntl(fd_in, F_SETFL, fcntl(0, F_GETFL, 0) | O_NONBLOCK);
#endif
    new Session(this, fd_in, fd_out);
}

/*
 * Console::add_command()
 */
void Console::add_command(const char *name, Callback callback, void *context)
{
    Command *current = &helpMark;

    /* seek to the end of the list */
    while (current->next)
    {
        current = current->next;
    }

    /** add the command to the end of the list */
    current->next = new Command(name, callback, context);
}

/*
 * Console::help_command()
 */
Console::CommandStatus Console::help_command(FILE *fp, int argc, const char *argv[])
{
    fprintf(fp, "%10s : print out this help menu\n", "help | ?");    

    /* call each of the commands with argc = 0 */
    for (Command *current = helpMark.next; current; current = current->next)
    {
        fprintf(fp, "%10s : ", current->name);
        const char *argv[2] = {current->name, "             "};
        (*current->callback)(fp, 0, argv, NULL);
    }

    return COMMAND_OK;
}

/*
 * Console::quit_command()
 */
Console::CommandStatus Console::quit_command(FILE *fp, int argc, const char *argv[])
{
    switch (argc)
    {
        case 0:
            fprintf(fp, "terminate the current login session, only\n%s"
                        "has an effect on socket based logins sessions\n",
                    argv[1]);
            return COMMAND_OK; 
        case 1:
            return COMMAND_CLOSE;
        default:
            return COMMAND_ERROR;
    }
}

/*
 * Console::CommandFlow::CommandFlow()
 */
Console::CommandFlow::CommandFlow(Console *console, const char *name)
    : StateFlowBase(console)
{
    Command *current = &console->helpMark;

    /* seek to the end of the list */
    while (current->next)
    {
        current = current->next;
    }

    /** add the command to the end of the list */
    command = new Command(name, this);
    current->next = command;
}

/*
 * Console::CommandFlow::CommandFlow()
 */
Console::CommandFlow::~CommandFlow()
{
    Console *console = static_cast<Console *>(service());
    Command *current = &console->helpMark;

    /* seek to just before our command instance */
    while (current)
    {
        if (current->next == command)
        {
            /* remove and our command from the list and delete instance */
            current->next = command->next;
            delete command;
            break;
        }
        current = current->next;
    }
}

#if defined (CONSOLE_NETWORKING)
/*
 * Console::Listen::Listen()
 */
Console::Listen::Listen(Service *service, int port)
    : StateFlowBase(service)
    , fdListen(-1)
    , selectHelper(this)
{
    if (port < 0)
    {
        /* invalid port number, this class will do nothing */
        return;
    }

    int                yes = 1;
    struct sockaddr_in sockaddr;
    int                result;

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = INADDR_ANY;
    sockaddr.sin_port = htons(port);

    /* open TCP socket */
    fdListen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    HASSERT(fdListen >= 0);

    /* reuse socket address if already in lingering use */
    result = setsockopt(fdListen, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    HASSERT(result == 0);

    /* turn off the nagel alogrithm */
    result = setsockopt(fdListen, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(int));
    HASSERT(result == 0);

    /* bind the address parameters to the socket */
    result = bind(fdListen, (struct sockaddr*)&sockaddr, sizeof(sockaddr));
    HASSERT(result == 0);

    /* set socket non-blocking */
    result = fcntl(fdListen, F_SETFL, fcntl(0, F_GETFL, 0) | O_NONBLOCK);
    HASSERT(result == 0);

    /* mark socket as listen */
    result = ::listen(fdListen, 10);
    HASSERT(result == 0);

    /* start state flow */
    start_flow(STATE(entry));
}

/*
 * Console::Listen::accept()
 */
StateFlowBase::Action Console::Listen::accept()
{
    int newfd = ::accept(fdListen, NULL, NULL);
    if (newfd >= 0)
    {
        int yes = 1;
        int result = setsockopt(newfd, IPPROTO_TCP,
                                TCP_NODELAY, &yes, sizeof(int));
        HASSERT(result == 0);
        static_cast<Console *>(service())->open_session(newfd, newfd);
    }

    return listen_and_call(&selectHelper, fdListen, STATE(accept));
}
#endif

/*
 * Console::Listen::process_read()
 */
StateFlowBase::Action Console::Session::process_read()
{
    size_t count = (line_size - pos) - selectHelper.remaining_;

    if (count == 0)
    {
        struct stat stat;
        fstat(fdIn, &stat);
#ifdef HAVE_BSDSOCKET
        if (S_ISSOCK(stat.st_mode))
        {
            /* Socket connection closed */
            return delete_this();
        }
#endif
    }

    while(count--)
    {
        if (line[pos++] == '\n')
        {
            /* parse the line input into individual arguments */
            unsigned argc = 0;
            char last = '\0';

            for (size_t i = 0; i < pos; ++i)
            {
                switch (line[i])
                {
                    case '\r':
                    case '\n':
                    case ' ':
                        line[i] = '\0';
                        break;
                    case '"':
                        /** @todo quoted arguments not yet supported */
                    default:
                        if (last == '\0')
                        {
                            args[argc] = &line[i];
                            argc = (argc == MAX_ARGS) ? MAX_ARGS : argc + 1;
                        }
                        break;
                }
                last = line[i];
            }

            if (command != nullptr)
            {
                command->flow->notify();
                command->flow->argc = argc;
                command->flow->argv = args;
                //printf("%s", args[0]);
                return wait_and_call(STATE(exit_interactive));
            }

            switch (argc)
            {
                case 0:
                    break;
                case MAX_ARGS:
                    fprintf(fp, "too many arguments\n");
                    break;
                default:
                {
                    CommandStatus status = callback(argc, args);
                    if (status == COMMAND_NEXT)
                    {
                        return wait_and_call(STATE(exit_interactive));
                    }
                    else
                    {
                        if (callback_result_process(status, args[0]) == false)
                        {
                            return delete_this();
                        }
                    }
                }
            }
            pos = 0;
            prompt(fp);
        }
        else
        {
            if (pos >= line_size)
            {
                /* double the line buffer size */
                line_size *= 2;
                char *new_line = (char*)malloc(line_size);
                memcpy(new_line, line, pos);
                free(line);
                line = new_line;
            }
        }
    }
    return call_immediately(STATE(entry));
}

/*
 * Console::Session::Callback()
 */
Console::CommandStatus Console::Session::callback(int argc, const char *argv[])
{
    Console *console = static_cast<Console *>(service());

    /* run through each command */
    for (Command *current = &console->help; current; current = current->next)
    {
        /* look for a command match */
        if (strcmp(current->name, argv[0]) == 0)
        {
            /* found a match, call the registered callback */
            if (current->interactive)
            {
                command = current;
                return current->flow->callback(this, fdIn, fp, argc, argv);
            }
            else
            {
                return (*current->callback)(fp, argc, argv, current->context);
            }
        }
    }

    return COMMAND_NOT_FOUND;
}

/*
 * Console::Session::exit_interactive()
 */
StateFlowBase::Action Console::Session::exit_interactive()
{
    if (command->flow->status == COMMAND_NEXT)
    {
        pos = 0;
        return call_immediately(STATE(entry));
    }
    HASSERT(callback_result_process(command->flow->status, command->name) == true);

    pos = 0;
    prompt(fp);
    
    command = nullptr;

    return call_immediately(STATE(entry));
}

/*
 * Console::Session::callback_result_process()
 */
bool Console::Session::callback_result_process(CommandStatus status,
                                               const char *name)
{
    switch (status)
    {
        default:
            break;
        case COMMAND_ERROR:
            fprintf(fp, "invalid arguments\n");
            break;
        case COMMAND_CLOSE:
        {
            struct stat stat;
            fstat(fdIn, &stat);
#ifdef HAVE_BSDSOCKET
            if (S_ISSOCK(stat.st_mode))
            {
                fprintf(fp, "shutting down session\n");
                return false;
            }
#endif
            fprintf(fp, "session not a socket, "
                        "aborting session shutdown\n");
            break;
        }
        case COMMAND_NOT_FOUND:
            fprintf(fp, "%s: command not found\n", name);
    }

    return true;
}
