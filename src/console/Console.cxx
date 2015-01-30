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

#if defined (__FreeRTOS__)
#else
#define CONSOLE_NETWORKING
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include "console/Console.hxx"

/** Constructor.
 * @param stdio start a Console connection instance on stdio if true
 * @param port TCP port number to open a telnet listen socket on, -1 means
 *        do not open a listen port.
 */
Console::Console(bool stdio, int port)
    : OSThread()
    , help("help", help_command, this, &helpMark)
    , helpMark("?", help_command, this)
    , first(NULL)
    , fdListen(-1)
    , fdHighest(0)
{
    FD_ZERO(&readfds);
    if (port >= 0)
    {
        listen_create(port);
    }

    if (stdio)
    {
        open_session(0);
    }

    add_command("quit", quit_command, this);
    start("console", 0, 1024);
}

/** Open and initialize a new session.
 * @param fd file descriptor belonging to session
 */
void Console::open_session(int fd)
{
    fcntl(0, F_SETFL, fcntl(0, F_GETFL, 0) | O_NONBLOCK);
    Session *new_session = new Session(fd);

    if (first == NULL)
    {
        first = new_session;
    }
    else if (fd < first->fd)
    {
        new_session->next = first;
        first = new_session;
    }
    else
    {
        Session *current = first->next;
        Session *last = first;

        while (current)
        {
            if (current->fd > fd && last->fd < fd)
            {
                break;
            }
            last = current;
            current = current->next;
        }
        last->next = new_session;
        new_session->next = current;
    }
    if (fd > fdHighest)
    {
        fdHighest = fd;
    }
    FD_SET(fd, &readfds);
    prompt(new_session->fp);

}

/** Close a previously established session
 * @param s Session to close
 */
void Console::close_session(Session *s)
{
    /** @todo need to pull this out of the Session list */
    FD_CLR(s->fd, &readfds);

    if (fdHighest == s->fd)
    {
        for (int i = s->fd; i >= 0; --i)
        {
            if (FD_ISSET(i, &readfds))
            {
                fdHighest = s->fd;
                break;
            }
        }
    }
    fclose(s->fp);
    close(s->fd);
    free(s->line);
    delete s;
}

/** Get the session belonging to a file descriptor.
 * @param fd file descriptor
 * @return session belonging to the file descriptor
 */
Console::Session *Console::get_session(int fd)
{
    for (Session *current = first; current; current = current->next)
    {
        if (current->fd == fd)
        {
            return current;
        }
    }
    return NULL;
}

/** Create a listen socket.
 * @param port port number to listen on
 */
void Console::listen_create(int port)
{
#if defined (CONSOLE_NETWORKING)
    int                yes = 1;
    struct sockaddr_in sockaddr;
    int                result;

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = INADDR_ANY;

    sockaddr.sin_port = htons(port);

    fdListen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fdListen < 0)
    {
        printf("Unable to create listen socket, punt: %s\n", strerror(errno));
        abort();
    }

    result = setsockopt(fdListen, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    if (result != 0)
    {
        printf("Unable to setsockopt SO_REUSEADDR, punt: %s\n", strerror(errno));
        abort();
    }

    /* turn off the nagel alogrithm */
    result = setsockopt(fdListen, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(int));
    if (result != 0)
    {
        printf("Unable to setsockopt TCP_NODELAY, punt: %s\n", strerror(errno));
        abort();
    }

    result = bind(fdListen, (struct sockaddr*)&sockaddr, sizeof(sockaddr));
    if (result != 0)
    {
        if (errno == EADDRINUSE)
        {
            printf("Address in use\n");
        }
        printf("Unable to bind socket, punt: %s\n", strerror(errno));
        abort();
    }

    result = listen(fdListen, 10);
    if (result != 0)
    {
        printf("Unable to listen on socket, punt: %s\n", strerror(errno));
        abort();
    }
    FD_SET(fdListen, &readfds);
    fdHighest = fdListen;
#endif
}

/** Add a new command to the console.
 * @param name command name
 * @param callback callback function for command
 * @param context context pointer to pass into callback
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

/** Print out the help menu.
 * @param fp file pointer to console
 * @param argc number of arguments including the command itself
 * @param argv array of arguments starting with the command itself
 * @return COMMAND_OK
 */
int Console::help_command(FILE *fp, int argc, const char *argv[])
{
    fprintf(fp, "%10s : print out this help memu\n", "help | ?");    

    /* call each of the commands with argc = 0 */
    for (Command *current = helpMark.next; current; current = current->next)
    {
        fprintf(fp, "%10s : ", current->name);
        const char *argv[2] = {current->name, "             "};
        (*current->callback)(fp, 0, argv, NULL);
    }

    return COMMAND_OK;
}

/** Quit out of the current login session.
 * @param fp file pointer to console
 * @param argc number of arguments including the command itself
 * @param argv array of arguments starting with the command itself
 * @return COMMAND_OK
 */
int Console::quit_command(FILE *fp, int argc, const char *argv[])
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

/** Process a potential callback for a given command
 * @param fp file pointer to console
 * @param argc number of arguments including the command itself
 * @param argv array of arguments starting with the command itself
 * @return if false, close the session
 */
bool Console::callback(FILE *fp, int argc, const char *argv[])
{
    /* run through each command */
    for (Command *current = &help; current; current = current->next)
    {
        /* look for a command match */
        if (strcmp(current->name, argv[0]) == 0)
        {
            /* found a match, call the registered callback */
            int result = (*current->callback)(fp, argc, argv, current->context);
            switch (result)
            {
                default:
                    break;
                case COMMAND_ERROR:
                    fprintf(fp, "invalid arguments\n");
                    break;
                case COMMAND_CLOSE:
                    return false;
            }
            return true;
        }
    }
    fprintf(fp, "%s: command not found\n", argv[0]);
    return true;
}

/** Decode in incoming character.
 * @param c character to decode
 * @param session console session the character belongs to
 */
void Console::decode(char c, Session *s)
{
    s->line[s->pos++] = c;

    if (c == '\n')
    {
        /* parse the line input into individual arguments */
        unsigned argc = 0;
        const char *args[MAX_ARGS];
        char last = '\0';

        for (size_t i = 0; i < s->pos; ++i)
        {
            switch (s->line[i])
            {
                case '\r':
                case '\n':
                case ' ':
                    s->line[i] = '\0';
                    break;
                case '"':
                default:
                    if (last == '\0')
                    {
                        args[argc] = &s->line[i];
                        argc = (argc == MAX_ARGS) ? MAX_ARGS : argc + 1;
                    }
                    break;
            }
            last = s->line[i];
        }

        switch (argc)
        {
            case 0:
                break;
            case MAX_ARGS:
                fprintf(s->fp, "too many arguments\n");
                break;
            default:
                if (callback(s->fp, argc, args) == false)
                {
                    struct stat stat;
                    fstat(s->fd, &stat);
                    if (stat.st_mode & S_IFSOCK)
                    {
                        fprintf(s->fp, "shutting down session\n");
                        close_session(s);
                        return;
                    }
                    fprintf(s->fp, "session not a socket,"
                                   " aborting session shutdown\n");
                }
                break;
        }
        s->pos = 0;
        prompt(s->fp);
    }
    else
    {
        if (s->pos >= s->line_size)
        {
            /* double the line buffer size */
            s->line_size *= 2;
            char *new_line = (char*)malloc(s->line_size);
            memcpy(s->line, new_line, s->pos);
            free(s->line);
            s->line = new_line;
        }
    }
}

/** Entry point to the thread.
 * @return should never return
 */
void *Console::entry()
{
    for ( ; /* forever */ ; )
    {
        fd_set rfds = readfds;
        int result = select(fdHighest + 1, &rfds, NULL, NULL, NULL);
        if (result <= 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            printf("select failed: %s\n", strerror(errno));
            abort();
        }
        for (int i = 0; i <= fdHighest && result > 0; i++)
        {
            if (FD_ISSET(i, &rfds))
            {
                --result;
#if defined (CONSOLE_NETWORKING)
                if (i == fdListen)
                {
                    /* establish a new connection request for a new session */
                    int newfd = accept(fdListen, NULL, NULL);
                    if (newfd < 0)
                    {
                        printf("accept failed: %s\n", strerror(errno));
                        abort();
                    }

                    int yes   = 1;
                    result = setsockopt(newfd, IPPROTO_TCP,
                                        TCP_NODELAY, &yes, sizeof(int));
                    if (result != 0)
                    {
                        printf("Unable to setsockopt TCP_NODELAY: %s\n",
                               strerror(errno));
                        abort();
                    }
                    open_session(newfd);
                }
                else
#endif
                {
                    /* handle an existing connection */
                    char c;
                    ssize_t result = read(i, &c, 1);
                    if (result == 0)
                    {
                        close_session(get_session(i));
                    }
                    else
                    {
                        decode(c, get_session(i));
                    }
                }
            }
        }
    }

    return NULL;
}

