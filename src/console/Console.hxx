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
 * \file Console.hxx
 * This file provides an implementation of an interactive text console.
 *
 * @author Stuart W. Baker
 * @date 10 May 2014
 */

#ifndef _CONSOLE_CONSOLE_HXX_
#define _CONSOLE_CONSOLE_HXX_

#include <sys/select.h>
#include <cstdio>

#include "os/OS.hxx"

/** This class provides a console available from stdin/stdout as well as via
 * telnet.  Adding a command to the console is as simple as using the @ref
 * add_command method.  A command callback is responsible for providing its own
 * help text.  When the command callback is called with argc = 0, this is a
 * clue to the callback to print its help information.  In this case, argv[0]
 * will still have the command name itself, but argv[1] will have an additional
 * parameter that can be used to indent and align multi-line help information.
 */
class Console : public OSThread
{
public:
    /** Constructor.
     * @param stdio start a Console connection instance on stdio if true
     * @param port TCP port number to open a telnet listen socket on, -1 mean
     *        do not open a listen port.
     */
    Console(bool stdio, int port = -1);

    /** Destructor.
     */
    ~Console()
    {
    }

    /** Enumeration of recognized command callback results.
     */
    enum
    {
        COMMAND_OK,    /**< Command executed successfully */
        COMMAND_ERROR, /**< Command had some kind of error */
        COMMAND_CLOSE, /**< Command wants to close the session. */
    };

    /** Console command callback.
     */
    typedef int (*Callback)(FILE *, int, const char *argv[], void *);

    /** Add a new command to the console.
     * @param name command name
     * @param callback callback function for command
     * @param context context pointer to pass into callback
     */
    void add_command(const char *name, Callback callback, void *context = NULL);

private:
    /** Maximum number of supported arguments including the command itself */
    static const size_t MAX_ARGS = 10;

    /** Console command metadata.
     */
    struct Command
    {
        /** Construct a new command.
         * @param name command name
         * @param callback callback function for command
         * @param context context pointer to pass into callback
         * @param next next command in list
         */
        Command(const char *name, Callback callback, void *context = NULL, Command *next = NULL)
            : name(name)
            , callback(callback)
            , context(context)
            , next(next)
        {
        }

        const char *name;  /**< command name */
        Callback callback; /**< callback function for command */
        void *context;     /**< context pointer to pass into callback */
        Command *next;     /**< next Command in list */
    };

    /** Console session metadata.
     */
    struct Session
    {
        /** Constructor.
         * @param fd file descriptor for the session
         */
        Session(int fd)
            : fd(fd)
            , fp(fdopen(fd, "r+"))
            , line((char*)malloc(64))
            , line_size(64)
            , pos(0)
            , next(NULL)
        {
        }
        int fd;           /**< file descriptor of session */
        FILE *fp;         /**< file pointer of session */
        char *line;       /**< current line content */
        size_t line_size; /**< current max line size */
        size_t pos;       /**< current line position */
        Session *next;    /**< next Session in the list */
    };

    Command help;     /**< the "help" command instance */
    Command helpMark; /**< the help "?" command instance */
    Session *first;   /**< the first session in the session list */
    int fdListen;     /**< file descriptor of the listen socket */
    int fdHighest;    /**< highest file descriptor in the set */
    fd_set readfds;   /**< read file descriptor set */

    /** Print out the help menu by calling the in context helper function.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @param context pointer to a Console context
     * @return COMMAND_OK
     */
    static int help_command(FILE *fp, int argc, const char *argv[], void *context)
    {
        return static_cast<Console*>(context)->help_command(fp, argc, argv);
    }

    /** Print out the help menu.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return COMMAND_OK
     */
    int help_command(FILE *fp, int argc, const char *argv[]);

    /** Quit out of the current login session by calling the in context helper
     * function.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @param context pointer to a Console context
     * @return COMMAND_OK
     */
    static int quit_command(FILE *fp, int argc, const char *argv[], void *context)
    {
        return static_cast<Console*>(context)->quit_command(fp, argc, argv);
    }

    /** Quit out of the current login session.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return COMMAND_OK
     */
    int quit_command(FILE *fp, int argc, const char *argv[]);

    /** Create a listen socket.
     * @param port port number to listen on
     */
    void listen_create(int port);

    /** Open and initialize a new session.
     * @param fd file descriptor belonging to session
     */
    void open_session(int fd);

    /** Close a previously established session
     * @param s Session to close
     */
    void close_session(Session *s);

    /** Get the session belonging to a file descriptor.
     * @param fd file descriptor
     * @return session belonging to the file descriptor
     */
    Session *get_session(int fd);

    /** Print the standard prompt.
     * @param fp FILE pointer to send prompt to
     */
    void prompt(FILE *fp)
    {
        fputs("> ", fp);
        fflush(fp);
    }

    /** Process a potential callback for a given command
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return if false, close the session
     */
    bool callback(FILE *fp, int argc, const char *argv[]);

    /** Decode in incoming character.
     * @param c character to decode
     * @param session console session the character belongs to
     */
    void decode(char c, Session *s);

    /** Entry point to the thread.
     * @return should never return
     */
    void *entry() override;
};

#endif // _CONSOLE_CONSOLE_HXX_
