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

#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"

#if defined (__FreeRTOS__) && !defined (GCC_ARMCM3)
#else
#define CONSOLE_NETWORKING
#endif

/** This class provides a console available from stdin/stdout as well as via
 * telnet.  Adding a command to the console is as simple as using the @ref
 * add_command method.  A command callback is responsible for providing its own
 * help text.  When the command callback is called with argc = 0, this is a
 * clue to the callback to print its help information.  In this case, argv[0]
 * will still have the command name itself, but argv[1] will have an additional
 * parameter that can be used to indent and align multi-line help information.
 */
class Console : public Service
{
public:
    /** Constructor.
     * @param executor the executor thread that the Console flows will execute
     *                 on
     * @param stdio start a Console connection instance on stdio if true
     * @param port TCP port number to open a telnet listen socket on, -1 mean
     *        do not open a listen port.
     */
    Console(ExecutorBase *exectutor, bool stdio, int port = -1);

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

#if defined (CONSOLE_NETWORKING)
    /** State flow that will accept incoming connections.
     */
    class Listen : public StateFlowBase
    {
    public:
        /** Constructor.
         * @param service service instance that this listen socket belongs to
         * @param port port number to listen on
         */
        Listen(Service *service, int port);

    private:
        /** Entry point to the state machine.
         * @return next state is accept() pending an active listen socket
         */
        StateFlowBase::Action entry()
        {
            return listen_and_call(&selectHelper, fdListen, STATE(accept));
        }

        /** Accept the incoming connection.
         * @return next state is accept() to accept the next connection
         */
        StateFlowBase::Action accept();

        /** listen socket descriptor */
        int fdListen;

        /** metadata for waiting on the listen socket to become active */
        StateFlowBase::StateFlowSelectHelper selectHelper;

        DISALLOW_COPY_AND_ASSIGN(Listen);
    };
#endif

    /** Console session metadata.
     */
    class Session : public StateFlowBase
    {
    public:
        /** Constructor.
         * @param service service instance that this session belongs to
         * @param fd file descriptor for the session
         */
        Session(Service *service, int fd)
            : StateFlowBase(service)
            , fd(fd)
            , fp(fdopen(fd, "r+"))
            , line((char*)malloc(64))
            , line_size(64)
            , pos(0)
            , selectHelper(this)
        {
            prompt(fp);
            start_flow(STATE(entry));
        }

        /** Desctructor.
         */
        ~Session()
        {
            fclose(fp);
            close(fd);
            free(line);
        }

    private:
        /** Entry point to the state machine.
         * @return next state is accept() pending an active listen socket
         */
        StateFlowBase::Action entry()
        {
            return read_single(&selectHelper, fd, line + pos,
                               line_size - pos, STATE(process_read));
        }

        /** Entry point to the state machine.
         * @return next state is accept() pending an active listen socket
         */
        StateFlowBase::Action process_read();

        /** Print the standard prompt.
         * @param fp FILE pointer to send prompt to
         */
        void prompt(FILE *fp)
        {
            fputs("> ", fp);
            fflush(fp);
        }

        int fd;           /**< file descriptor of session */
        FILE *fp;         /**< file pointer of session */
        char *line;       /**< current line content */
        size_t line_size; /**< current max line size */
        size_t pos;       /**< current line position */

        /** metadata for waiting on the listen socket to become active */
        StateFlowBase::StateFlowSelectHelper selectHelper;

        DISALLOW_COPY_AND_ASSIGN(Session);
    };

    Command help;     /**< the "help" command instance */
    Command helpMark; /**< the help "?" command instance */
#if defined (CONSOLE_NETWORKING)
    Listen listen; /**< object that will listen for incoming connections */
#endif
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

    /** Open and initialize a new session.
     * @param fd file descriptor belonging to session
     */
    void open_session(int fd);

    /** Process a potential callback for a given command
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return if false, close the session
     */
    bool callback(FILE *fp, int argc, const char *argv[]);

    /** Give Listen class access to Console private members */
    friend class Listen;

    /** Give Session class access to Console private members */
    friend class Session;
};

#endif // _CONSOLE_CONSOLE_HXX_
