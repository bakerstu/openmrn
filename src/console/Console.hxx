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

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"

#if OPENMRN_FEATURE_BSD_SOCKETS
#define CONSOLE_NETWORKING
#endif

/** This class provides a console available from stdin/stdout as well as via
 * telnet.  Adding a command to the console is as simple as using the @ref
 * add_command() method or instantiating a new derived instance if @ref
 * CommandFlow.  A command callback is responsible for providing its own
 * help text.  When the command callback is called with argc = 0, this is a
 * clue to the callback to print its help information.  In this case, argv[0]
 * will still have the command name itself, but argv[1] will have an additional
 * parameter that can be used to indent and align multi-line help information.
 *
 * New CommandFlow instances and add_command() calls must occur either in
 * global construction (before main() is called), or within the same executor
 * context as the Console instance they are being attached to in order to avoid
 * a problem with mutual exclusion.
 *
 * add_command() is used to generate a standard out of context callback when the
 * specified command is entered on the console.
 *
 * A CommandFlow is specialized in order to generate a state flow based command.
 * This may be useful for interactive commands, but may also be used with
 * non-interactive commands.
 */
class Console : public Service
{
private:
    /** forward declaration */
    struct Command;

    /** forward declaration */
    class Session;

public:
    /** Enumeration of recognized command callback results.
     */
    enum CommandStatus
    {
        COMMAND_OK,        /**< Command executed successfully */
        COMMAND_NEXT,      /**< Command waiting for input */
        COMMAND_ERROR,     /**< Command had some kind of error */
        COMMAND_CLOSE,     /**< Command wants to close the session. */
        COMMAND_NOT_FOUND, /**< Command not found */
    };

    /** Console command callback.
     */
    typedef CommandStatus (*Callback)(FILE *, int, const char *argv[], void *);

    /** Constructor.
     * @param executor the executor thread that the Console flows will execute
     *                 on
     * @param port TCP port number to open a telnet listen socket on, -1 mean
     *        do not open a listen port.
     */
    Console(ExecutorBase *executor, uint16_t port);

    /** Constructor.
     * @param executor the executor thread that the Console flows will execute
     *                 on
     * @param fd_in input file descriptor for the session
     * @param fd_out output file descriptor for the session
     * @param port TCP port number to open a telnet listen socket on, -1 mean
     *        do not open a listen port.
     */
    Console(ExecutorBase *executor, int fd_in, int fd_out, int port = -1);

    /** Destructor.
     */
    ~Console()
    {
    }

    /** Add a console session.
     * @param fd_in input file descriptor for the session
     * @param fd_out output file descriptor for the session
     */
    void add_session(int fd_in, int fd_out)
    {
        open_session(fd_in, fd_out);
    }

    /** Add a new command to the console.
     * @param name command name
     * @param callback callback function for command
     * @param context context pointer to pass into callback
     */
    void add_command(const char *name, Callback callback, void *context = NULL);

    /** Default STDIN file descriptor */
    static const int FD_STDIN = 0;

    /** Default STDOUT file descriptor */
    static const int FD_STDOUT = 1;

    /** State flow base class that handles interactive commands.
     * This may also be used for non-interactive commands that want to use
     * the StateFlow paradigm for responding.
     *
     * Example:
     * @code
     *  class ExampleCommandFlow : public Console::CommandFlow
     *  {
     *  public:
     *      ExampleCommandFlow()
     *          : CommandFlow(&console, "test")
     *      {
     *      }
     *
     *      ~ExampleCommandFlow()
     *      {
     *      }
     *
     *  private:
     *      StateFlowBase::Action entry()
     *      {
     *          fprintf(fp, "param: ");
     *          return wait_for_line_and_call(STATE(input));
     *      }
     *
     *      StateFlowBase::Action input()
     *      {
     *          if (argc == 1 && strcmp(argv[0])
     *          {
     *              // do stuff
     *              return record_status_and_exit(Console::COMMAND_OK);
     *          }
     *          return record_status_and_exit(Console::COMMAND_ERROR);
     *      }
     *  };
     * @endcode
     */
    class CommandFlow : public StateFlowBase
    {
    protected:
        /** Constructor.
         * @param console Console instance that this belongs to
         * @param name command name
         */
        CommandFlow(Console *console, const char *name);

        /** Destructor.
         */
        ~CommandFlow();

        /** Entry point to command flow.
         * @return defined by derived class
         */
        virtual StateFlowBase::Action entry() = 0;

        /** Read a line of input
         * @return next state wait_for_line()
         */
        StateFlowBase::Action read_line();

        /** number of arguments on command line */
        int argc;

        /** list of arguments on command line */
        const char **argv;

        /** FILE* reference for output data */
        FILE *fp;

        /** Wait for a complete line of input
         * @param c state to call when the line is acquired.
         * @return next action
         */
        StateFlowBase::Action wait_for_line_and_call(StateFlowBase::Callback c)
        {
            fflush(fp);
            this->status = COMMAND_NEXT;
            session->notify();
            return wait_and_call(c);
        }

        /** Record the CommandStatus and exit the CommandFlow.
         * @param status exit status of the command
         * @return exit action
         */
        StateFlowBase::Action record_status_and_exit(CommandStatus status)
        {
            this->status = status;
            session->notify();
            return STATE(exit);
        }

    private:
        /** Start flow with the incoming data parameters */
        CommandStatus callback(Session *session, int fd, FILE *fp, int argc, const char *argv[])
        {
            this->session = session;
            this->fd = fd;
            this->fp = fp;
            this->argc = argc;
            this->argv = argv;
            start_flow(STATE(entry));

            return COMMAND_NEXT;
        }

        /** Keep track of Command instance for destruction time */
        Command *command;

        /** file descriptor for input data */
        int fd;

        /** Session flow to notify when command is complete */
        Session *session;

        /** Resulting status of running command */
        CommandStatus status;

        /** Give Console class access to CommandFlow private members */
        friend class Console;

        /** Give Session class access to Commandflow private members */
        friend class Session;

        DISALLOW_COPY_AND_ASSIGN(CommandFlow);
    };

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
            , interactive(false)
            , next(next)
        {
        }

        /** Construct a new command.
         * @param name command name
         * @param flow ??
         * @param next next command in list
         */
        Command(const char *name, CommandFlow *flow, Command *next = NULL)
            : name(name)
            , flow(flow)
            , interactive(true)
            , next(next)
        {
        }

        const char *name;  /**< command name */
        union 
        {
            struct
            {
                Callback callback; /**< callback function for command */
                void *context;     /**< context pointer to pass into callback */
            };
            CommandFlow *flow; /**< state flow for interactive commands */
        };
        bool interactive; /**< true if command is interactive */
        Command *next;    /**< next Command in list */
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
         * @param fd_in input file descriptor for the session
         * @param fd_out output file descriptor for the session
         */
        Session(Service *service, int fd_in, int fd_out);

        /** Desctructor.
         */
        ~Session()
        {
            /* should only delete sockets, which should have the same read
             * and write fds
             */
            HASSERT(fdIn == fdOut);
            fclose(fp);
            /* There is no need for a "close(fdIn)" because the "fclose(fp)"
             * will already have completed that operation.
             */
            free(line);
        }

    private:
        /** Entry point to the state machine.  Read some command line input.
         * @return next state is process_read()
         */
        StateFlowBase::Action entry()
        {
            return read_single(&selectHelper, fdIn, line + pos,
                               line_size - pos, STATE(process_read));
        }

        /** Process the incoming command line input.
         * @return next state is entry() to gather more data, exit_interactive()
         *         if an interactive command is called.
         */
        StateFlowBase::Action process_read();

        /** Wait for completion of an interactive command in order to cleanup
         * based on result.
         * @return next state is entry()
         */
        StateFlowBase::Action exit_interactive();

        /** Process a potential callback for a given command
         * @param argc number of arguments including the command itself
         * @param argv array of arguments starting with the command itself
         * @return status of the callback being called
         */
        CommandStatus callback(int argc, const char *argv[]);

        /** Print the standard prompt.
         * @param fp FILE pointer to send prompt to
         */
        void prompt(FILE *fp)
        {
            fputs("> ", fp);
            fflush(fp);
        }

        /** Process the result of the command callback
         * @param status the result of running a command
         * @param name name of the command itself
         * @return false to close the session, else true
         */
        bool callback_result_process(CommandStatus status, const char *name);

        int fdIn;         /**< input file descriptor of the session */
        int fdOut;        /**< output file descriptor of the session */
        FILE *fp;         /**< file pointer of session */
        char *line;       /**< current line content */
        size_t line_size; /**< current max line size */
        size_t pos;       /**< current line position */
        const char *args[MAX_ARGS]; /**< parsed argument list */

        /** metadata for waiting on the listen socket to become active */
        StateFlowBase::StateFlowSelectHelper selectHelper;

        /** Command instance that we are currently acting on */
        Command *command;

        DISALLOW_COPY_AND_ASSIGN(Session);
    };

    /** Print out the help menu by calling the in context helper function.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @param context pointer to a Console context
     * @return COMMAND_OK
     */
    static CommandStatus help_command(FILE *fp, int argc, const char *argv[], void *context)
    {
        return static_cast<Console*>(context)->help_command(fp, argc, argv);
    }

    /** Print out the help menu.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return COMMAND_OK
     */
    CommandStatus help_command(FILE *fp, int argc, const char *argv[]);

    /** Quit out of the current login session by calling the in context helper
     * function.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @param context pointer to a Console context
     * @return COMMAND_OK
     */
    static CommandStatus quit_command(FILE *fp, int argc, const char *argv[], void *context)
    {
        return static_cast<Console*>(context)->quit_command(fp, argc, argv);
    }

    /** Quit out of the current login session.
     * @param fp file pointer to console
     * @param argc number of arguments including the command itself
     * @param argv array of arguments starting with the command itself
     * @return COMMAND_OK
     */
    CommandStatus quit_command(FILE *fp, int argc, const char *argv[]);

    /** Open and initialize a new session.
     * @param fd_in input file descriptor belonging to session
     * @param fd_out output file descriptor belonging to session
     */
    void open_session(int fd_in, int fd_out);

    Command help;     /**< the "help" command instance */
    Command helpMark; /**< the help "?" command instance */
#if defined (CONSOLE_NETWORKING)
    Listen listen; /**< object that will listen for incoming connections */
#endif
    /** Give CommandFlow class access to Console private members */
    friend class CommandFlow;

    /** Give Listen class access to Console private members */
    friend class Listen;

    /** Give Session class access to Console private members */
    friend class Session;

    DISALLOW_COPY_AND_ASSIGN(Console);
};

#endif // _CONSOLE_CONSOLE_HXX_
