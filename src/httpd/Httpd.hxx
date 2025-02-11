// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sat Feb 8 19:08:26 2025
//  Last Modified : <250210.1950>
//
//  Description	
//
//  Notes
//
//  History
//	
/////////////////////////////////////////////////////////////////////////////
/// @copyright
/// Copyright (C) 2025  Robert Heller D/B/A Deepwoods Software
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are  permitted provided that the following conditions are met:
///
///  - Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///
///  - Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @file Httpd.hxx
/// @author Robert Heller
/// @date Sat Feb 8 19:08:26 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#ifndef __HTTPD_HXX
#define __HTTPD_HXX


#if OPENMRN_FEATURE_BSD_SOCKETS

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "httpd/HTTP_Method.hxx"
#include "httpd/Uri.hxx"
#include "httpd/HttpRequest.hxx"
#include "httpd/HttpReply.hxx"

namespace HTTPD {

/** Forward declaration */
class RequestHandler;

/** This class provides a httpd server instance. The @ref add_uri() method
 * is used to add elements to the dispatching table.
 */
class Httpd : public Service
{
public:
    /** Enumeration of recognized HTTP status codes. */
    enum HttpStatusCode
    {
        Data_follows = 200,
        No_Content = 204,
        Moved_PermanentlyA = 301,
        Found = 302,
        Moved_TemporarilyA = 303,
        Not_Modified = 304,
        Moved_PermanentlyB = 307,
        Moved_TemporarilyB = 308,
        Bad_Request = 400,
        Authorization_Required = 401,
        Permission_denied = 403,
        Not_Found = 404,
        Request_Timeout = 408,
        Length_Required = 411,
        Expectation_Failed = 419,
        Server_Internal_Error = 500,
        Server_Busy = 501,
        Service_Unavailable = 503,
        Service_Temporarily_Unavailable = 504,
        HTTP_Version_Not_Supported = 505
    };
    /** Http Request Handler callback. 
     * @param request HttpRequest class instance containing information about 
     *                the request
     * @param reply HttpReply class instance to be used to build the reply
     * @param userContext a user supplied pointer for use by the user's 
     *                    code.
     */
    typedef  std::function<void(const HttpRequest *request, HttpReply *reply, 
                                void *userContext)> HandlerFunction;
    
    /** Constructor.
     * @param executor the executor thread that the Httpd Server flows will 
     * execute on
     * @param port TCP port number to open a httpd server listen socket on
     */            (defaults to 80)
    Httpd(ExecutorBase *executor, uint16_t port = 80);
    
    /** Destructor. */
    ~Httpd()
    {
    }
    
    /** Add a new URI handler to the server.
     * @param uri the URI to dispatch on.
     * @param callback callback function for the uri.
     * @param context context pointer to pass into callback.
     */
    void add_uri(const Uri &uri, HandlerFunction callback, void *context);
    
    /** Add a handler for 404 (Not Found).
     * @param callback callback function for the the 404 page.
     * @param context context pointer to pass into callback.
     */
    void add_notfound(HandlerFunction callback, void *context);
    
    /** Add an upload handler.
     * @param callback callback function for uploading.
     * @param context context pointer to pass into callback.
     */
    void add_upload(HandlerFunction callback, void *context);
    
    
    /** Uri Handler Class */
    class UriHandler {
    public:
        /** Construct a Uri Handler.
         * @param callback callback function for uri
         * @param context context pointer to pass into callback
         */
        UriHandler(HandlerFunction callback, void *context = NULL)
                    : callback_(callback)
              , context_(context)
    
        {
        }
        /** Copy construct a Uri Handler.
         * @param other the other Uri Handler.
         */
        UriHandler(const UriHandler& other)
        {
            callback_ = other.callback_;
            context_  = other.context_;
        }
        /** Assignment operator
         * @param other RHS of the equal sign (source object)
         * @returns this LHS of the equal sign (destination object)
         */
        UriHandler& operator = (const UriHandler& other)
        {
            callback_ = other.callback_;
            context_  = other.context_;
            return *this;
        }
        /** Invoke the handler.  Calls the handler with its context.
         * @param request The request instance
         * @param reply The reply instance
         */
        void handle(const HttpRequest *request, HttpReply *reply) const
        {
            callback_(request,reply,context_);
        }
        /** Not operator: is this a valid handler.
         * @returns true if the callback function is a nullptr.
         */
        bool operator !() const
        {
            return callback_ == nullptr;
        }
    private:
        HandlerFunction callback_ = nullptr; /**< callback function for Uri */
        void *context_ = nullptr; /**< context pointer to pass into callback */
    };
    
    /** Return the head of the handler list. */
    RequestHandler * FirstHandler() {return firstHandler_;}
    /** Return the not found handler. */
    UriHandler * NotFoundHandler() {return &notFoundHandler_;}
private:
    
    RequestHandler *firstHandler_; /**< Head of the handler list. */
    RequestHandler *lastHandler_; /**< Tail of the handler list. */
    UriHandler      notFoundHandler_; /**< Not Found (404) handler */
    UriHandler      fileUploadHandler_; /**< File Upload handler */
    
    /** Add a request handler to the list
     * @param handler The handler to add.
     */
    void addRequestHandler_(RequestHandler* handler);
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
    
    /** Open and initialize a new HTTPRequest.
     * @param fd_in input file descriptor belonging to HTTPRequest
     * @param fd_out output file descriptor belonging to HTTPRequest
     */
    void open_request(int fd_in, int fd_out);
    Listen listen; /**< object that will listen for incoming connections */
    
    /** Default Not Found (404) handler
     * @param request the request instance
     * @param reply the reply instance
     * @param userContext context pointer (not used)
     */
    static void DefaultNotFound(const HttpRequest *request, HttpReply *reply, void *userContext);
    /** Default upload handler
     * @param request the request instance
     * @param reply the reply instance
     * @param userContext context pointer (not used)
     */
    static void DefaultUpload(const HttpRequest *request, HttpReply *reply, void *userContext);
    
    /** Give Listen class access to Httpd private members */
    friend class Listen;
    
    
    DISALLOW_COPY_AND_ASSIGN(Httpd);
    
};

};
#else
#error The Httpd class is only available when OPENMRN_FEATURE_BSD_SOCKETS is true.
#endif
#endif // __HTTPD_HXX

