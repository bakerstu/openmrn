// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sun Feb 9 15:18:08 2025
//  Last Modified : <250210.0843>
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
/// @file Httpd.cxx
/// @author Robert Heller
/// @date Sun Feb 9 15:18:08 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#include <netinet/tcp.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "httpd/HTTP_Method.hxx"
#include "httpd/Uri.hxx"
#include "httpd/RequestHandler.hxx"
#include "httpd/HttpRequest.hxx"
#include "httpd/HttpReply.hxx"
#include "httpd/Httpd.hxx"
#include <string>

using String = std::string;

namespace HTTPD {

/*
 * Construct a Web Server
 */
Httpd::Httpd(ExecutorBase *executor, uint16_t port)
      : Service(executor)
, firstHandler_(nullptr)
, lastHandler_(nullptr)
, notFoundHandler_(DefaultNotFound,this)
, fileUploadHandler_(DefaultUpload,this)
, listen(this, port)
{
}

/*
 * Add Uri handler using default upload handler
 */
void Httpd::add_uri(const Uri &uri, HandlerFunction callback, void *context)
{
    addRequestHandler_(
          new FunctionRequestHandler(Httpd::UriHandler(callback,context),
                                     fileUploadHandler_,
                                     uri,HTTP_ANY));
    
}

/*
 * Add not found handler
 */
void Httpd::add_notfound(HandlerFunction callback, void *context)
{
    notFoundHandler_ = Httpd::UriHandler(callback,context);
}

/*
 * Add Upload handler
 */
void Httpd::add_upload(HandlerFunction callback, void *context)
{
    fileUploadHandler_ = Httpd::UriHandler(callback,context);
}

void Httpd::addRequestHandler_(RequestHandler* handler)
{
    if (!lastHandler_)
    {
        firstHandler_ = handler;
        lastHandler_  = handler;
    }
    else
    {
        lastHandler_->next(handler);
        lastHandler_ = handler;
    }
}

/*
 * Start a Http Request.
 */
void Httpd::open_request(int fd_in, int fd_out)
{
    new HttpRequest(this, fd_in, fd_out);
}

/*
 * Default not found handler
 */
void Httpd::DefaultNotFound(const HttpRequest *request, HttpReply *reply, void *userContext)
{
    reply->SetStatus(HttpStatusCode::Not_Found);
    reply->SetHeader("Content-Type","text/html");
    reply->Puts("<HTML><HEAD><TITLE>Not Found</TITLE></HEAD>\r\n");
    reply->Puts(String("<BODY><H1>")+
               request->RequestUri()+
               " Not Found</H1></BODY></HTML>\r\n");
    reply->SendReply();
}

/*
 * Default upload handler (not handle it gracefully).
 */
void Httpd::DefaultUpload(const HttpRequest *request, HttpReply *reply, void *userContext)
{
    reply->SetStatus(HttpStatusCode::Service_Unavailable);
    reply->SetHeader("Content-Type","text/html");
    reply->Puts("<HTML><HEAD><TITLE>Service Unavailable</TITLE></HEAD>\r\n");
    reply->Puts("<BODY><H1>Upload Service Unavailable</H1></BODY></HTML>\r\n");
    reply->SendReply();
}

/*
 * Httpd::Listen::Listen()
 */
Httpd::Listen::Listen(Service *service, int port)
      : StateFlowBase(service)
, fdListen(-1)
, selectHelper(this)
{
    HASSERT(port >= 0);
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
 * Httpd::Listen::accept()
 */
StateFlowBase::Action Httpd::Listen::accept()
{
    int newfd = ::accept(fdListen, NULL, NULL);
    if (newfd >= 0)
    {
        int yes = 1;
        int result = setsockopt(newfd, IPPROTO_TCP,
                                TCP_NODELAY, &yes, sizeof(int));
        HASSERT(result == 0);
        static_cast<Httpd *>(service())->open_request(newfd, newfd);
    }

    return listen_and_call(&selectHelper, fdListen, STATE(accept));
}


};

