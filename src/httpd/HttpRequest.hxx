// -!- c++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sat Feb 8 21:04:02 2025
//  Last Modified : <250208.2130>
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
/// @file HttpRequest.hxx
/// @author Robert Heller
/// @date Sat Feb 8 21:04:02 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#ifndef __HTTPREQUEST_HXX
#define __HTTPREQUEST_HXX

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "httpd/HTTP_Method.hxx"
#include "httpd/Uri.hxx"
#include "httpd/RequestHandler.hxx"
#include "httpd/HttpReply.hxx"

namespace HTTPD {

class HttpRequest : public StateFlowBase
{
public:
    /** Constructor.
     * @param service service instance that this HttpRequest belongs to
     * @param fd_in input file descriptor for the HttpRequest
     * @param fd_out output file descriptor for the HttpRequest
     */
    HttpRequest(Service *service, int fd_in, int fd_out);
    /** Desctructor.
     */
    ~HttpRequest()
    {
        /* should only delete sockets, which should have the same read
         * and write fds
         */
        HASSERT(fdIn == fdOut);
    }
private:
    /** Entry point to the state machine.  Read HTTP request line.
     * @return next state is process_requestline()
     */
    StateFlowBase::Action entry()
    {
          return read_single(&selectHelper, fdIn, line_, sizeof(line_),
                             STATE(process_requestline));
    }
    /** Process the first Http request line.
     * @return next state is read_header to collect the Http headers.
     */
    StateFlowBase::Action process_requestline();
    /** Read a Http header line.
     * @return next state is process_header to collect the Http headers.
     */
    StateFlowBase::Action read_header();
    /** Process a Http header line.
     * @return next state is read_header OR read_body OR handle_request.
     */
    StateFlowBase::Action process_header();
    /** Read the body.
     * @return next state is process_body OR handle_request.
     */
    StateFlowBase::Action read_body();
    /** Process a Http body.
     * @return next state is handle_request.
     */
    StateFlowBase::Action process_body();
    /** Handle Request.
     * @return next state delete_this
     */
    StateFlowBase::Action handle_request();
    
    int fdIn;         /**< input file descriptor of the HTTPRequest */
    int fdOut;        /**< output file descriptor of the HTTPRequest (passed to the HTTPReply) */
    char line[2048];  /**< current line buffer */
    
    /** metadata for waiting on the listen socket to become active */
    StateFlowBase::StateFlowSelectHelper selectHelper;
    
    DISALLOW_COPY_AND_ASSIGN(HTTPRequest);
};
    


};
#endif // __HTTPREQUEST_HXX

