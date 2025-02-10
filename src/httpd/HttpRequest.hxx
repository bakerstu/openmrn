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
//  Last Modified : <250210.0839>
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
#include "httpd/HttpReply.hxx"
#include <string>
#include <map>
#include <utility>

using String = std::string;

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
         * and write fds.  Socket will be closed in the HttpReply once
         * the reply is completely sent.
         */
        HASSERT(fdIn == fdOut);
    }
    using HeaderMap_t = std::multimap<String,String>;
    using HeaderMap_iterator_t = HeaderMap_t::iterator;
    size_t ContentLength() const {return contentLength_;}
    const String ContentType() const {return contentType_;}
    HTTPMethod Method() const {return method_;}
    const String RequestUri() const {return uri_;}
    int HttpVersion () const {return httpVersion_;}
    const String Host () const {return host_;}
    const String Query () const {return query_;}
    std::pair<HeaderMap_iterator_t,HeaderMap_iterator_t> HeaderValues (String header)
    {
        return headerMap_.equal_range(header);
    }
    const String Body () const {return body_;}
private:
    HttpReply *Reply (int version=0);
    
    HTTPMethod lookupMethodString(const char *methodString);
    /** Entry point to the state machine.  Read HTTP request line.
     * @return next state is process_requestline()
     */
    StateFlowBase::Action entry()
    {
        return read_single(&selectHelper, fdIn, line + pos, 
                           line_size - pos, STATE(process_requestline));
    }
    /** Process the first Http request line.
     * @return next state is read_header to collect the Http headers.
     */
    StateFlowBase::Action process_requestline();
    /** Read a Http header line.
     * @return next state is process_header to collect the Http headers.
     */
    StateFlowBase::Action read_header()
    {
        return read_single(&selectHelper, fdIn, line + pos,
                           line_size - pos, STATE(process_header));
    }
    /** Process a Http header line.
     * @return next state is read_header OR read_body OR handle_request.
     */
    StateFlowBase::Action process_header();
    /** Read the body.
     * @return next state is process_body OR handle_request.
     */
    StateFlowBase::Action read_body()
    {
        return read_single(&selectHelper, fdIn, line + pos,
                           line_size - pos, STATE(process_body));
    }
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
    char *line;       /**< current line content */
    size_t line_size; /**< current max line size */
    size_t pos;       /**< current line position */
    size_t contentLength_; /**< Content Length received */
    String contentType_;   /**< Content Type received */
    HTTPMethod method_;    /**< HTTP Method */
    String uri_;           /**< URI */
    int httpVersion_;      /**< HTTP Version */
    String host_;          /**< Host: header value */
    String query_;         /**< Query string */
    HeaderMap_t headerMap_; /**< All of the headers received. */
    String body_;          /**< The body of the request, if any */
    
    /** metadata for waiting on the http request socket to become active */
    StateFlowBase::StateFlowSelectHelper selectHelper;
    
    static const char *HttpMethodStrings[HTTP_METHOD_COUNT];

    DISALLOW_COPY_AND_ASSIGN(HttpRequest);
};
    


};
#endif // __HTTPREQUEST_HXX

