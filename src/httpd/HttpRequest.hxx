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
//  Last Modified : <250210.2009>
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


/** HTTP Request class.  Contains the Http request information, including the
 * request method, the Uri, the Http version.  Also all of the request headers.
 * A const pointer to an instanstance of this class is passed to the handler
 * function.
 */
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
    }
    /** Request headers container type */
    using HeaderMap_t = std::multimap<String,String>;
    /** Request headers container iterator type */
    using HeaderMap_iterator_t = HeaderMap_t::iterator;
    /** Content length accessor */
    size_t ContentLength() const {return contentLength_;}
    /** Content type accessor */
    const String ContentType() const {return contentType_;}
    /** Http Method accessor */
    HTTPMethod Method() const {return method_;}
    /** Request Uri accessor */
    const String RequestUri() const {return uri_;}
    /** Http version accessor */
    int HttpVersion () const {return httpVersion_;}
    /** Host accessor */
    const String Host () const {return host_;}
    /** Query string accessor */
    const String Query () const {return query_;}
    /** Header accessor
     * @param header Header name
     * @returns a pair of iterators giving the range of results
     */
    std::pair<HeaderMap_iterator_t,HeaderMap_iterator_t> HeaderValues (String header)
    {
        return headerMap_.equal_range(header);
    }
    /** Request body accessor */
    const String Body () const {return body_;}
private:
    /** Create a reply for this request.
     * @param version the version of the http request
     * @returns a new HttpReply instance to be filled in by the handler
     */
    HttpReply *Reply (int version=0);
    /** Lookup the method code for the request method
     * @param methodString the method string
     * @returns the method code
     */
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
    size_t hlineStart; /**< start of current header */
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
    /** The method string array. */
    static const char *HttpMethodStrings[HTTP_METHOD_COUNT];

    DISALLOW_COPY_AND_ASSIGN(HttpRequest);
};
    


};
#endif // __HTTPREQUEST_HXX

