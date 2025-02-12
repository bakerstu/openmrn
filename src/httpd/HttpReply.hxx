// -!- c++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sun Feb 9 13:17:25 2025
//  Last Modified : <250210.2019>
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
/// @file HttpReply.hxx
/// @author Robert Heller
/// @date Sun Feb 9 13:17:25 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#ifndef __HTTPREPLY_HXX
#define __HTTPREPLY_HXX

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include <string>
#include <map>

using String = std::string;

namespace HTTPD {

class HttpReply : public StateFlowBase
{
public:
    /** Constructor.
     * @param service service instance that this HttpRequest belongs to
     * @param fd_out output file descriptor for the HttpRequest
     */
    HttpReply(Service *service, int fd_out, int version = 0)
                : StateFlowBase(service)
          , fdOut(fd_out)
          , version_(version)
          , status_(200)
          , selectHelper(this)
    {
    }
    /** Desctructor.  Close the socket.
     */
    ~HttpReply()
    {
        close(fdOut);
    }
    /** Add a header to the reply headers
     * @param header the header name
     * @param value the header's value
     */
    void SetHeader(const String header,const String value)
    {
        headerMap_.insert(std::pair<String,String>(header,value));
    }
    /** Set the statuss code
     * @param StatusCode The status code
     */
    void SetStatus(int StatusCode)
    {
        status_ = StatusCode;
    }
    /** Append text to the output body
     * @param s some text to add to the body
     */
    void Puts(const String s)
    {
        body_ += s;
    }
    /** Set the content type
     * @param contentType the content type string
     */
    inline void SetContentType(const String contentType)
    {
        SetHeader("Content-Type",contentType);
    }
    /** Start sending the reply
     */
    void SendReply()
    {
        start_flow(STATE(entry));
    }
private:
    /** Lookup the status message
     * @param statuscode The status code to lookup
     * @returns the status message
     */
    const char *lookupStatusMessage(int statuscode) const;
    /** Entry point to the state machine.  Write the reply headers.
     * @return next state is write_body
     */
    StateFlowBase::Action entry()
    {
        if (headerMap_.count("Content-Length") == 0)
        {
            SetHeader("Content-Length",std::to_string(body_.length()));
        }
        if (body_.length() > 0 && headerMap_.count("Content-Type") == 0)
        {
            SetContentType("application/octet-stream");
        }
        char buffer[2048];
        snprintf(buffer,sizeof(buffer),"HTTP/1.%d %d %s\r\n",version_,status_,lookupStatusMessage(status_));
        return write_repeated(&selectHelper, fdOut, buffer, strlen(buffer),
                              STATE(write_headers));
    }
    /** Start writing header lines. Initialize iterator and then call 
     *  write_nextheader
     */
    StateFlowBase::Action write_headers()
    {
        next_header_ = headerMap_.begin();
        return call_immediately(STATE(write_nextheader));
    }
    /** Write one header line, if more headers increment the iterator and 
     * write the next header.  Otherwise write a blank line and write the body.
     */
    StateFlowBase::Action write_nextheader()
    {
        if (next_header_ == headerMap_.end())
        {
            return write_repeated(&selectHelper, fdOut, "\r\n",
                                  2, STATE(write_body));
        }
        else
        {
            CurrentHeader_ = next_header_->first + ": " + next_header_->second + "\r\n";
            next_header_++;
            return write_repeated(&selectHelper, fdOut, 
                                  CurrentHeader_.c_str(), 
                                  CurrentHeader_.length(), 
                                  STATE(write_nextheader));
        }
    }
    /** Write out the body.
     * @return next state is delete_this.
     */
    StateFlowBase::Action write_body()
    {
        if (body_.length() == 0) {
            return call_immediately(STATE(delete_this));
        }
        else
        {
             return write_repeated(&selectHelper, fdOut, body_.c_str(),
                                  body_.length(), STATE(delete_this));
        }
    }
    int fdOut;        /**< output file descriptor of the HTTPReply */
    int version_;     /**< The http version */
    int status_;      /**< The reply status */
    std::multimap<String,String> headerMap_; /**< All of the headers to send. */
    std::multimap<String,String>::iterator next_header_;
    String CurrentHeader_; /**< Holds the current header line being sent */
    String body_;          /**< The body of the reply. */
    
    /** metadata for waiting on the http reply socket to become active */
    StateFlowBase::StateFlowSelectHelper selectHelper;
    
    DISALLOW_COPY_AND_ASSIGN(HttpReply);

};

};
#endif // __HTTPREPLY_HXX

