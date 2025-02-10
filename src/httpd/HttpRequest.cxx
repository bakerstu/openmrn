// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sun Feb 9 20:36:57 2025
//  Last Modified : <250210.1212>
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
/// @file HttpRequest.cxx
/// @author Robert Heller
/// @date Sun Feb 9 20:36:57 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "httpd/HTTP_Method.hxx"
#include "httpd/HttpReply.hxx"
#include "httpd/Httpd.hxx"
#include "httpd/HttpRequest.hxx"
#include "httpd/RequestHandler.hxx"
#include <string>
#include <map>
#include <utility>
#include <cstring>

using String = std::string;

namespace HTTPD {

HttpRequest::HttpRequest(Service *service, int fd_in, int fd_out)
      : StateFlowBase(service)
, fdIn(fd_in)
, fdOut(fd_out)
, line((char*)malloc(64))
, line_size(64)
, pos(0)
, contentLength_(0)
, selectHelper(this)
{
    /* start state flow */
    start_flow(STATE(entry));
}

HttpReply *HttpRequest::Reply (int version)
{
    return new HttpReply(static_cast<Httpd *>(service()),fdOut,version);
}

StateFlowBase::Action HttpRequest::process_requestline()
{
    size_t count = (line_size - pos) - selectHelper.remaining_;
    if (count == 0)
    {
        return delete_this();
    }
    
    while(count--)
    {
        if (line[pos++] == '\n')
        {
            size_t uri_start = 0;
            /* have the request line 0..pos-1. */
            for (size_t i = 0; i < pos; ++i)
            {
                if (line[i] == ' ')
                {
                    /* End of Method string */
                    line[i] = '\0';
                    method_ = lookupMethodString(line);
                    uri_start = i+1;
                    break;
                }
            }
            for (size_t i = uri_start; i < pos; ++i)
            {
                if (line[i] == ' ')
                {
                    /* End of URI string */
                    line[i] = '\0';
                    uri_ = String(&line[uri_start]);
                    httpVersion_ = atoi(&line[i+8]);
                    break;
                }
            }
            size_t question = uri_.find('?');
            if (question != String::npos)
            {
                query_ = uri_.substr(question+1);
            }
            else
            {
                query_ = "";
            }
            if (count == 0)
            {
                pos = 0;
                hlineStart = 0;
                return call_immediately(STATE(read_header));
            }
            else
            {
                hlineStart = pos;
                return call_immediately(STATE(process_header));
            }
        }
        else
        {
            if (pos >= line_size)
            {
                /* double the line buffer size */
                line_size *= 2;
                char *new_line = (char*)malloc(line_size);
                memcpy(new_line, line, pos);
                free(line);
                line = new_line;
            }
        }
    }
    return call_immediately(STATE(entry));
}

StateFlowBase::Action HttpRequest::process_header()
{
    size_t count = (line_size - pos) - selectHelper.remaining_;
    if (count == 0)
    {
        return delete_this();
    }
    
    while(count--)
    {
        if (line[pos++] == '\n')
        {
            String headerName;
            String headerValue;
            size_t v_start = 0;
            
            for (size_t i = hlineStart; i < pos; ++i)
            {
                if (line[i] == ':')
                {
                    line[i] = '\0';
                    headerName = String(&line[hlineStart]);
                    v_start = i+1;
                    break;
                }
            }
            if (v_start == 0)
            {
                if (contentLength_ == 0)
                {
                    return call_immediately(STATE(handle_request));
                }
                if (count == 0)
                {
                    pos = 0;
                    hlineStart = 0;
                    return call_immediately(STATE(read_body));
                }
                else
                {
                    hlineStart = pos;
                    return call_immediately(STATE(process_body));
                }
            }
            while (line[v_start] == ' ') v_start++;
            for (size_t i = pos-1; i > v_start; i--)
            {
                if (line[i] == '\n' || line[i] == '\r') 
                {
                    line[i] = '\0';
                    continue;
                }
                else
                {
                    break;
                }
            }
            headerValue = String(&line[v_start]);
            headerMap_.insert(std::pair<String,String>(headerName,headerValue));
            if (headerName == "Content-Length")
            {
                contentLength_ = std::stoi(headerValue);
            }
            else if (headerName == "Content-Type")
            {
                contentType_ = headerValue;
            }
            else if (headerName == "Host")
            {
                host_ = headerValue;
            }
            if (count == 0)
            {
                pos = 0;
                hlineStart = 0;
                return call_immediately(STATE(read_header));
            }
            else
            {
                hlineStart = pos;
                return again();
            }
        }
        else
        {
            if (pos >= line_size)
            {
                /* double the line buffer size */
                line_size *= 2;
                char *new_line = (char*)malloc(line_size);
                memcpy(new_line, line, pos);
                free(line);
                line = new_line;
            }
        }
    }
    return call_immediately(STATE(read_header));
}

StateFlowBase::Action HttpRequest::process_body()
{
    size_t count = (line_size - pos) - selectHelper.remaining_;
    if (count == 0)
    {
        return delete_this();
    }
    body_.append(&line[0],count);
    if (body_.length() < contentLength_)
    {
        pos = 0;
        return call_immediately(STATE(read_body));
    }
    else
    {
        return call_immediately(STATE(handle_request));
    }
}

StateFlowBase::Action HttpRequest::handle_request()
{
    Httpd *httpd = static_cast<Httpd *>(service());
    RequestHandler* handler;
    for (handler = httpd->FirstHandler(); handler; handler = handler->next()) 
    {
        if (handler->canHandle(method_, uri_)) break;
    }
    if (handler == nullptr)
    {
        httpd->NotFoundHandler()->handle(this,Reply(httpVersion_));
    }
    else
    {
        handler->handle(this,Reply(httpVersion_),method_,uri_);
    }
    return delete_this();
}

HTTPMethod HttpRequest::lookupMethodString(const char *methodString)
{
    for (int i = 0; i < HTTPMethod::HTTP_METHOD_COUNT; i++)
    {
        if (strncmp(methodString,HttpMethodStrings[i],strlen(HttpMethodStrings[i])) == 0)
        {
            return (HTTPMethod) i;
        }
    }
    return HTTP_ANY;
}

const char *HttpRequest::HttpMethodStrings[HTTP_METHOD_COUNT] = {
    "DELETE",
    "GET",
    "HEAD",
    "POST",
    "PUT"
};

};
