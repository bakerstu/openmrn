// -!- c++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sat Feb 8 19:22:26 2025
//  Last Modified : <250208.2103>
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
/// @file RequestHandler.hxx
/// @author Robert Heller
/// @date Sat Feb 8 19:22:26 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#ifndef __REQUESTHANDLER_HXX
#define __REQUESTHANDLER_HXX

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "httpd/HTTP_Method.hxx"
#include "httpd/Uri.hxx"
#include "httpd/RequestHandler.hxx"
#include "httpd/HttpRequest.hxx"
#include "httpd/HttpReply.hxx"
#include "httpd/mimetable.hxx"

#include <vector>
#include <string>

using String = std::string;

namespace HTTPD {

class RequestHandler {
public:
    virtual ~RequestHandler() { }
    virtual bool canHandle(HTTPMethod method, String uri) { (void) method; (void) uri; return false; }
    virtual bool canUpload(String uri) { (void) uri; return false; }
    virtual void upload(WebServer& server, String requestUri, HTTPUpload& upload) { (void) server; (void) requestUri; (void) upload; }
#ifdef HTTP_RAW
    virtual bool canRaw(String uri) { (void) uri; return false; }
    virtual void raw(WebServer& server, String requestUri, HTTPRaw& raw) { (void) server; (void) requestUri; (void) raw; }
#endif
    virtual bool handle(WebServer& server, HTTPMethod requestMethod, String requestUri) { (void) server; (void) requestMethod; (void) requestUri; return false; }

    RequestHandler* next() { return _next; }
    void next(RequestHandler* r) { _next = r; }

private:
    RequestHandler* _next = nullptr;

protected:
    std::vector<String> pathArgs;

public:
    const String& pathArg(unsigned int i) { 
        HASSERT(i < pathArgs.size());
        return pathArgs[i];
    }
};


using namespace mime;

class FunctionRequestHandler : public RequestHandler {
public:
    FunctionRequestHandler(const Httpd::UriHandler &fn, 
                           const Httpd::UriHandler &ufn, 
                           const Uri &uri, HTTPMethod method)
    : _fn(fn)
    , _ufn(ufn)
    , _uri(uri.clone())
    , _method(method)
    {
        _uri->initPathArgs(pathArgs);
    }

    ~FunctionRequestHandler() {
        delete _uri;
    }

    bool canHandle(HTTPMethod requestMethod, String requestUri) override  {
        if (_method != HTTP_ANY && _method != requestMethod)
            return false;

        return _uri->canHandle(requestUri, pathArgs);
    }

    bool canUpload(String requestUri) override  {
        if (!_ufn || !canHandle(HTTP_POST, requestUri))
            return false;

        return true;
    }
#ifdef HTTP_RAW                                                                 
    bool canRaw(String requestUri) override {
        if (!_ufn || _method == HTTP_GET)
            return false;

        return true;
    }
    void raw(const HttpRequest &request, HttpReply &reply,String requestUri) override {
        if (canRaw(requestUri))
            _ufn.handle(request,reply);
    }
#endif
    
    bool handle(const HttpRequest &request, HttpReply &reply,HTTPMethod requestMethod, String requestUri) override {
        if (!canHandle(requestMethod, requestUri))
            return false;

        _fn.handle(request,reply);
        return true;
    }

    void upload(const HttpRequest &request, HttpReply &reply,String requestUri) override {
        if (canUpload(requestUri))
            _ufn.handle(request,reply)();
    }


protected:
    const Httpd::UriHandler _fn;
    const Httpd::UriHandler _ufn;
    Uri *_uri;
    HTTPMethod _method;
};

};
#endif // __REQUESTHANDLER_HXX

