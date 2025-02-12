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
//  Last Modified : <250210.2049>
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
#include "httpd/Httpd.hxx"
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

/** Base class for Request handlers
 */
class RequestHandler {
public:
    /** Destructor
     */
    virtual ~RequestHandler() { }
    /** can handle predicate
     * @param method the request method
     * @param uri the request string
     * @returns true if this request handler can handle this uri
     */
    virtual bool canHandle(HTTPMethod method, String uri) /*override*/
    { 
        (void) method; 
        (void) uri; 
        return false; 
    }
    /** can upload predicate
     * @param method the request method
     * @param uri the request string
     * @returns true if this request handler can upload this uri
     */
    virtual bool canUpload(String uri) /*override */
    { 
        (void) uri; 
        return false; 
    }
    /** upload handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    virtual void upload(const HttpRequest *request, HttpReply *reply,
                        HTTPMethod requestMethod, String requestUri) /*override*/
    { 
        (void) request; 
        (void) reply;
        (void) requestMethod;
        (void) requestUri; 
    }
#ifdef HTTP_RAW
    /** The raw predicate
     * @param uri the uri
     * @returns true if this handler can handle a raw request
     */
    virtual bool canRaw(String uri) override
    { 
        (void) uri; 
        return false; 
    }
    /** raw handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    virtual void raw(const HttpRequest *request, HttpReply *reply,
                        HTTPMethod requestMethod, String requestUri) override
    { 
        (void) request; 
        (void) reply;
        (void) requestMethod;
        (void) requestUri; 
    }
#endif
    /** the handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    virtual void handle(const HttpRequest *request, HttpReply *reply,
                        HTTPMethod requestMethod, String requestUri) /*override*/
    { 
        (void) request; 
        (void) reply;
        (void) requestMethod;
        (void) requestUri; 
    }
    /** Next pointer accessor
     * @returns the next pointer
     */
    RequestHandler* next() 
    { 
        return _next; 
    }
    /** Next pointer setter
     * @param r the new next
     */
    void next(RequestHandler* r) 
    { 
        _next = r; 
    }

private:
    RequestHandler* _next = nullptr; /**< next pointer */

protected:
    std::vector<String> pathArgs; /**< the pathArgs */

public:
    /** Accessor for a path arg
     * @param i the index of the desired path arg
     * @returns the ith path arg
     */
    const String& pathArg(unsigned int i) 
    { 
        HASSERT(i < pathArgs.size());
        return pathArgs[i];
    }
};


using namespace mime;

/** Function request handler
 */
class FunctionRequestHandler : public RequestHandler {
public:
    /** Constructor
     * @param fn Uri handler for the handler function
     * @param ufn Uri handler for the upload function
     * @uri the uri match object
     * @method the request method for this handler
     */
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
    
    /** Destructor
     */
    ~FunctionRequestHandler() 
    {
        delete _uri;
    }
    
    /** can handle predicate
     * @param method the request method
     * @param uri the request string
     * @returns true if this request handler can handle this uri
     */
    bool canHandle(HTTPMethod requestMethod, String requestUri) override  
    {
        if (_method != HTTP_ANY && _method != requestMethod)
            return false;

        return _uri->canHandle(requestUri, pathArgs);
    }

    /** can upload predicate
     * @param method the request method
     * @param uri the request string
     * @returns true if this request handler can upload this uri
     */
    bool canUpload(String requestUri) override  
    {
        if (!_ufn || !canHandle(HTTP_POST, requestUri))
            return false;

        return true;
    }
#ifdef HTTP_RAW                                                                 
    /** The raw predicate
     * @param uri the uri
     * @returns true if this handler can handle a raw request
     */
    bool canRaw(String requestUri) override 
    {
        if (!_ufn || _method == HTTP_GET)
            return false;

        return true;
    }
    /** raw handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    void raw(const HttpRequest *request, HttpReply *reply,String requestUri) override 
    {
        if (canRaw(requestUri))
            _ufn.handle(request,reply);
    }
#endif
    
    /** the handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    void handle(const HttpRequest *request, HttpReply *reply,HTTPMethod requestMethod, String requestUri) override 
    {
        if (!canHandle(requestMethod, requestUri))
            return;

        _fn.handle(request,reply);
    }

    /** upload handler
     * @param request the http request
     * @param reply the http reply
     * @param requestMethod the request method
     * @param requestUri the request uri
     */
    void upload(const HttpRequest *request, HttpReply *reply,String requestUri) /*override */
    {
        if (canUpload(requestUri))
            _ufn.handle(request,reply);
    }


protected:
    const Httpd::UriHandler _fn; /**< handler function */
    const Httpd::UriHandler _ufn; /**< the upload handler */
    Uri *_uri;                    /**< the uri match */
    HTTPMethod _method;           /**< the request method */
};

};
#endif // __REQUESTHANDLER_HXX

