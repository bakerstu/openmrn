// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sat Feb 8 19:11:53 2025
//  Last Modified : <250210.2033>
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
/// @file Uri.hxx
/// @author Robert Heller
/// @date Sat Feb 8 19:11:53 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#ifndef __URI_HXX
#define __URI_HXX

#include <string>
#include <vector>
#include <regex>
#include <fnmatch.h>

using String = std::string;

namespace HTTPD {

/** Class for a plain constant uri (and base class for all Uris)
 * This class holds a Uri match string
 */
class Uri {
protected:
    const String _uri; /**< the uri string */
public:
    /** Constructor given a C char* string 
     * @param uri the uri
     */
    Uri(const char *uri) : _uri(uri) {}
    /** Constructor given a std::string 
     * @param uri the uri
     */
    Uri(const String &uri) : _uri(uri) {}
    /** Destructor */
    virtual ~Uri() {}
    /** Clone a Uri 
     * @returns a new Uri
     */
    virtual Uri* clone() const {
        return new Uri(_uri);
    };
    /** Initialize the PathArgs
     * @param pathArgs a vector of path elements
     */
    virtual void initPathArgs(__attribute__((unused)) std::vector<String> &pathArgs) {}
    /** Predicate to check if this Uri matches
     * @param requestUri the uri
     * @param pathArgs (not used)
     */
    virtual bool canHandle(const String &requestUri, __attribute__((unused)) std::vector<String> &pathArgs) {
        return _uri == requestUri;
    }
};

/** Class to use brace matching for uris
 */
class UriBraces : public Uri {
    
public:
    /** Constructor for char *
     * @param uri the uri
     */
    explicit UriBraces(const char *uri) : Uri(uri) {};
    /** Constructor for std::string
     * @param uri the uri
     */
    explicit UriBraces(const String &uri) : Uri(uri) {};
    
    /** Clone a Uri
     * @returns a new Uri
     */
    Uri* clone() const override final {
        return new UriBraces(_uri);
    };
    
    /** Initialize the PathArgs
     * @param pathArgs a vector of path elements
     */
    void initPathArgs(std::vector<String> &pathArgs) override final {
        size_t numParams = 0, start = 0;
        do {
            start = _uri.find("{}", start);
            if (start > 0) {
                numParams++;
                start += 2;
            }
        } while (start > 0);
        pathArgs.resize(numParams);
    }
    /** Predicate to check if this Uri matches
     * @param requestUri the uri
     * @param pathArgs the path args to fill in
     */
    bool canHandle(const String &requestUri, std::vector<String> &pathArgs) override final {
        if (Uri::canHandle(requestUri, pathArgs))
            return true;
        
        size_t uriLength = _uri.length();
        unsigned int pathArgIndex = 0;
        unsigned int requestUriIndex = 0;
        for (unsigned int i = 0; i < uriLength; i++, requestUriIndex++) {
            char uriChar = _uri[i];
            char requestUriChar = requestUri[requestUriIndex];
            
            if (uriChar == requestUriChar)
                continue;
            if (uriChar != '{')
                return false;
            
            i += 2; // index of char after '}'
            if (i >= uriLength) {
                // there is no char after '}'
                pathArgs[pathArgIndex] = requestUri.substr(requestUriIndex);
                return pathArgs[pathArgIndex].find("/") == String::npos; // path argument may not contain a '/'
            }
            else
            {
                char charEnd = _uri[i];
                size_t uriIndex = requestUri.find(charEnd, requestUriIndex);
                if (uriIndex == String::npos)
                    return false;
                pathArgs[pathArgIndex] = requestUri.substr(requestUriIndex, uriIndex-requestUriIndex);
                requestUriIndex = (unsigned int) uriIndex;
            }
            pathArgIndex++;
        }
        
        return requestUriIndex >= requestUri.length();
    }
};

/** Class to use glob matching
 */
class UriGlob : public Uri {

public:
    /** Constructor given a C char* string 
     * @param uri the uri
     */
    explicit UriGlob(const char *uri) : Uri(uri) {};
    /** Constructor given a std::string 
     * @param uri the uri
     */
    explicit UriGlob(const String &uri) : Uri(uri) {};

    /** Clone a Uri 
     * @returns a new Uri
     */
    Uri* clone() const override final {
        return new UriGlob(_uri);
    };

    /** Predicate to check if this Uri matches
     * @param requestUri the uri
     * @param pathArgs (not used)
     */
    bool canHandle(const String &requestUri, __attribute__((unused)) std::vector<String> &pathArgs) override final {
        return fnmatch(_uri.c_str(), requestUri.c_str(), 0) == 0;
    }
};


/** Class to use Regular Expression matching
 */
class UriRegex : public Uri {

public:
    /** Constructor for char *
     * @param uri the uri
     */
    explicit UriRegex(const char *uri) : Uri(uri) {};
    /** Constructor for std::string
     * @param uri the uri
     */
    explicit UriRegex(const String &uri) : Uri(uri) {};

    /** Clone a Uri
     * @returns a new Uri
     */
    Uri* clone() const override final {
        return new UriRegex(_uri);
    };

    /** Initialize the PathArgs
     * @param pathArgs a vector of path elements
     */
    void initPathArgs(std::vector<String> &pathArgs) override final {
        std::regex rgx((_uri + "|").c_str());
        std::smatch matches;
        std::string s{""};
        std::regex_search(s, matches, rgx);
        pathArgs.resize(matches.size() - 1);
    }

    /** Predicate to check if this Uri matches
     * @param requestUri the uri
     * @param pathArgs the path args to fill in
     */
    bool canHandle(const String &requestUri, std::vector<String> &pathArgs) override final {
        if (Uri::canHandle(requestUri, pathArgs))
            return true;

        unsigned int pathArgIndex = 0;
        std::regex rgx(_uri.c_str());
        std::smatch matches;
        std::string s(requestUri.c_str());
        if (std::regex_search(s, matches, rgx)) {
            for (size_t i = 1; i < matches.size(); ++i) {  // skip first
                pathArgs[pathArgIndex] = String(matches[i].str().c_str());
                pathArgIndex++;
            }
            return true;
        }
        return false;
    }
};

};

#endif // __URI_HXX

