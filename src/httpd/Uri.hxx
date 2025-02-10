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
//  Last Modified : <250209.1657>
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

class Uri {
protected:
    const String _uri;
public:
    Uri(const char *uri) : _uri(uri) {}
    Uri(const String &uri) : _uri(uri) {}
    virtual ~Uri() {}
    virtual Uri* clone() const {
        return new Uri(_uri);
    };
    virtual void initPathArgs(__attribute__((unused)) std::vector<String> &pathArgs) {}
    virtual bool canHandle(const String &requestUri, __attribute__((unused)) std::vector<String> &pathArgs) {
        return _uri == requestUri;
    }
};


class UriBraces : public Uri {
    
public:
    explicit UriBraces(const char *uri) : Uri(uri) {};
    explicit UriBraces(const String &uri) : Uri(uri) {};
    
    Uri* clone() const override final {
        return new UriBraces(_uri);
    };
    
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

class UriGlob : public Uri {

public:
    explicit UriGlob(const char *uri) : Uri(uri) {};
    explicit UriGlob(const String &uri) : Uri(uri) {};

    Uri* clone() const override final {
        return new UriGlob(_uri);
    };

    bool canHandle(const String &requestUri, __attribute__((unused)) std::vector<String> &pathArgs) override final {
        return fnmatch(_uri.c_str(), requestUri.c_str(), 0) == 0;
    }
};

class UriRegex : public Uri {

public:
    explicit UriRegex(const char *uri) : Uri(uri) {};
    explicit UriRegex(const String &uri) : Uri(uri) {};

    Uri* clone() const override final {
        return new UriRegex(_uri);
    };

    void initPathArgs(std::vector<String> &pathArgs) override final {
        std::regex rgx((_uri + "|").c_str());
        std::smatch matches;
        std::string s{""};
        std::regex_search(s, matches, rgx);
        pathArgs.resize(matches.size() - 1);
    }

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

