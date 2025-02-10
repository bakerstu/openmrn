// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Sun Feb 9 15:04:53 2025
//  Last Modified : <250209.1505>
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
/// @file mimetable.cxx
/// @author Robert Heller
/// @date Sun Feb 9 15:04:53 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#include "mimetable.hxx"

namespace mime
{

// Table of extension->MIME
const Entry mimeTable[maxType] = 
{
    { ".html", "text/html" },
    { ".htm", "text/html" },
    { ".css", "text/css" },
    { ".txt", "text/plain" },
    { ".js", "application/javascript" },
    { ".json", "application/json" },
    { ".png", "image/png" },
    { ".gif", "image/gif" },
    { ".jpg", "image/jpeg" },
    { ".ico", "image/x-icon" },
    { ".svg", "image/svg+xml" },
    { ".ttf", "application/x-font-ttf" },
    { ".otf", "application/x-font-opentype" },
    { ".woff", "application/font-woff" },
    { ".woff2", "application/font-woff2" },
    { ".eot", "application/vnd.ms-fontobject" },
    { ".sfnt", "application/font-sfnt" },
    { ".xml", "text/xml" },
    { ".pdf", "application/pdf" },
    { ".zip", "application/zip" },
    { ".gz", "application/x-gzip" },
    { ".appcache", "text/cache-manifest" },
    { "", "application/octet-stream" } 
};

}
