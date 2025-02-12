// -!- C++ -!- //////////////////////////////////////////////////////////////
//
//  System        : 
//  Module        : 
//  Object Name   : $RCSfile$
//  Revision      : $Revision$
//  Date          : $Date$
//  Author        : $Author$
//  Created By    : Robert Heller
//  Created       : Mon Feb 10 11:03:12 2025
//  Last Modified : <250210.1405>
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
/// @file main.cxx
/// @author Robert Heller
/// @date Mon Feb 10 11:03:12 2025
/// 
///
//////////////////////////////////////////////////////////////////////////////

#include "openmrn_features.h"
#include "utils/macros.h"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "httpd/Httpd.hxx"
#include "httpd/HttpRequest.hxx"
#include "httpd/HttpReply.hxx"
#include "httpd/Uri.hxx"

Executor<1> httpd_executor("httpd_executor", 0, 1024);
HTTPD::Httpd httpd(&httpd_executor,8081);

static void main_page(const HTTPD::HttpRequest *request, 
                      HTTPD::HttpReply *reply,void *userContext)
{
    reply->SetStatus(HTTPD::Httpd::HttpStatusCode::Data_follows);
    reply->SetHeader("Content-Type","text/html");
    reply->Puts("<HTML><HEAD><TITLE>Hello World</TITLE></HEAD>\r\n");
    reply->Puts("<BODY><H1>Hello World</H1></BODY></HTML>\r\n");
    reply->SendReply();
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    httpd.add_uri(HTTPD::Uri("/"),main_page,NULL);
    while (true)
    {
        {
            sleep(1);
        }
    }
    return 0;
}
