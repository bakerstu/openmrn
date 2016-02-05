#!/usr/bin/python
# Copyright (c) 2013, Stuart W Baker
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are  permitted provided that the following conditions are met:
# 
#  - Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  - Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Create a source file with the CDI string.  This is an adaptation of
# "xmltoint.py" writen by Bob Jacobson.
#
# @author Stuart W. Baker
# @date 21 July 2013

import sys
import os
import time
import re
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-i", "--input", dest="input",
                  help="input file that will serve as the source XML",
                  metavar="FILE")
parser.add_option("-o", "--output", dest="output",
                  help="output file that will serve as the destination C code",
                  metavar="FILE")

(options, args) = parser.parse_args()

# the path is a required parameter
if options.input == None:
    parser.error('No input file specified')

# at least one target is required
if options.output == None:
    parser.error('No output file specified')

file_in = open(options.input, 'r')
file_out = open(options.output, 'w')

file_out.write('/* Generated code based off of ' + options.input + ' */\n\n')
file_out.write('#include <cstdint>\n#include <unistd.h>\n\n')
file_out.write('namespace nmranet {\n')
file_out.write('extern const char CDI_DATA[];\n')
file_out.write('const char CDI_DATA[] =\n{\n')

bytesPerGroup = 15
bytesPerLine = 70
cnt = 0
comment = ""
file_out.write("   ")
input_lines = file_in.readlines()
m = re.search("<!-- events: (.*)-->", input_lines[-1])
if m:
    event_offsets = m.group(1)
    input_lines.pop()
else:
    print("cound not find events: last line", input_lines[-1])
    event_offsets = None
input_chars = ''.join(input_lines)
for c in input_chars:
    if ord(c) < 100:
        file_out.write(" ")
    file_out.write(str(ord(c[0:1]))+", ")
    cnt = cnt+1
    if ord(c[0:1]) >= ord(' ') :
        comment = comment+c
    if cnt >= bytesPerLine :
        file_out.write("\n   /* ")
        file_out.write(comment)
        file_out.write(" */\n\n   ")
        cnt = 0
        comment = ""
    elif (cnt % bytesPerGroup) == 0:
        file_out.write("\n   ")
if cnt != 0 :
    file_out.write("   // | ")
    file_out.write(comment)
    file_out.write("|\n   ")    
file_out.write("0\n")
   
file_out.write('\n};\n')

if event_offsets:
    file_out.write('extern const uint16_t CDI_EVENT_OFFSETS[];\n');
    file_out.write('const uint16_t CDI_EVENT_OFFSETS[] = {%s0};\n\n' % event_offsets);

file_out.write('}  //namespace nmranet\n');

file_in.close()
file_out.close()
