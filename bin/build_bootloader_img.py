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
# Converts a binary bootloader file into a C source with arrays representing
# the binary file to flash.  This is an adaptation of "xmltoint.py" written by
# Bob Jacobson.
#
# @author Balazs Racz
# @date 27 Oct 2015

import sys
import os
import time
from optparse import OptionParser

class Block:
  pass

parser = OptionParser()
parser.add_option("-i", "--input", dest="input",
                  help="input file that will serve as the source BIN",
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

file_in = open(options.input, 'rb')
file_out = open(options.output, 'w')

file_out.write('/* Generated code based off of ' + options.input + ' */\n\n')
file_out.write('#include <cstdint>\n#include <unistd.h>\n\n')
file_out.write('#include "utils/ReflashBootloader.hxx"\n\n')
file_out.write('extern const SegmentTable table[];\n')

bin_file = []
ofs = 0
while True :
    c = file_in.read(1)
    if c == "" :
        break
    bin_file.append(c)
    ofs = ofs + 1
file_in.close()

ofs = 0
last_end = -10000

blocks = []
while ofs < len(bin_file):
  blocks.append(bin_file[ofs:ofs + 1024])
  ofs = ofs + 1024

outputblocks = []

for k, block in enumerate(blocks):
  if k == 0:
    b = Block()
    b.ofs = 0
    b.data = block
    b.end_ofs = 0
    outputblocks.append(b)
    continue
  if block.count('\0') == len(block):  # all zeros
    continue
  if outputblocks[-1].end_ofs == k - 1:
    outputblocks[-1].data += block
    outputblocks[-1].end_ofs = k
  else:
    b = Block()
    b.ofs = k
    b.data = block
    b.end_ofs = k
    outputblocks.append(b)

segmenttable = ''
for b in outputblocks:
  file_out.write('const uint8_t payload_%d[] =\n{\n  ' % b.ofs)
  for i, c in enumerate(b.data):
    file_out.write("0x%02x, " % ord(c))
    if (i + 1) % 12 == 0:
      file_out.write('\n  ')
  file_out.write('\n};\n\n')
  segmenttable += '  { (uint8_t*)0x%x, payload_%d, %d},\n' % (b.ofs * 1024, b.ofs, len(b.data))
segmenttable += '  { 0, NULL, 0},\n'

file_out.write('const SegmentTable table[] =\n{\n')
file_out.write(segmenttable)
file_out.write('};\n\n')

file_out.close()
