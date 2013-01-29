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
# Create a new application sub-director(y/ies) in an existing application tree.
#
# @author Stuart W. Baker
# @date 24 January 2013

import sys
import os
from optparse import OptionParser
import subprocess

parser = OptionParser()
parser.add_option("-p", "--path", dest="path", default=None,
                  help="absolute path to root of application tree",
                  metavar="PATH")
parser.add_option("-s", "--subdirs", dest="subdirs", default=None,
                  help="quoted list of source subdirectories",
                  metavar="SUBDIRS")

(options, args) = parser.parse_args()

# the path is a required parameter
if options.path == None:
    parser.error('No application root directory specified')

# at least one subdirectory is required
if options.subdirs == None:
    parser.error('No sub-director(y/ies) specified')

cmd = 'ls ' + options.path + '/targets/'
proc = subprocess.Popen(cmd,stdout=subprocess.PIPE,shell=True)
(targets, err) = proc.communicate()
targets = targets.replace('Makefile', '')
targets = targets.rstrip('\n')
print targets
print proc.returncode

if proc.returncode != 0:
    parser.error('Invalid application root directory specified')

subdirs = open(options.path + '/subdirs', 'r')
subdirs_data = subdirs.read()
subdirs.close()
for item in options.subdirs.split(' '):
    subdirs_data = subdirs_data.replace('SUBDIRS = \\', 'SUBDIRS = \\\n          ' + item + ' \\')
subdirs = open(options.path + '/subdirs', 'w')
subdirs.write(subdirs_data)
subdirs.close()

for target in targets.split('\n'):
    subtarget = target.split('.')
    subtarget = subtarget[0] + '.' + subtarget[1]


    # create all the <basepath>/targets/<target>/<subdirs>/Makefile
    subdir_makefile_in = open('../etc/templates/application/targets/' +
                              subtarget + '/subdir/Makefile', 'r')
    subdir_makedata_in = subdir_makefile_in.read()
    subdir_makefile_in.close()
    subdir_makedata_out = subdir_makedata_in.split('##TARGET##')
    for item in options.subdirs.split(' '):
        cmd = 'mkdir ' + options.path + '/targets/' + target + '/' + item
        os.system(cmd)
        subdir_makefile_out = open(options.path + '/targets/' + target + 
                                   '/' + item + '/Makefile', 'w')
        subdir_makefile_out.write(subdir_makedata_out[0] + 'TARGET = ' +
                                  subtarget + subdir_makedata_out[1])
        subdir_makefile_out.close()
   
