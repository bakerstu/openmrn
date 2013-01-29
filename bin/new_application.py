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
# Create a new application tree from a template.
#
# @author Stuart W. Baker
# @date 24 January 2013

import sys
import os
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-t", "--targets", dest="targets",
                  default='"linux.x86 mach.x86 mach.x86_64"',
                  help="quoted list of targets: "
                       "(linux.x86 mach.x86 mach.x86_64 freertos.armv7m), "
                       "optionally a user defined suffix can be added, "
                       "for example: freertos.armv7m.ek-lm4f120xl "
                       "[default: %default]",
                  metavar="TARGETS")
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

# at least one target is required
if options.targets == None:
    parser.error('No target(s) specified')

# make sure we have a valid target parameter
targets = options.targets.split(' ')
for item in targets:
    subtarget = item.split('.')
    if ((subtarget[0] == 'linux' or subtarget[0] == 'mach' or subtarget[0] == 'freertos') and
        (subtarget[1] == 'x86' or subtarget[1] == 'x86_64' or subtarget[1] == 'armv7m')):
        continue
    else:
        parser.error('Unknown target: ' + item)

# create base tree:
# <basepath>/
#     doc/
#     include/
#     main.cxx
#     Makefile
#     targets/
cmd = 'mkdir ' + options.path
os.system(cmd)

cmd = 'mkdir ' + options.path + '/doc'
os.system(cmd)

cmd = 'mkdir ' + options.path + '/include'
os.system(cmd)

cmd = 'mkdir ' + options.path + '/targets'
os.system(cmd)

cmd = 'cp ../etc/templates/application/Makefile ' + options.path + '/'
os.system(cmd)

cmd = 'cp ../etc/templates/application/main.cxx ' + options.path + '/'
os.system(cmd)

# create <basepath>/targets/Makefile
makefile_in = open('../etc/templates/application/targets/Makefile', 'r')
makedata_in = makefile_in.read()
makefile_in.close()
makedata_out = makedata_in.split('##TARGETS##')
makefile_out = open(options.path + '/targets/Makefile', 'w')
makefile_out.write(makedata_out[0] + options.targets + makedata_out[1])
makefile_out.close()

# create sub-directories under <basepath>/
subdirs_file = open(options.path + '/subdirs', 'w')
subdirs_file.write('SUBDIRS =')
subdirs = options.subdirs.split(' ')
for item in subdirs:
    cmd = 'mkdir ' + options.path + '/' + item
    os.system(cmd)
    subdirs_file.write(' \\\n          ' + item)
subdirs_file.write('\n\n')
subdirs_file.close()

# create target directories under targets
# eg: <basepath>/targets/linux.x86
for target in targets:
    # create <basepath>/targets/<target>/
    cmd = 'mkdir ' + options.path + '/targets/' + target
    os.system(cmd)

    # create <basepath>/targets/<target>/lib
    cmd = 'mkdir ' + options.path + '/targets/' + target + '/lib'
    os.system(cmd)
    
    # create <basepath>/targets/<target>/lib/Makefile
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + subtarget[1] + '/lib/Makefile ' + options.path + '/targets/' + target + '/lib/Makefile'
    os.system(cmd)
    
    # prepare to strip off the user extention of the target
    subtarget = target.split('.')    
    
    # create <basepath>/targets/<target>/Makefile
    subdir_makefile_in = open('../etc/templates/application/targets/' +
                              subtarget[0] + '.' + subtarget[1] +
                              '/Makefile', 'r')
    subdir_makedata_in = subdir_makefile_in.read()
    subdir_makefile_in.close()
    subdir_makedata_out = subdir_makedata_in.split('##TARGET##')
    subdir_makefile_out = open(options.path + '/targets/' + target + 
                               '/' + '/Makefile', 'w')
    subdir_makefile_out.write(subdir_makedata_out[0] + 'TARGET = ' +
                              subtarget[0] + '.' + subtarget[1] +
                              subdir_makedata_out[1])
    subdir_makefile_out.close()
    
    # copy target specific *.ld, *.c, *.cxx, *.cpp, and *.S files to
    # <basepath>/targets/<target>/
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + \
          subtarget[1] + '/*.ld '  + options.path + '/targets/' + target + '/'
    os.system(cmd + ' &> /dev/null')
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + \
          subtarget[1] + '/*.c '   + options.path + '/targets/' + target + '/'
    os.system(cmd + ' &> /dev/null')
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + \
          subtarget[1] + '/*.cxx ' + options.path + '/targets/' + target + '/'
    os.system(cmd + ' &> /dev/null')
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + \
          subtarget[1] + '/*.cpp ' + options.path + '/targets/' + target + '/'
    os.system(cmd + ' &> /dev/null')
    cmd = 'cp ../etc/templates/application/targets/' + subtarget[0] + '.' + \
          subtarget[1] + '/*.S ' + options.path + '/targets/' + target + '/'
    os.system(cmd + ' &> /dev/null')
    
    # create all the <basepath>/targets/<target>/<subdirs>/Makefile
    subdir_makefile_in = open('../etc/templates/application/targets/' +
                              subtarget[0] + '.' + subtarget[1] +
                              '/subdir/Makefile', 'r')
    subdir_makedata_in = subdir_makefile_in.read()
    subdir_makefile_in.close()
    subdir_makedata_out = subdir_makedata_in.split('##TARGET##')
    for item in subdirs:
        cmd = 'mkdir ' + options.path + '/targets/' + target + '/' + item
        os.system(cmd)
        subdir_makefile_out = open(options.path + '/targets/' + target + 
                                   '/' + item + '/Makefile', 'w')
        subdir_makefile_out.write(subdir_makedata_out[0] + 'TARGET = ' +
                                  subtarget[0] + '.' + subtarget[1] +
                                  subdir_makedata_out[1])
        subdir_makefile_out.close()




