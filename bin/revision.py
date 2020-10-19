#!/usr/bin/env python
##
# Parse out a string array to be compiled into an executable for revision
# history
#
# @author Stuart W. Baker
# @data 28 April 2018

from optparse import OptionParser
import subprocess
import os
import time
import platform
import getpass
import filecmp

f_null = open(os.devnull, "w")

usage = "usage: %prog [options]\n\n" + \
        "  %prog -i $(APP_PATH) -o revisions.cxxout\n"

parser = OptionParser(usage=usage)
parser.add_option("-o", "--output", dest="output", metavar="FILE",
                  default="Revision",
                  help="generated output files (without the file extentions)")
parser.add_option("-i", "--input", dest="input", metavar="FILE",
                  default=None,
                  help="space seperated list of repository root paths")
parser.add_option("-d", "--dirty", dest="dirty", action="store_true",
                  default=False,
                  help="add the \"dirty\" suffix: -d")
parser.add_option("-u", "--untracked", dest="untracked", action="store_true",
                  default=False,
                  help="add the \"untracked files\" suffix: -u")
parser.add_option("-g", "--gcc", dest="gcc", metavar="`gcc -dumpversion`",
                  default=None,
                  help="add the GNU GCC version")
parser.add_option("-t", "--time", dest="date", action="store_true",
                  default=False,
                  help="add the date/time to the output")
parser.add_option("-H", "--host", dest="hostname", action="store_true",
                  default=False,
                  help="add the hostname to the output")
parser.add_option("-U", "--username", dest="username", action="store_true",
                  default=False,
                  help="add the username and hostname to the output")

(options, args) = parser.parse_args()

if (options.input == None) :
    parser.error('missing parameter -i')

print options.input

options.input = options.input.replace('  ', ' ')
inputs = options.input.split(" ")

orig_dir = os.path.abspath('./')
outputcxx = '#include <cstddef>\n\n'
outputcxx += '#include "utils/macros.h"\n'
outputcxx += '#include "utils/Revision.hxx"\n\n'
outputcxx += 'const char* Revision::REVISION[] = \n{\n'

#initialize hxx output to nothing
outputhxx = ''

# add data/time
if options.date :
    now = time.strftime("%a, %d %b %Y %H:%M:%S %Z")
    outputcxx += '    "' + now + '",\n'
    outputhxx += '"' + now + '\\n"\n'

# add user and host names
if options.username or options.hostname :
    outputcxx += '    "'
    outputhxx += '"'
    if options.username :
        username = getpass.getuser() + '@'
        outputcxx += username
        outputhxx += username
        options.hostname = True
    if options.hostname :
        hostname = platform.node()
        outputcxx += hostname
        outputhxx += hostname
    outputcxx +='",\n'
    outputhxx +='\\n"\n'

# add GCC version
if options.gcc != None :
    options.gcc = options.gcc.replace('.', '-')
    gcc = 'gcc-' + options.gcc
    outputcxx += '    "' + gcc + '",\n'
    outputhxx += '"' + gcc + '\\n"\n'

main_git_hash = None
    
for x in inputs :
    print x
    # go into the root of the repo
    os.chdir(orig_dir)
    os.chdir(x)

    # get the short hash
    git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'])
    git_hash = git_hash[:7]

    # get the dirty flag
    dirty = os.system('git diff --quiet')

    # get the untracked flag
    untracked_files = subprocess.check_output(['git', 'status', '-u', '-s'])
    untracked_files = untracked_files.splitlines()
    untracked = False
    for y in untracked_files :
        if y[0] == '?':
            untracked = True
            break

    # format the output
    outputcxx += '    "'
    outputcxx += git_hash
    outputcxx += ':' + os.path.split(os.path.abspath(x))[1]
    outputhxx += '"' + git_hash + ':' + os.path.split(os.path.abspath(x))[1]

    hashopts = ""
    
    if dirty or untracked :
        hashopts += ':'
    if dirty :
        hashopts += '-d'
    if untracked :
        hashopts += '-u'
    outputcxx += hashopts + '",\n'
    outputhxx += hashopts + '\\n"\n'

    if main_git_hash is None:
        main_git_hash = git_hash + hashopts

outputcxx += '    nullptr\n'
outputcxx += '};\n'
outputhxx = outputhxx[:-1]
outputhxx = outputhxx.replace('\n', '\n                      ')
outputhxx =  'CDI_GROUP(BuildRevisions, Name("Build Revisions"),\n' + \
             '          Description(' + outputhxx
outputhxx += '));\n'
outputhxx += 'CDI_GROUP_END();\n'

if main_git_hash is not None:
    outputhxx += '\n#define REVISION_GIT_HASH "' + main_git_hash + '"\n'

os.chdir(orig_dir)

# generate the *.cxxout style content
outputcxx += '\nsize_t Revision::count()\n'
outputcxx += '{\n'
outputcxx += '    return ARRAYSIZE(REVISION) - 1;\n'
outputcxx += '}\n\n'

# generate the *.hxxout file
output_file = open(options.output + 'Try.hxxout', 'w')
output_file.write(outputcxx)
output_file.write(outputhxx)
output_file.close()

# because a build may always run the script, we only want to replace the actual
# output files that get built if something has changed

diffhxx = subprocess.call(["diff",
                           "-I", """Sun, """, "-I", """Mon, """,
                           "-I", """Tue, """, "-I", """Wed, """,
                           "-I", """Thu, """, "-I", """Fri, """,
                           "-I", """Sat, """,
                           options.output + 'Try.hxxout',
                           options.output + '.hxxout'],
                          stdout=f_null, stderr=f_null)
diffcxx = subprocess.call(['diff',
                           "-I", """Sun, """, "-I", """Mon, """,
                           "-I", """Tue, """, "-I", """Wed, """,
                           "-I", """Thu, """, "-I", """Fri, """,
                           "-I", """Sat, """,
                           options.output + 'Try.cxxout',
                           options.output + '.cxxout'],
                          stdout=f_null, stderr=f_null)
# disable this because we are not actually writing a cxx file above.
diffcxx = 0

if diffhxx != 0 :
    os.system('rm -f ' + options.output + '.hxxout')
    os.system('mv ' + options.output + 'Try.hxxout ' + options.output + '.hxxout')

if diffcxx != 0 :
    os.system('rm -f ' + options.output + '.cxxout')
    os.system('mv ' + options.output + 'Try.cxxout ' + options.output + '.cxxout')



os.system('rm -f ' + options.output + 'Try.hxxout')
os.system('rm -f ' + options.output + 'Try.cxxout')
f_null.close()
