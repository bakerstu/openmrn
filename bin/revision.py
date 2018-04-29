#!/usr/bin/env python
##
# Parse out a string array to be compiled into an executable for revision
# history
#
# @author Stuart W. Baker
# @data 28 April 2018

from optparse import OptionParser
import os
import time
import platform
import getpass

usage = "usage: %prog [options]\n\n" + \
        "  %prog -i $(APP_PATH) -o revisions.cxxout\n"

parser = OptionParser(usage=usage)
parser.add_option("-o", "--output", dest="output", metavar="FILE",
                  default="revisions.cxxout",
                  help="generated output file")
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
output = '#include <cstddef>\n\n'
output += 'const char *REVISIONS[] = \n{\n'

# add data/time
if options.date :
    output += '    "' + time.strftime("%a, %d %b %Y %H:%M:%S %Z") + '",\n'

# add user and host names
if options.username or options.hostname :
    output += '    "'
    if options.username :
        output += getpass.getuser() + '@'
        options.hostname = True
    if options.hostname :
        output += platform.node()
    output +='",\n'

# add GCC version
if options.gcc != None :
    options.gcc = options.gcc.replace('.', '-')
    output += '    "gcc-' + options.gcc + '",\n'

for x in inputs :
    print x
    # go into the root of the repo
    os.chdir(orig_dir)
    os.chdir(x)

    # get the short hash
    os.system('git rev-parse --short HEAD > /tmp/git_hash')

    # get the dirty flag
    dirty = os.system('git diff --quiet')

    # get the untracked flag
    os.system('git status -u -s > /tmp/git_untracked')

    # format the output
    git_hash_file = open('/tmp/git_hash', 'r')
    output += '    "'
    output += git_hash_file.read(7)
    output += ':' + os.path.split(os.path.abspath(x))[1]

    if dirty or os.stat('/tmp/git_untracked').st_size != 0 :
        output += ':'
    if dirty :
        output += '-d'
    if os.stat('/tmp/git_untracked').st_size != 0 :
        output += '-u'
    output += '",\n'

os.chdir(orig_dir)
output += '    nullptr\n'
output += '};\n'
output_file = open(options.output, 'w')
output_file.write(output)
output_file.close()

os.system('rm -f /tmp/git_hash')
os.system('rm -f /tmp/git_untracked')

