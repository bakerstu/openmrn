#!/usr/bin/python
# powerpc-gekko-objdump -d example.elf | python cg.py | dot -Tpng > cg.png && eog cg.png

import re, sys, math, subprocess, getopt, bisect

## These regular expressions match on the ARM assembler output (and maybe
# MIPS). @todo: update this regexp to also match x86 output.

# Matches a new symbol start, typically the start of a function.
f_re = re.compile('(^[0-9a-f]*) <([a-zA-Z0-9_:()*., ]*)>:$')

# Matches a function call, when one function depends on another. 
c_re = re.compile('\t(b[xl]?|jal)[ \t]*[0-9a-f]* <([a-zA-Z0-9_:()*, ]*)>')

# Matches an embedded address of a symbol, typically when one function takesthe
# address of another (to do indirect call etc.)
a_re = re.compile('\t.word[ \t]*0x([0-9a-f]*)$')

# This matches every line that takes up code space, by the hex address at the
# beginning of the line. The last of these lines in a block of code will
# determine how long that block of code is.
pa_re = re.compile('(^[0-9a-f]*):\t')

# Regexp for finding template arguments
tmpl_re = re.compile('.*(<[^<>]*>).*')

_blacklist_pre = ['__', 'str', 'f_', 'VIDEO_']
_blacklist_cont = ['printf']

_blacklist = set(['diewith', '__diewith_from_arm', 'memcpy', 'os_mutex_lock',
                  'os_mutex_unlock', 'malloc', 'free', 'abort',
                  'vPortEnterCritical', 'vPortExitCritical',
                  'xTaskResumeAll', 'vTaskSuspendAll',
                  'xQueueGenericSend', 'xQueueGenericReceive',
                  '_ZdlPv', '_Znwj',  # operator new and delete
                  ])
# '_vfprintf_r',
_blacklist_pre = []
_blacklist_cont = []

class Flags:
  pass

FLAGS = Flags()

FLAGS.verbose = False
FLAGS.map_file = None
FLAGS.min_size = 0
FLAGS.demangle = True
FLAGS.strip_template = True


def Blacklist(s):
    if s in _blacklist: return True
    for f in _blacklist_cont:
        if f in s: return True
    for f in _blacklist_pre:
        if s.startswith(f): return True
    return False

def escape(s):
  try:
    ret = re.sub('[\*\(\)\-\:\+ ,/.]', '_', s)
    return ret
  except Exception, e:
    print >> sys.stderr, e
    print >> sys.stderr, "bad type of s: " , s
    return "xxxxxxx"

all_symbols = dict()
enmangle = dict()
address_lookup = dict()
sections = dict()
section_addresses = []
object_expn = dict()  # object escaped name -> called symbol name
object_parent = dict()  # object escaped name -> caller object escaped name


def GetSymbol(name, address = None):
  if name in all_symbols:
    return all_symbols[name]
  new_object = Symbol(name)
  all_symbols[name] = new_object
  new_object.address = address
  return new_object

class MapEntry(object):
  """Represents a line read from the .map file"""

  __slots__ = ('section',
               'subsection',
               'address',
               'length',
               'library',
               'objfile',
               'function',
               )
  def __str__(self):
    return "MapEntry section %s subsection %s address 0x%8x length %s lib %s obj %s fun %s" % (
      self.section,self.subsection,self.address,self.length,self.library,self.objfile,self.function) 

def StripTemplate(name):
  while True:
    m = tmpl_re.match(name)
    if m is None: break
    name = name[:m.start(1)] + name[m.end(1):]
  return name
  
class Symbol(object):
  """Represents one linker symbol in the linked executable"""

  __slots__ = ('name',  # linker name (mangled)
               'address', #in the memory map
               'objfile', #object file from linker
               'displayname',  # displayname (possibly unmangled)
               'codesize',  # Code space used in bytes
               'deps',  # other symbols it references
               'indeps', # symbols referencing this
               'cycle', # a symbol name which references this circular
                        # dependencies
               'total_code_size', # total size this symbol is responsible for
               'blacklisted',  #whether to skip outputting edges to it
               'in_cycle_edge_count', #how many in-edges are from cycles
               'has_cycle', #whether the current node was involved in a cycle
               'removed_by_filter', #true if a filter killed this node
               )

  def __init__(self, name):
    self.deps = dict()
    self.indeps = dict()
    self.cycle = None
    self.name = name
    self.displayname = name
    self.codesize = 0
    self.in_cycle_edge_count = 0
    self.blacklisted = Blacklist(name)
    self.removed_by_filter = False
    self.objfile = None

  def AddDep(self, dep):
    other_symbol = GetSymbol(dep)
    self.deps[dep] = other_symbol
    other_symbol.indeps[self.name] = self

  def PrintNode(self):
    cyclestring = ""
    comment = "//" if self.removed_by_filter else ""
    try:
      if self.has_cycle: cyclestring = "c"
    except AttributeError, err:
      print >>sys.stderr,"Not processed? ", self.name
      cyclestring = "N"
      self.total_code_size = self.codesize
    if FLAGS.strip_template:
      displayname = StripTemplate(self.displayname)
    else:
      displayname = self.displayname
    return ("%s%s [shape=box, label=\"%s\\n%d / %d %s\"];" %
            (comment, escape(self.name), displayname, self.codesize, self.total_code_size,
             cyclestring))

  def DebugString(self):
    d = ""
    d += ("name = %s " % self.name)
    d += ("address = %x " % self.address)
    d += ("objfile = %s " % self.objfile)
    d += ("in edge count = %d " % len(self.indeps))
    d += ("in cycle count = %d " % self.in_cycle_edge_count)
    return d


def Demangle(names):
    args = ['c++filt']
    args.extend(names)
    pipe = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    pstdout, _ = pipe.communicate()
    #print >> pstdin, "\n".join(names)
    demangled = pstdout.split("\n")

    # Each line ends with a newline, so the final entry of the split output
    # will always be ''.
    assert len(demangled) == len(names)+1

    f = open('demangled.txt', 'w')
    for i in xrange(len(demangled)-1):
      print >>f, names[i], demangled[i]
    f.close()
    return demangled[:-1]

def ReadLstFile(f):
  current_symbol = None
  last_offset = 0;
  address_edges = []
  last_seen_address = None
  for line in f:
    line = line.strip()
    m = f_re.match(line)
    if m:  # we matched a new symbol start
      name = m.group(2)
      offset = int(m.group(1), 16)
      if current_symbol:
        if offset < last_offset:
          current_symbol.codesize = last_seen_address - last_offset + 4
        else:
          current_symbol.codesize = offset - last_offset
        print >>sys.stderr, "last symbol ", current_symbol.name ," size: ", current_symbol.codesize
      current_symbol = GetSymbol(name, offset)
      if current_symbol.address is None:
        current_symbol.address = offset
      elif current_symbol.address != offset:
        print >>sys.stderr, "multiple addresses: %08x %08x for %s" % (current_symbol.address, offset, current_symbol.name)
      if offset:
        address_lookup[offset + 1] = current_symbol
        #address_lookup[offset] = current_symbol  # creates false positives
      last_offset = offset
      #s = cg[m.group(2)] = []
      #print "function: ", m.group(1)

    m = c_re.search(line)
    if m:
      dest = m.group(2)
      #print >>sys.stderr, "call: ", dest
      if current_symbol:
        current_symbol.AddDep(dest)

    m = a_re.search(line)
    if m:
      dest_offset = int(m.group(1), 16)
      address_edges.append((current_symbol, dest_offset))

    m = pa_re.match(line)
    if m:
      last_seen_address = int(m.group(1), 16)

  for (symbol, dst_address) in address_edges:
    if dst_address not in address_lookup: continue
    dstname = address_lookup[dst_address].name
    print >>sys.stderr, "Address edge: %s -> %08x %s" % (
        symbol.name, dst_address, dstname)
    symbol.AddDep(dstname)


def ReadMapFile(f):
  print >>sys.stderr, "reading map file."
  cpptext_re = re.compile('^ [.]text[.](.*)$')
  obj_re = re.compile('^ ([.]text)?[ \t]+0x([0-9a-f]*)[ \t]+0x([0-9a-f]*)[ \t]+(?:([a-z/A-Z0-9_\.]*[.]a)[\(])?([a-zA-Z_-]*[.]o)[\)]?$')

  section_start_re = re.compile('Linker script and memory map')
  in_section = False
  section_re = re.compile('^ (?:[.](?P<section>[-a-zA-Z_]+)(?:[.](?P<subsection>[^ \t\n]+))?)?(?:[ \t]+0x(?P<address>[0-9a-fA-F]+)[ \t]+(?:0x(?P<length>[0-9a-fA-F]+) (?:(?P<library>.*[.]a)[\(])?(?P<object>.*[.]o)[\)]?|(?P<function>[^0].*)))?$');
  count = 0;
  entries = []
  end_of_reason_re = re.compile('Allocating common symbols')
  in_reason = True
  

  pulled_re = re.compile('^(?P<library>.*[.]a)[\(](?P<object>.*[.]o)[\)]$');
  caller_re = re.compile('^ {30}(?:(?P<library>.*[.]a)[\(])?(?P<object>.*[.]o)[\)]? [\(](?P<function>.*)[\)]$');

  for line in f:
    printed = False
    """
    m = cpptext_re.match(line)
    if m:
      if not printed:
        print >>sys.stderr, line
        printed = True
      print >>sys.stderr, "matched cpp text section ", m.group(1)
    m = obj_re.match(line)
    if m:
      if not printed:
        print >>sys.stderr, line
        printed = True
      print >>sys.stderr, "matched obj/text ", m.groups() 
"""
    m = end_of_reason_re.match(line)
    if m:
      in_reason = False
    if in_reason:
      m = pulled_re.match(line)
      if m:
        dst_lib = m.group('library')
        dst_obj = m.group('object')
        print >>sys.stderr, ("pulled lib %s obj %s" % (dst_lib, dst_obj))
      m = caller_re.match(line)
      if m:
        src_lib = m.group('library')
        src_obj = m.group('object')
        src_fun = m.group('function')
        print >>sys.stderr, ("called by lib %s obj %s fun %s" % (src_lib, src_obj, src_fun))
        # we try to assign a symbol name to the function
        sym_name = None
        symbol = None
        if src_fun in all_symbols:
          sym_name = src_fun
          symbol = all_symbols[src_fun]
        elif src_fun in enmangle and enmangle[src_fun] in all_symbols:
          sym_name = enmangle[src_fun]
          symbol = all_symbols[sym_name]
        elif "__" + src_fun in all_symbols:
          sym_name = "__" + src_fun
          symbol = all_symbols[sym_name]
        elif src_fun.replace("D2Ev", "D1Ev") in all_symbols:
          sym_name = src_fun.replace("D2Ev", "D1Ev")
          symbol = all_symbols[sym_name]
        else:
          print >>sys.stderr, "Cound not resolve function name to symbol: ", src_fun
        if sym_name:
          object_expn[escape(dst_obj)] = symbol
          print >>sys.stderr, ("expn[%s] = %s"% ( escape(dst_obj), symbol.name))
        object_parent[escape(dst_obj)] = escape(src_obj)
    m = section_start_re.match(line)
    if m:
      in_section = True
      section = None
    if in_section:
      m = section_re.match(line)
    else:
      m = None
    if m:
      count = count + 1
      if False and FLAGS.verbose and not printed:
        print >>sys.stderr, line
        printed = True
        print >>sys.stderr, "matched textsection ", m.groups() 
      if m.group('section'):
        section = m.group('section')
        subsection = None
        if section[:6] == 'debug_':
          continue
      if m.group('subsection'):
        subsection = m.group('subsection')
      if m.group('object'):
        objfile = m.group('object')
        library = m.group('library')
      if m.group('length') is not None and m.group('function') is not None:
        print >>sys.stderr, line
        print >>sys.stderr, "length %s and function %s " % (m.group('length'), m.group('function')), m.groups() 
      if section is not None and m.group('function') is not None:
        entry = MapEntry()
        entry.section = section
        entry.subsection = subsection
        entry.address = int(m.group('address'), 16)
        entry.length = None
        if m.group('length') is not None:
          entry.length = int(m.group('length'), 16)
        entry.objfile = objfile
        entry.library = library
        entry.function = m.group('function')
        entries.append(entry)
      if section is not None and m.group('address') is not None and m.group('length') is not None:
        entry = MapEntry()
        entry.section = section
        entry.subsection = subsection
        entry.address = int(m.group('address'), 16)
        entry.length = int(m.group('length'), 16)
        entry.objfile = objfile
        entry.library = library
        section_addresses.append(entry.address)
        sections[entry.address] = entry
  print >>sys.stderr, "found %d lines, %d entries." % (count, len(entries))
  return entries


def ProcessMapEntries(entries):
  enmangle_found_count = 0;
  direct_found_count = 0;
  subsection_found_count = 0;
  not_found_count = 0;
  alias_count = 0;
  last_address = 0;
  for e in entries:
    print >> sys.stderr, "sym: ", e
    if e.address == 0:
      continue;
    if e.function == "(size before relaxing)":
      continue;
    if ((0xbd000000 <= e.address and
        e.address < 0xbe000000) or
        (e.address < 512*1024)):
      # Interesting function
      if e.function in all_symbols:
        symbol = all_symbols[e.function]
        direct_found_count += 1
      elif e.function in enmangle and enmangle[e.function] in all_symbols:
        symbol = all_symbols[enmangle[e.function]]
        enmangle_found_count += 1
        #print >>sys.stderr, "found enmanged symbol for %s: %s"%(e.function, symbol.name)
      elif e.subsection in all_symbols and e.address == all_symbols[e.subsection].address:
        #print >>sys.stderr, "subsection symbol found: ", e, all_symbols[e.subsection].address
        symbol = all_symbols[e.subsection]
        subsection_found_count += 1
      elif e.address + 1 in address_lookup:
        print >>sys.stderr, ("symbol 0x%x is an alias of %s: " % (e.address, address_lookup[e.address + 1].name)), e
        alias_count += 1
        continue
      else:
        print >>sys.stderr, "symbol not found: ", e
        not_found_count += 1
        continue
      last_address = e.address
      if e.objfile is not None:
        symbol.objfile = escape(e.objfile)
  print >>sys.stderr, ('map symbol lookup: found %d via direct, %d via enmangle, %d via subsection, %d aliased, %d not found' % (direct_found_count, enmangle_found_count, subsection_found_count, alias_count, not_found_count))
  missing_count = 0
  section_addresses.sort()
  for sym in all_symbols.itervalues():
    if sym.objfile is not None:
      continue
    # lookup section
    i = bisect.bisect_right(section_addresses, sym.address)
    print >>sys.stderr, ("bisect lookup address %x index %d, section count %d" % (sym.address, i, len(section_addresses)))
    if i:
      section = sections[section_addresses[i-1]]
      if not (section.address <= sym.address and (section.address + section.length) > sym.address):
        print >>sys.stderr, ("section address mismatch: name %s, address %x, section address %x, section length %d" % (sym.name, sym.address, section.address, section.length))
      elif section.subsection is not None and section.address == sym.address and section.subsection != sym.name and  sym.name != re.sub('D2Ev', 'D1Ev', section.subsection):
        print >>sys.stderr, ("subsection mismatch: name %s, subsection %s" % (sym.name, section.subsection))
      else:
        print >>sys.stderr, "objfile found for symbol: ", sym.name, (" :@%x " % section.address), section.objfile
        sym.objfile = escape(section.objfile)
        continue
    else:
      print >>sys.stderr, ("Bisect address lookup failed: address %x" % sym.address)
    missing_count += 1
    print >>sys.stderr, "symbol missing objfile: ", sym.name
  print >>sys.stderr, ("%d symbols missing objfile" % missing_count)

def TraverseSymbol(pending_set, visited_set, symbol, depth):
  """returns the depth of the minimum cycle that was found"""
  if symbol.name in visited_set:
    return depth
  cycle = depth
  pending_set[symbol.name] = depth
  symbol.in_cycle_edge_count = 0
  total_size = symbol.codesize * 1.0
  for dep_symbol in symbol.deps.itervalues():
    if dep_symbol.name in pending_set:
      cycle = min(cycle, pending_set[dep_symbol.name])
      dep_symbol.in_cycle_edge_count += 1
      continue
    new_cycle = TraverseSymbol(pending_set, visited_set, dep_symbol, depth + 1)
    if new_cycle < cycle:
      print >>sys.stderr,"cycle %s/%d-> %s/%d"% (symbol.name, depth,
                                                 dep_symbol.name, new_cycle)
    cycle = min(new_cycle, cycle)
    dep_edge_fraction = (len(dep_symbol.indeps) -
                         dep_symbol.in_cycle_edge_count)
    if dep_edge_fraction > 0:
      total_size += (dep_symbol.total_code_size / dep_edge_fraction)
    else:
      print >>sys.stderr, "Symbol %s -> %s has no dep_edge." % (
          symbol.name, dep_symbol.name)
  symbol.total_code_size = total_size
  del pending_set[symbol.name]
  visited_set.add(symbol.name)
  symbol.has_cycle = (cycle < depth)
  return cycle


def BindSymbolsToMain():
  # Takes any symbols with no in-edges and binds them to an object file,
  # another symbol or to the main binary.
  mainfile = GetSymbol('binary')
  v = [v for v in all_symbols.itervalues()]
  for symbol in v:
    if symbol.name == "binary": continue
    if (len(symbol.indeps) == 0 or len(symbol.indeps) == symbol.in_cycle_edge_count):
      print >>sys.stderr, "symbol %s with no indeps." % symbol.name
      if symbol.objfile is not None:
        objsymbol = GetSymbol(symbol.objfile)
        objsymbol.AddDep(symbol.name)
        if not len(objsymbol.indeps):
          if symbol.objfile in object_expn:
            callersym = object_expn[symbol.objfile]
            if callersym.name == symbol.name:
              current_obj = symbol.objfile
              while True:
                parent_obj = object_parent[current_obj]
                if parent_obj in object_expn:
                  callersym = object_expn[parent_obj]
                  break;
                elif parent_obj in object_parent:
                  current_obj = parent_obj;
                  continue;
                else:
                  callersym = mainfile
                  break;
              print >>sys.stderr, "circular objfile ref: ", symbol.name, " object ", symbol.objfile, ", parent obj " , parent_obj  ,"resolved to caller ", callersym.name
            callersym.AddDep(objsymbol.name)
          else:
            print >>sys.stderr, "unreferenced symbol unbound: ", symbol.name, " object ", symbol.objfile
            mainfile.AddDep(objsymbol.name)
      else:
        print >>sys.stderr, "Unknown-objfile and unreferenced symbol ", symbol.name
        mainfile.AddDep(symbol.name)


def FindCycles():
  mainfile = GetSymbol('binary')
  pending_queue = [mainfile]

  done_set = set()
  for symbol in pending_queue:
    TraverseSymbol(dict(), done_set, symbol, 1)
  for symbol in all_symbols.itervalues():
    TraverseSymbol(dict(), done_set, symbol, 1)



def CollectTotalSizes():
  # Start with a list of symbols with no inbound dependencies.
  BindSymbolsToMain()
  FindCycles()
  BindSymbolsToMain()
  FindCycles()
  BindSymbolsToMain()
  FindCycles()

def ProcessDemangledName(name):
  name = name.replace("NMRAnet::", "N:")
  return name

def DemangleAllNames():
  all_names = [k for k in all_symbols.iterkeys()]
  demangled_names = Demangle(all_names)
  for i in range(len(all_names)):
    symbol = all_symbols[all_names[i]]
    demangled_name = demangled_names[i]
    symbol.displayname = ProcessDemangledName(demangled_name)
    enmangle[demangled_name] = all_names[i]

def PrintObjects():
  objtotalsize = dict()
  for symbol in all_symbols.itervalues():
    o = symbol.objfile
    newtotalsize = symbol.codesize
    if o in objtotalsize:
      newtotalsize += objtotalsize[o]
    objtotalsize[o] = newtotalsize
  objlist = [ (v, k) for k, v in objtotalsize.iteritems() ]
  objlist.sort()
  print >>sys.stderr, "Objects:"
  totalsize = 0
  for (v, k) in objlist:
    print >>sys.stderr, "%5d %s" % (v, k)
    totalsize += v
  print >>sys.stderr, "--------------------"
  print >>sys.stderr, "%5d %s" % (totalsize, "Total")
  

def PrintOutput():
  print "digraph g {"

  # first print names
  for name, symbol in all_symbols.iteritems():
    print symbol.PrintNode()
  # then print links
  for name, symbol in all_symbols.iteritems():
    for dname, dep_symbol in symbol.deps.iteritems():
      if dep_symbol.blacklisted: continue
      if dep_symbol.removed_by_filter: continue
      if symbol.removed_by_filter: continue
      print "%s -> %s;" % (escape(symbol.name), escape(dep_symbol.name));
  print "}"
  return

  l = ['main', 'main_thread', 'out_blinker_thread(void*)', 'can_open', 'can_close', 'can_read', 'can_write', 'can_ioctl', 'mbed_can_init(devtab*)', 'mbed_can_tx_msg(devtab*)']
  ls = set(l)
  for i in l:
    if not (i.startswith('__') and (i.endswith('_from_thumb') or i.endswith('_from_arm'))):
      print "%s;" % escape(i)
    for g in cg[i]:
      if g in ls: continue
      ls.add(g)
      l.append(g)

  for i in ls:
    if (i.startswith('__') and (i.endswith('_from_thumb') or i.endswith('_from_arm'))):
      continue
    s = set()
    for j in cg[i]:
      if j not in s:
        s.add(j)
        dest = j
        if dest.startswith('__') and dest.endswith('_from_thumb'):
          dest = dest[2:-11]
        if dest.startswith('__') and dest.endswith('_from_arm'):
          dest = dest[2:-9]
          print "%s -> %s;" % (escape(i), escape(dest));
  print "}"


def usage():
  print "Usage: callgraph.py [-hC] [(-m|--map) mapfile] > callgraph.dot\n"
  print """
Options:
  -h --help: print this message
  -C --[no-]demangle: turns on/off C++ symbol demangling
  -m --map file: reads file (format gnu LD .map) for additional information
"""

def parseargs():
  try:
    opts, args = getopt.getopt(sys.argv[1:], "hm:v", ["help", "map=", "demangle", "min_size="])
  except getopt.GetoptError as err:
    # print help information and exit:
    print str(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)
  output = None
  verbose = False
  for o, a in opts:
    if o == "-v":
      verbose = True
    elif o in ("-h", "--help"):
      usage()
      sys.exit()
    elif o in ("-m", "--map"):
      FLAGS.map_file = a
      print >> sys.stderr, "will read map file ", FLAGS.map_file
    elif o in ("-C", "--demangle"):
      FLAGS.demangle = True
    elif o in ("-t", "--no-strip-template"):
      FLAGS.strip_template = False
    elif o in ("--no-demangle"):
      FLAGS.demangle = False
    elif o in ("--min_size"):
      FLAGS.min_size = int(a)
      print >> sys.stderr, "minimum symbol size ", FLAGS.min_size
    else:
      assert False, "unhandled option"

def ApplyFilters():
  l = []
  for name, symbol in all_symbols.iteritems():
    if symbol.total_code_size < FLAGS.min_size:
      symbol.removed_by_filter = True
    else:
      for dname, dep_symbol in symbol.indeps.iteritems():
        l.append(dep_symbol)
  # We go into all inwards dependencies and make them show
  for symbol in l:
    if symbol.removed_by_filter:
      for dname, dep_symbol in symbol.indeps.iteritems():
        l.append(dep_symbol)
    symbol.removed_by_filter = False


def main():
  parseargs()
  ReadLstFile(sys.stdin)
  if FLAGS.demangle:
    DemangleAllNames()
  print >> sys.stderr, "will read map file ", FLAGS.map_file
  if FLAGS.map_file:
    entries = ReadMapFile(open(FLAGS.map_file, 'r'))
    ProcessMapEntries(entries)
  CollectTotalSizes()
  ApplyFilters()
  PrintObjects()
  PrintOutput()

if __name__ == "__main__":
    main()
