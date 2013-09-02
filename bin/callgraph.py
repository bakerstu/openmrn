#!/usr/bin/python
# powerpc-gekko-objdump -d example.elf | python cg.py | dot -Tpng > cg.png && eog cg.png

import re, sys, math, subprocess, getopt

f_re = re.compile('(^[0-9a-f]*) <([a-zA-Z0-9_:()*, ]*)>:$')
c_re = re.compile('\t(b[xl]?|jal)[ \t]*[0-9a-f]* <([a-zA-Z0-9_:()*, ]*)>')
a_re = re.compile('\t.word[ \t]*0x([0-9a-f]*)$')
pa_re = re.compile('(^[0-9a-f]*):\t')


_blacklist = set(['sys_init', 'SYS_ResetSystem', 'puts', 'getButtons',
    'malloc', 'free', 'memalign', 'fflush', 'sd_mkdir', 'check_fatpath',
    'memset', 'memcpy', 'DCFlushRange', 'iosAlloc', 'iosFree',
    'DCInvalidateRange'])
_blacklist_pre = ['__', 'str', 'f_', 'VIDEO_']
_blacklist_cont = ['printf']

_blacklist = set(['diewith', '__diewith_from_arm', 'memcpy', 'os_mutex_lock',
                  'os_mutex_unlock', 'malloc', 'free', 'abort',
                  'vPortEnterCritical', 'vPortExitCritical',
                  'xTaskResumeAll', 'vTaskSuspendAll',
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
FLAGS.demangle = True


def Blacklist(s):
    if s in _blacklist: return True
    for f in _blacklist_cont:
        if f in s: return True
    for f in _blacklist_pre:
        if s.startswith(f): return True
    return False


all_symbols = dict()
enmangle = dict()

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
               )

  def __init__(self, name):
    self.deps = dict()
    self.indeps = dict()
    self.cycle = None
    self.name = name
    self.displayname = name
    self.codesize = 0
    self.blacklisted = Blacklist(name)

  def AddDep(self, dep):
    other_symbol = GetSymbol(dep)
    self.deps[dep] = other_symbol
    other_symbol.indeps[self.name] = self

  def PrintNode(self):
    cyclestring = ""
    try:
      if self.has_cycle: cyclestring = "c"
    except AttributeError, err:
      print >>sys.stderr,"Not processed? ", self.name
      cyclestring = "N"
      self.total_code_size = self.codesize
    return ("%s [shape=box, label=\"%s\\n%d / %d %s\"];" %
            (self.name, self.displayname, self.codesize, self.total_code_size,
             cyclestring))

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
    return demangled[:-1]

def ReadLstFile(f):
  current_symbol = None
  last_offset = 0;
  address_lookup = dict()
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

  section_re = re.compile('^ (?:[.](?P<section>[-a-zA-Z_]+)(?:[.](?P<subsection>[^ \t\n]+))?)?(?:[ \t]+0x(?P<address>[0-9a-fA-F]+)[ \t]+(?:0x(?P<length>[0-9a-fA-F]+) (?:(?P<library>.*[.]a)[\(])?(?P<object>.*[.]o)[\)]?|(?P<function>[^0].*)))?$');
  count = 0;
  entries = []
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
    m = section_re.match(line)
    if m:
      count = count + 1
      if False and FLAGS.verbose and not printed:
        print >>sys.stderr, line
        printed = True
        print >>sys.stderr, "matched textsection ", m.groups() 
      if m.group('section'):
        section = m.group('section')
        subsection = None
      if m.group('subsection'):
        subsection = m.group('subsection')
      if m.group('object'):
        objfile = m.group('object')
        library = m.group('library')
      if m.group('length') is not None and m.group('function') is not None:
        print >>sys.stderr, line
        print >>sys.stderr, "length %s and function %s " % (m.group('length'), m.group('function')), m.groups() 
      if m.group('function') is not None:
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
  print >>sys.stderr, "found %d lines, %d entries." % (count, len(entries))
  return entries

def escape(s):
  return re.sub('[*()-: ,]', '_', s)


def ProcessMapEntries(entries):
  enmangle_found_count = 0;
  for e in entries:
    if e.address == 0:
      continue;
    if (0xbd000000 <= e.address and
        e.address < 0xbe000000):
      # Interesting function
      if e.function in all_symbols:
        symbol = all_symbols[e.function]
      elif e.function in enmangle and enmangle[e.function] in all_symbols:
        symbol = all_symbols[enmangle[e.function]]
        enmangle_found_count = enmangle_found_count + 1
        #print >>sys.stderr, "found enmanged symbol for %s: %s"%(e.function, symbol.name)
      elif e.subsection in all_symbols and e.address == all_symbols[e.subsection].address:
        #print >>sys.stderr, "subsection symbol found: ", e, all_symbols[e.subsection].address
        symbol = all_symbols[e.subsection]
      else:
        print >>sys.stderr, "symbol not found: ", e
        continue
      symbol.objfile = escape(e.objfile)

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

def CollectTotalSizes():
  # Start with a list of symbols with no inbound dependencies.
  mainfile = GetSymbol('binary')
  v = [v for v in all_symbols.itervalues()]
  for symbol in v:
    if not len(symbol.indeps):
      if hasattr(symbol, 'objfile'):
        objsymbol = GetSymbol(symbol.objfile)
        objsymbol.AddDep(symbol.name)
        mainfile.AddDep(objsymbol.name)
      else:
        print >>sys.stderr, "Unknown-objfile and unreferenced symbol ", symbol.name
        mainfile.AddDep(symbol.name)
  pending_queue = [mainfile]

  done_set = set()
  for symbol in pending_queue:
    TraverseSymbol(dict(), done_set, symbol, 1)
  for symbol in all_symbols.itervalues():
    TraverseSymbol(dict(), done_set, symbol, 1)

def DemangleAllNames():
  all_names = [k for k in all_symbols.iterkeys()]
  demangled_names = Demangle(all_names)
  for i in range(len(all_names)):
    symbol = all_symbols[all_names[i]]
    demangled_name = demangled_names[i]
    symbol.displayname = demangled_name
    enmangle[demangled_name] = all_names[i]
    

def PrintOutput():
  print "digraph g {"

  # first print names
  for name, symbol in all_symbols.iteritems():
    print symbol.PrintNode()
  # then print links
  for name, symbol in all_symbols.iteritems():
    for dname, dep_symbol in symbol.deps.iteritems():
      if (dep_symbol.blacklisted): continue
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
    opts, args = getopt.getopt(sys.argv[1:], "hm:v", ["help", "map=", "demangle"])
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
    elif o in ("--no-demangle"):
      FLAGS.demangle = False
    else:
      assert False, "unhandled option"


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
  PrintOutput()

if __name__ == "__main__":
    main()


