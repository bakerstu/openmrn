

arpreproc: delete.sym
	while read a ; do echo -W $${a} ; done < $< > $@

ldpreproc: delete.sym
	(echo --undef=destructor ; while read a ; do echo --defsym=$${a}=destructor ; done < $< ) > $@

destructors: cg.svg Makefile $(OPENMRNPATH)/etc/stripsym.mk
	grep '~' cg.dot | sed s/^[/][/]// | sort | sed 's/\\n/ /' | while read a b c ; do echo $$a $$c ; done | sed 's/label="//'  | sed 's/"];//' | sed 's/^/0 \0/' > destructors 

LDFLAGSEXTRA += -Wl,@ldpreproc

$(EXECUTABLE)$(EXTENTION): ldpreproc

define TGTLIBSTRIP_template

lib/$(notdir $(1)): $(LIBDIR)/$(notdir $(1)) arpreproc $(LIBDIR)/timestamp
	$(OBJCOPY) @arpreproc $$< $$@

$(EXECUTABLE)$(EXTENTION): lib/$(notdir $(1))

endef

### Argument: "foo" if there is a "lib/libfoo.a" from a subdir called "lib"
define LIBSTRIP_template

lib/lib$(1).stripped: $(1)/lib$(1).a arpreproc | build-$(1)
	rm lib/lib$(1).a
	$(OBJCOPY) @arpreproc $(1)/lib$(1).a lib/lib$(1).a
	touch $$@

$(EXECUTABLE)$(EXTENTION): lib/lib$(1).stripped

endef

### Argument: .o file name in the current dir
define OBJSTRIP_template

$(1).stripped: $(1) arpreproc
	$(OBJCOPY) @arpreproc $$<
	touch $$@

$(EXECUTABLE)$(EXTENTION): $(1).stripped

endef

### Argument: fullpath/obj.o obj.o
define TGTOBJSTRIP_template

lib/$(2): $(1) arpreproc $(LIBDIR)/timestamp
	$(OBJCOPY) @arpreproc $$< $$@

$(EXECUTABLE)$(EXTENTION): lib/$(2)

endef

# Calls the directory template for every target lib file we have.
$(foreach lib,$(wildcard $(LIBDIR)/*.a),$(eval $(call TGTLIBSTRIP_template,$(lib))))

# Calls the object template for every app lib file we have.
$(foreach lib,$(SUBDIRS),$(eval $(call LIBSTRIP_template,$(lib))))

# Calls the object template for every .o file we have.
$(foreach obj,$(OBJS),$(eval $(call OBJSTRIP_template,$(obj))))

# Calls the object template for every .o file we have included from remotely.
$(foreach obj,$(OBJEXTRA),$(eval $(call TGTOBJSTRIP_template,$(obj) $(notdir $(obj)))))

OBJEXTRA:=$(foreach obj,$(OBJEXTRA), $(notdir $(obj)))


clean: clean-stripped

clean-stripped:
	rm -f $(wildcard lib/*.a lib/*.o lib/*.stripped)
