
ifeq ($(wildcard wifi_settings.c),)

OBJS += wifi_settings.o

$(EXECUTABLE)$(EXTENTION): wifi_settings.o

wifi_settings.c:
	cp $(OPENMRNPATH)/etc/wifi_settings.c_example wifi_settings.c
	if ! grep -q wifi_settings.c .gitignore ; then echo "wifi_settings.c*" >> .gitignore ; fi

endif
