-include ../openmrnpath.mk
ifndef OPENMRNPATH
OPENMRNPATH := $(realpath ..)
endif
export OPENMRNPATH


SUBDIRS = \
	async_blink \
	blink_raw \
	bootloader \
	bootloader_client \
	reflash_bootloader \
	clinic_app \
	hub \
	io_board \
	js_hub \
	js_client \
	js_cdi_server \
	send_datagram \
	simple_client \
	tractionproxy \
	train \
	usb_can \


include $(OPENMRNPATH)/etc/recurse.mk
