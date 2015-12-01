KERNEL_VERSION	:= `uname -r`

obj-m := snd-usb-exbox.o

DEBUG_CFLAGS=-g -Wall -DDEBUG=1

snd-usb-exbox-objs := chip.o pcm.o midi.o

KDIR   := /lib/modules/$(KERNEL_VERSION)/build
PWD    := $(shell pwd)
MODDIR := $(DESTDIR)/lib/modules/$(KERNEL_VERSION)/kernel/sound/usb/
BINDIR := $(DESTDIR)/usr/local/bin
INCDIR := $(DESTDIR)/usr/include/alsa/sound


default::
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) EXTRA_CFLAGS="${DEBUG_CFLAGS} ${BROKEN}" modules

install-only:: default
	mkdir -p $(MODDIR) $(BINDIR)
	cp snd-usb-exbox.ko $(MODDIR)

install:: install-only
	/sbin/depmod -a
	sync
	/sbin/rmmod snd-usb-exbox.ko || true
	/sbin/modprobe snd-usb-exbox
	sync
	sync

clean::
	rm -f core .*.cmd *.o *.ko *.mod.c Module.* modules.order *.bak .\#* *~
	rm -rf .tmp_versions

