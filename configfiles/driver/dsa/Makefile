#
# Makefile for Motorcomm YT921x Switch
#

obj-m += tag_yt921x.o
obj-m += yt921x.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean