
export KDIR ?= /lib/modules/$(shell uname -r)/build
obj-m		+= core.o
obj-m		+= pci.o
CLANG ?= clang
ifeq ($(origin CC),default)
CC := ${CLANG}
endif

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) CC=$(CC) CONFIG_CC_IS_CLANG=y

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) CC=$(CC) clean
