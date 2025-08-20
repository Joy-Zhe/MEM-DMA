obj-m := mem_dma.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DHOST_BUILD" modules

host:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DHOST_BUILD" modules

hostdebug:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DDEBUG -DHOST_BUILD" modules

arm:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DARM_BUILD" modules

armdebug:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DARM_BUILD -DDEBUG" modules

armtest:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DARM_BUILD -DBAR4_TEST" modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
