obj-m := mem_dma.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

host:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

hostdebug:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DDEBUG" modules

arm:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="-DARM_BUILD" modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
