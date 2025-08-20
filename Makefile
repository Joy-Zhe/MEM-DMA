SUBDIRS := mem_dma_driver

.PHONY: all clean host hostdebug arm armdebug armtest

all:
	$(MAKE) -C $(SUBDIRS) all

host:
	$(MAKE) -C $(SUBDIRS) host

hostdebug:
	$(MAKE) -C $(SUBDIRS) hostdebug

arm:
	$(MAKE) -C $(SUBDIRS) arm

armdebug:
	$(MAKE) -C $(SUBDIRS) armdebug

armtest:
	$(MAKE) -C $(SUBDIRS) armtest

clean:
	$(MAKE) -C $(SUBDIRS) clean
