PREFIX=/usr/local
INSTDIR=$(DESTDIR)/$(PREFIX)/bin
MANDIR=$(DESTDIR)/$(PREFIX)/share/man
INSTALL=/usr/bin/install


SUBDIRS = src doc

all: TARGET=all

clean: TARGET=clean

install: TARGET=install

uninstall: TARGET=uninstall
     
.PHONY: recurse $(SUBDIRS)
     
recurse all clean install uninstall: $(SUBDIRS)
     
$(SUBDIRS):
	$(MAKE) -C $@ $(TARGET) PREFIX=$(PREFIX) INSTDIR=$(INSTDIR) INSTALL=$(INSTALL)

     
# Example; say src directory depends on lib directory
# src: lib
