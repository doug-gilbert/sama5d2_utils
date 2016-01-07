# Place holder until manpages

PREFIX=/usr/local
MANDIR=$(DESTDIR)/$(PREFIX)/share/man
INSTALL=/usr/bin/install
MAN_PREF=man8

MAN_PGS       = a5d3_tc_freq.8 a5d3_pmc.8 gpio_sysfs.8 readbits.8 setbits.8

EXTRA_MAN_PGS	=

all:

clean:

install:
	install -d $(MANDIR)/$(MAN_PREF)
	for mp in $(MAN_PGS); \
		do install -m 644 $$mp $(MANDIR)/$(MAN_PREF); \
		gzip -9f $(MANDIR)/$(MAN_PREF)/$$mp; \
        done

uninstall:
	for mp in $(MAN_PGS); do \
		rm -f $(MANDIR)/$(MAN_PREF)/$$mp.gz; \
	done

