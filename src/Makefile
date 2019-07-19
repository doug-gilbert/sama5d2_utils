
PREFIX=/usr/local
INSTDIR=$(DESTDIR)/$(PREFIX)/bin
INSTALL=/usr/bin/install

## CC = gcc
## CC = gcc-8
## CC=arm-linux-gnueabi-gcc 
# for C++/clang testing
## CC = g++ -std=c++11
## CC = g++
## CC = clang
## CC = clang++


PROGS     = setbits readbits is_foxlx is_foxg20 hex2tty mem2io \
	    is_ariag25 is_arm is_sama5d3 a5d2_pmc a5d2_pio_status \
	    a5d2_pio_set gpio_sysfs a5d2_tc_freq i2c_bbtest \
	    i2c_devtest xbee_api w1_temp is_sama5d2 is_sama5d4

SCRIPTS =


SUBDIRS =


# -std=<s> can be c99, c11, gnu11, etc. Default is gnu89 (gnu90 is the same)
CFLAGS = -g -O2 -Wall -W
# CFLAGS = -g -O2 -Wall -W -std=c11
# CFLAGS = -g -O2 -Wall -W -D_REENTRANT
#LDFLAGS = -L/usr/arm-linux-gnueabi/lib,-rpath-link=/usr/arm-linux-gnueabi/lib


all: $(PROGS) subdirs

gpio_sysfs: gpio_sysfs.o
	$(CC) $(LDFLAGS) $^ -lrt $(LDLIBS) -o $@ 
# librt depends on libpthread but can't find it in Ubuntu 10.10
# 	$(CC) $(LDFLAGS) $^ -lpthread -lrt $(LDLIBS) -o $@

## gpio_ioctl: gpio_ioctl.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

setbits: setbits.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

readbits: readbits.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_foxlx: is_foxlx.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_foxg20: is_foxg20.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

## w1_bbtest: w1_bbtest.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

hex2tty: hex2tty.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

## i2c_bbtest: i2c_bbtest.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

i2c_devtest: i2c_devtest.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

devmem2: devmem2.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

mem2io: mem2io.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

## g20tc_freq: g20tc_freq.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

## g20gpio_status: g20gpio_status.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

## g20gpio_set: g20gpio_set.o
## 	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_arm: is_arm.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_ariag25: is_ariag25.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_sama5d3: is_sama5d3.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_sama5d4: is_sama5d4.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

is_sama5d2: is_sama5d2.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

a5d2_pmc: a5d2_pmc.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

a5d2_pio_status: a5d2_pio_status.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

a5d2_pio_set: a5d2_pio_set.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

a5d2_tc_freq: a5d2_tc_freq.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

i2c_bbtest: i2c_bbtest.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

xbee_api: xbee_api.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

w1_temp: w1_temp.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@


subdirs:
	for i in $(SUBDIRS); do $(MAKE) -C $$i ; done


install: $(PROGS)
	$(INSTALL) -d $(INSTDIR)
	$(INSTALL) -p -m 0777 $(PROGS) $(INSTDIR)
	if [ $(SCRIPTS) ] ; then $(INSTALL) -p -m 0777 $(SCRIPTS) $(INSTDIR) ; fi
	for i in $(SUBDIRS); do $(MAKE) -C $$i install ; done

uninstall:
	dists="$(PROGS)"; \
	for name in $$dists; do \
		rm -f $(INSTDIR)/$$name; \
	done

clean:
	rm -f $(PROGS) *.o core
	for i in $(SUBDIRS); do $(MAKE) -C $$i clean ; done
