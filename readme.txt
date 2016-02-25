Introduction
============
This is a small group of Linux utilities for the Atmel SAMA5D2 family of
SoCs (System on a Chip). The SAMA5D2 family could be considered as an
enhancement of the SAMA5D3 and SAMA5D4 families and share many macrocells.
A macrocell is the term used for a named function, or instance of a
function, within the SoC. GPIO lines are grouped in units of 32 and
controlled by macrocells named, PIOA, PIOB, PIOC, and PIOD. Depending on
the context, the more general name "PIO" may be considered as a macrocell.
And that more general sense is how Atmel document the SAMA5D2, with a
section (or chapter) for each in their 2642 page "datasheet".

Some of these utilities work with the Linux kernel while others bypass
the kernel and directly manipulate the macrocell registers which are
memory mapped. Mainline "DT" (device tree) kernels since lk 4.2 have
support for the SAMA5D2 family.

Summary
=======
The following executables are built, their source code can be found in
the src/ directory. Some of these utilities have man pages and they are
found in the doc/ directory.

    a5d2_pio_set     set PIO attributes on given GPIO line
    a5d2_pio_status  fetch PIO status values for given GPIO line
    a5d2_pmc         monitor the Power Management Controller (PMC)
    a5d2_tc_freq     generate squarish waves using the TC macrocell(s)
    devmem2          read or write to memory address (useful for
                     controlling memory mapped IO) N.B. no longer
                     installed since clashes with Debian/Ubuntu
                     package of same name; can be made be hand
    gpio_sysfs       access and test a GPIO line using sysfs interface
    hex2tty          command/response tester for serial lines
    i2c_bbtest       I2C master using user space GPIO bit banging
    i2c_devtest      I2C master using kernel /dev/i2c-<num> device
    is_ariag25       script helper, exit status 0 for at91sam9g25
    is_arm           script helper, exit status 0 for EABI (ARM)
    is_foxg20        script helper, exit status 0 for at91sam9g20
    is_foxglx        script helper, exit status 0 for Axis 384LX
    is_sama5d2       script helper, exit status 0 for sama5d2 family
    is_sama5d3       script helper, exit status 0 for sama5d3 family
    is_sama5d4       script helper, exit status 0 for sama5d4 family
    mem2io           read or write multiple 32 bit values from memory.
    readbits         sysfs based GPIO line reader
    setbits          sysfs based GPIO line writer
    xbee_api         specialization of hex2tty for XBee API

Build system
============
The SAMA5D2 family is powerful enough to run to do the builds itself.
That is simpler than cross building. The following assumes a root
file system based on Debian 7, 8 or something similar.

Connect the system to the internet via one of the ethernet macrocells
(or perhaps a WiFi USB dongle) and make sure the root file system is
up to date:
  # apt-get update
  # apt-get dist-upgrade

To compile and build utilities load the (pseudo) package:
  # apt-get install build-essential

To run the "./build_debian" script to generate a deb binary package
that can be installed on other ARM systems; this pakage is required:
  # apt-get install debhelper

Other "-dev" packages may be needed for some peripherals.

In the top level directory of this package is a script called
build_debian.sh . It will build a Debian binary package and place it in
the directory above the current. It can be invoked like this:
  # ./build_debian.sh

RIf that is not suitable, then this should work:
  # make ; make install

You may need to check where the 'make install' places the executables and
man pages.


Documentation
=============

Read the code (and send corrections to me please). There is usage
information in each utility accessed with the '-h' option. With
some utilities '-hh', '-e' or '-ee' provide extra information (e.g.
a5d2_pio_status and a5d2_pio_set).

gpio_sysfs and gpio_ioctl
=========================
These two utilities have almost identical functionality. They differ in
the way the access the gpio lines at the kernel interface. gpio_sysfs
uses the sysfs gpio interface documented in the
<kernel_source>/Documentation/gpio.txt
file in a section titled "Sysfs Interface for Userspace". gpio_ioctl
uses ioctls on the /dev/gpio device node and is discussed in the next
section.

Both utilities can toggle the specified gpio line <num> times with a
<usec> delay between each transition (see the '-t', '-n' and '-d' options).
The high state may be as a result of an external or internal pull-up
resistor (default action) or may be driven high when the '-f' option
is used. The accuracy of the <usec> delay depends on several factors
(e.g. whether a tickless kernel is used) but it most certainly will
not be accurate down to the microsecond level.

The gpio_sysfs has an extra '-c' option to count rising, falling or
both edges on a gpio line for a given duration (default 1 second).
See the gpio.txt kernel documentation file, specifically about the
"edge" file. The use of poll() from the user space to detect a level
change is quite bizarre and curious readers are encouraged to read the
code for the process_count() function in gpio_sysfs.c .

Recent "device-tree" Linux kernels (e.g. lk 3.7) have changed the
GPIO kernel pin number ordering. In the past it started at 32 (for
PA0) to allow other interrupt number to use the numbers 0 to 31.
Those two numbering systems have been decoupled so GPIO kernel pin
numbers now start at 0 (for PA0). This can be detected at run time
by the presence or absence of this directory: /sys/class/gpio/gpiochip0 .
If it is present PA0 is GPIO kernel pin number 0.

devmem2 and mem2io
==================
The devmem2 program allows memory locations to be read and written.
The helps access the memory-mapped IO locations used by the Atmel
AT91 series of SoCs. It was written by Jan-Derk Bakker
<jdb@lartmaker.nl> in 2000. A copy of that source is in this directory
The author has added a similar program called mem2io which a few extra
features like the ability to read from a script file and change multiple
locations in one invocation.

Both devmem2 and mem2io were built and installed in earlier versions
of this package. However recently a Ubuntu server based system for
ARM was found to have the devmem2 program installed from its own
package. [Debian 6 does not have a devmem2 package in its standard
repositories and devmem2 is flagged as a "requested package" so
it may appear in the future.] So to avoid clashes in the future,
the devmem2 program is no longer built or installed. It can still
be made "by hand" with 'make devmem2'. The mem2io program can be used
as a replacement but it has a slightly different command line syntax.


Douglas Gilbert

dgilbert@interlog.com
21st February 2016
