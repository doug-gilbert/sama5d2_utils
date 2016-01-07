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

xxx

    a5d3_pio_set     set PIO attributes on given GPIO line
    a5d3_pio_status  fetch PIO status values for given GPIO line
    a5d3_pmc         monitor the Power Management Controller (PMC)
    a5d3_tc_freq     generate squarish waves using the TC macrocell(s)
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
The SAMA5D3 family is powerful enough to run to do the builds itself.
That is simpler than cross building. The following assumes a root
file system based on Debian 7 or something similar.

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

This package doesn't use libusb but if it did (use version 1.0) then
these packages would be needed:
  # apt-get install libusb-1.0-0
  # apt-get install libusb-1.0-0-dev

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
xxxxxxxxxxxxx


Read the code (and send corrections to me please). There is a usage
information in each utility accessed with the '-h' option. With
some utilities '-hh', '-e' or '-ee' provide extra information (e.g.
g20gpio_status and g25gpio_status).

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

g20gpio_status
==============
The kernel generic interfaces to GPIO lines (e.g. as used by gpio_sysfs)
on the AT91SAM9G20 overlook some of the more subtle features of these
lines. Information about these fine details can be found in the
AT91SAM9G20 preliminary manual in the section titled: "Parallel Input
Output Controller (PIO)". Examples of the details is whether or not an
internal pullup resistor is connected and whether a line is "open drain"
(a.k.a. open collector in the past). [I2C (a.k.a. TWI) requires SDA and
SCL to be "open drain", hence the need to an external pull-up reistor.]

g20gpio_status print out values from the PIO status registers
corresponding to the given line. An output like "psr=0" indicates that
the bit corresponding to the given GPIO line in the PIO_PSR memory-mapped
IO area (PIO_PSR is 32 bits wide) is 0. This means the line is in
peripheral mode and "absr=" should be checked to see if that is peripheral
A (which is absr=0) or peripheral B (which is absr=1).

When GPIO lines don't behave as expected it might be an idea to use
g20gpio_status to determine if there are any abnormal settings.

The g20gpio_status utility has been updated to work both with the
AT91SAM9G20 and the AT91SAM9G25. When the utility is installed a
"hard link" is formed from the name "g25gpio_status" to the g20gpio_status
executable. The utility can detect what name is was invoked by and set the
G20 or G25 mode as appropriate. Command line option '-0' and '-5' can
also be used to force G20 or G25 mode. New features include a status
summary of a whole bank (e.g. '-s -pb' to show all lines in bank B)
and more help and enumerations of each lines peripheral functions (see
'-h', '-hh', '-e' and '-ee').

g20gpio_set
===========
This utility allows most of the attributes shown in g20gpio_status to be
changed (set or cleared). This bypasses the Linux kernel and goes directly
to the PIO unit on the AT91SAM9G20 MCU in the FoxG20. So care should be
taken when changing settings also accessed by the kernel interfaces.

The g20gpio_set utility has been updated to work both with the
AT91SAM9G20 and the AT91SAM9G25. When the utility is installed a
"hard link" is formed from the name "g25gpio_set" to the g20gpio_set
executable. The utility can detect what name is was invoked by and set the
G20 or G25 mode as appropriate. Command line option '-0' and '-5' can
also be used to force G20 or G25 mode. Note that the PIO macrocell in the
G25 has many more features than in the G20 (e.g. optional pull-downs, more
interrupt modes, more peripheral modes (up to 4 for each line), and
debouncing inputs. Debouncing is actually a slow speed glitch filter.


hext2tty
========
The serial tty interface is still quite commonly used both directly
(RS-232 voltage levels or TTL levels (5, 3.3 or 1.8 volts)) or
tunnelled over something like USB so many peripherals still use it.
Being old means that it has very stable (if a bit messy)
kernel/user_space interfaces.

hext2tty supports command/response testing of serial tty based
devices. Commands are given in ASCII hex and any responses are printed
in ASCII hex. It has lots of serial line "knobs" (e.g. for RTS/CTS
hardware handshake).

I2C
===
Wikipedia has an excellent introduction to the I2C protocol. Atmel
call I2C the "Two Wire Interface" (TWI) due to former licensing issues
with the term I2C. Since 1st October 2006 no licensing fee was required
to implement I2C so it has been appearing in many low speed sensors
designed since that date. For example the Sensirion SHT71 temperature
and humidity sensor had its own I2C variant. Recently that company
released the SHT21 which has a real I2C interface.

For Linux documentation on I2C look in the kernel source
Documentation/i2c directory. In there they seem to stress the importance
of SMbus which is an I2C derivative (subset) and big on PC and laptop
motherboards. As far as I can see I2C (or TWI) rules in the embedded
space.

From my investigations the FoxG20 can talk (as "master") to
I2C devices ("slaves") one of two ways:
  a) via the kernel I2C-dev pass-through driver
  b) bit banging GPIO lines directly in the user space

The a) can be further sub-divided:
  a1) using the i2c-gpio bit banging kernel driver
  a2) using the i2c-at91 driver

The FoxG20 distros that I have looked at use a1)  with SDA on PA23 (J6.32)
and SCL on PA24 (J6.31). The Linux kernel source claims that the i2c-at91
is broken but it works with the AT91SAM9G20 (see below). Lack of error
processing can put the silicon in a bad state.

Then there is option b), basically do it yourself (DIY) from the user
space. I2C is a great little protocol in which the clock can
run down to DC (very ... slow), so a user space program getting
scheduled out for 50 milliseconds is not a problem. [When
you try to do the same thing with the one-wire protocol then it
breaks due to its strict timing requirements.]
The DIY advantages are that any spare GPIO lines can be used
and more than one pair can be used [compared with a1) which
is only set up for one pair and a2) that the silicon restricts to
one pair (on the PA23 and PA24 gpio lines)].

The i2c_bbtest utility in this package implements option b) and
pretty well "just works". In a similar fashion to hex2tty, i2c_bbtest
is meant to exercise devices that have a command/response structure.
Notice the '-w <usecs>' option: since the timing of an I2C slave
device's response is controlled by the master end, the slave may not be
finished processing the command it has been given when the master asks
for a response. Clock stretching (see I2C descriptions) is one possible
delaying mechanism by a slave but what I have seen is a NACK on the
attempted read operation to fetch the response. And the slave
continues to NACK until it is ready to respond.

I2C advice
==========
For those who really want to use the kernel I2C possibilities, my advice
is to try and get i2c_bbtest working first. Pick two GPIO lines and
test that they can be changed via the sysfs interface (e.g. with
the gpio_sysfs utility and a multimeter). Don't forget the pull-up
resistors on both SDA and SCL (4.7 kilo-ohm should be fine).
The pull-ups should be to 3.3 volts unless the PC* gpio lines are
being used, in that case the pull-up should be to 1.8 volts. Check the
voltage levels are acceptable to the I2C slave device. For example if
the I2C slave device is 5 volt then its Vih (voltage in, high) is
critical. If it is less that 3.2 or 3.1 volts then a bi-directional
level shifter may be needed.

Linux /dev/i2c-0
================
This is the device node in Linux that is most likely to be accessed
if a user space program is trying to access an I2C device (slave) via
option a1) or a2) described in the "I2C" section above.

The a1) driver stack (within the kernel) is:
        i2c_dev
        i2c_core
        i2c_gpio

[Underscores and hyphens seem to be interchangeable in various tools.]
The i2c_algo_bit module/driver will also be present and can be considered as
a helper. If all of these drivers are build as modules then they should
be visible with the lsmod command.
Using the convention common in other Linux subsystems, the driver at the top
of the stack is closest to the user/kernel interface and the one at the
bottom of the stack is closest to the hardware. The i2c_dev driver can be
thought of as owning the /dev/i2c-0 character device node.

The a2) driver stack is:
        i2c_dev
        i2c_core
        i2c_at91

The i2c_devtest utility in this package essentially "talks to" the i2c_dev
driver via the /dev/i2c-0 character device node. Where possible it has
the same options as the i2c_bbtest utility.

The i2c-at91.c driver
=====================
The interested reader can use google to find out why the driver has been
marked as broken. To make it "unbroken" some kernel source hacking is
required:  visit the <kernel_source>/drivers/i2c/busses
directory and modify Kconfig. I usually take a copy of
a file that I change first:
  cp -p Kconfig Kconfig2632

Then  remove the "&& BROKEN" as shown in this patch:
--- Kconfig2632 2009-12-18 17:27:07.000000000 -0500
+++ Kconfig     2010-01-21 00:28:07.000000000 -0500
@@ -279,7 +279,7 @@

 config I2C_AT91
        tristate "Atmel AT91 I2C Two-Wire interface (TWI)"
-       depends on ARCH_AT91 && EXPERIMENTAL && BROKEN
+       depends on ARCH_AT91 && EXPERIMENTAL
        help
          This supports the use of the I2C interface on Atmel AT91
          processors.

Then run 'make menuconfig' in the top level linux source directory
and select device_drivers|I2C_support|hardware_bus_support
and the Atmel AT91 entry should be there. Select it as a module
and deselect the GPIO based bitbanging I2C a few entries down.

The i2c-at91 driver (found in drivers/i2c/busses/i2c-at91.c) has
problems with AT91SAM9G20. Hard to know whether those who marked
it is broken in the Linux kernel looked at recent members of the
AT91 family. Anyway the driver in lk 2.6.32 is not a great example
of the art as it has no error checking. I made some changes and they
can be found here as a patch against lk 2.6.32 :
http://sg.danny.cz/foxg20/i2c-at91_2632dpg1.patch
Since the patch changes only one file (i2c-at91.c) here is that file:
http://sg.danny.cz/foxg20/i2c-at91.c

The clock rate of the SCL line is adjustable via the 'clockrate'
module parameter. Due to limitations of the clock divider, frequencies
from 4000 to 400000 Hz are accepted. The clock rate can be changed
on the fly with a command like:
   echo 25000 > /sys/module/i2c_at91/parameters/clockrate
[sysfs does have some uses.] Note that I2C NACKs appear as
errno=EREMOTEIO to ioctl(I2C_RDWR).

Linux kernel 3.7.0 and new i2c-at91 driver
==========================================
In the lk 3.7 series a new i2c-at91 driver by Nikolaus Voss was
introduced into the mainline kernel. This new driver (it was
proposed over a year ago) should be used in preference to the
original "broken" driver together with the author's patches to
improve it. The old driver and the author's patches to it are
discussed in the previous section.

The new driver does not have any clock speed selection (at the
moment). The clock frequency is fixed at 100 kHz which all I2C
(TWI) devices should accept.

See the <kernel_src>/Documentation/i2c directory for Linux specific
documentation. If the kernel is built with CONFIG_I2C_CHARDEV
(and CONFIG_I2C_AT91 to get this driver) then device nodes like
/dev/i2c-0 and /dev/i2c-1 should appear. Then user space programs
can use code shown in the "dev-interface" file in the
Documentation/i2c directory to control I2C devices.


devmem2 and mem2io
==================
The devmem2 program allows memory locations to be read and written.
The helps access the memory-mapped IO locations in the G20 and G25
MCUs. It was written by Jan-Derk Bakker <jdb@lartmaker.nl> in 2000.
A copy of that source is in this directory. The author has added
a similar program called mem2io which a few extra features like
the ability to read from a script file and change multiple locations
in one invocation.

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
6th January 2016
