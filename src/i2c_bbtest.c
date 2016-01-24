/*
 * Copyright (c) 2010-2016 Douglas Gilbert.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*****************************************************************
 * i2c_bbtest.c
 *
 * I2C "bit-banging" test program. Uses GPIO lines on SAMA5D2 family of
 * SoCs (and others with a similar GPIO structure) to manipulate SCL
 * (clock: out direction) and SDA (data: bidirectional) to act as an
 * I2C protocol master. The SDA line should have a pull-up resistor on
 * it (from 1.8 to 47 kohm). This utility runs the master end of I2C
 * protocol in the user space so it may be scheduled out to allow
 * other processes time to execute.
 * If the '-F' is given then SCL does not need a pull-up, otherwise it does.
 *
 ****************************************************/

#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>


static const char * version_str = "1.01 20130718";

static int force_scl_high = 0;
static int response_len = 0;
static int skip_delay = 0;
static int verbose = 0;

#define I2C_CMD_WRITE 0
#define I2C_CMD_READ 1

/* Example of this utility with a DS1307 RTC
 * > i2c_bbtest -i "68 0" -r 8
 * 27 47 22 04 01 04 09 03
 * Means: (20)090401 22:27:22 Wednesday (4th day of week) */
#define DS1307_I2C_ADDR 0x68

/* Example of this utility with a DS1621 and 1631 thermometer
 * # 1631 first, read the control register:
 * > i2c_bbtest -i "48 ac" -r 1
 * 8c
 * # start convert (control register showed in continuous mode):
 * > i2c_bbtest -i "48 51"
 * # read current temperature:
 * > i2c_bbtest -i "48 aa" -r 2
 * 17 40
 * # 16 bit two's complement with decimal point between bytes -> 23.25C
 *
 * # 1621, read the control register:
 * > i2c_bbtest -i "48 ac" -r 1
 * 8c
 * # start convert (control register showed in continuous mode):
 * > i2c_bbtest -i "48 51"
 * # read current temperature:
 * > i2c_bbtest -i "48 aa" -r 2
 * 17 80
 * # 16 bit two's complement with decimal point between bytes -> 23.5C */
#define DS1621_I2C_ADDR 0x48
#define DS1631_I2C_ADDR 0x48

/* Example of this utility with a BlinkM smart LED
 * > i2c_bbtest -i "9 70 6 0 0"
 * Turns on "cyan flash" pattern indefinitely
 * > i2c_bbtest -i "9 70 0 0 0"
 * Turns on default pattern (white->red->blue->green->off) indefinitely */
#define BLINKM_I2C_ADDR 0x9

/* Example of this utility with a 24LC256 EEPROM (256 Kbit)
 * The byte addressing range is 0 to 0x7fff
 * > i2c_bbtest -i "50 1 23 55"
 * writes 0x55 into address 0x123
 * > i2c_bbtest -i "50 1 23" -r 1
 * should read address 0x123 and hence return 55 (0x55)  */
#define EEPROM_24LC_I2C_ADDR 0x50 /* if A0, A1 and A2 low or floating */


/* SAMA5D2 family of SoCs has four PIOs (banks) each of 32 bits.
 * Notionally the first GPIO pin is PA0 and the last is PD31.
 * Note some GPIO pins are used for other IO so the user should
 * check what is available. */

#define GPIO_PORTA 'A'
#define GPIO_PORTB 'B'
#define GPIO_PORTC 'C'
#define GPIO_PORTD 'D'

/* No defaults */
#define DEF_I2C_SCL_LINE -1
#define DEF_I2C_SCL_PORT '\0'
#define DEF_I2C_SDA_LINE -1
#define DEF_I2C_SDA_PORT '\0'


static int sda_port = DEF_I2C_SDA_PORT;
static int scl_port = DEF_I2C_SCL_PORT;


static int half_delay(void);


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "i2c_bbtest -c <c_bn> [-C <c_port>] -d <d_bn> [-D <d_port>] "
            "[-F]\n"
            "                  [-h] -i <H,H...> [-I] [-r <num>] "
            "[-R <retries>] [-s <sa>]\n"
            "                  [-t] [-v] [-V] [-z]\n"
            "  where:\n"
            "    -c <c_bn>    SCL bit number within c_port. Also accepts\n"
            "                 prefix like 'pb' or just 'b' for <c_port>.\n"
            "    -C <c_port>    SCL port ('A', 'B', 'C' or 'D') or\n"
            "                   gpio kernel pin number\n"
            "    -d <d_bn>    SDA bit number within d_port. Also accepts\n"
            "                 prefix like 'pb' or just 'b' for <d_port>.\n"
            "    -D <d_port>    SDA port ('A', 'B', 'C' or 'D') or\n"
            "                   gpio kernel pin number\n"
            "    -F           force SCL line high (rather than rely on "
            "pull-up)\n"
            "    -h           print usage message\n"
            "    -i <H,H...>    send this string to device where 'H' is an "
            "ASCII hex\n"
            "                   byte. If '-s' not given then the slave "
            "address must\n"
            "                   be lower 7 bits in first byte (top bit "
            "ignored)\n"
            "    -I           ignore NAK and continue\n"
            "    -r <num>     number of bytes to request from slave (def: "
            "0)\n"
            "                 Uses slave address from '-i' or '-s' option\n"
            "    -R <retries>    Number of times to retry NAK on read, "
            "waiting\n"
            "                    <usec> before each retry. If <usec> 0 "
            "waits 1 ms\n"
            "    -s <sa>      slave address in hex. Prepended to string "
            "given to '-i'\n"
            "    -t           ignore other options and cycle SCL 10,000,000 "
            "times\n"
            "                 Should take 100 seconds if i2c clock is 100 "
            "kHz\n"
            "                 [when used twice just do timing loop, no IO]\n"
            "                 [when used thrice do IO but skip delays]\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n"
            "    -w <usec>    wait prior to getting response (def: 0 "
            "microseconds)\n"
            "    -z           attempt to drive both SCL and SDA lines low "
            "(testing)\n\n"
            "I2C bit banging test program. The slave address can be given "
            "either as the\nfirst byte of the '-i' list or "
            "with the '-s' option.\nExample: 24LC256 eeprom with "
            "slave_address=0x50, read byte at 0x123:\n"
            "\t'i2c_bbtest -c PC12 -d PC13 -i \"50 1 23\" -r 1'\n");
}


/* Assume SAMA5D2 ARMv7 CPU running at 496 MHz */
#define DELAY_LOOP_COUNT 10

#define EXPORT_FILE "/sys/class/gpio/export"
#define UNEXPORT_FILE "/sys/class/gpio/unexport"
#define GPIO_BASE_FILE "/sys/class/gpio/gpio"
#define PIO_BASE_FILE "/sys/class/gpio/pio"
#define GPIO_BANK_ORIGIN "/sys/class/gpio/gpiochip0"
/* Earlier kernels (G20+G25) offset GPIO numbers by 32 so
 * /sys/class/gpio/gpiochip32 (a directory) was the lowest numbered bank
 * and corresponded to PIOA (therefore PA0 was gpio number 32). By lk 3.7
 * /sys/class/gpio/gpiochip0 existed and corresponded to PIOA (therefore
 * PA0 becomes gpio number 0).
 * In lk 3.8 the sysfs gpio file names changed from /sys/class/gpio/gpio37
 * to /sys/class/gpio/pioB5 however the export and unexport files still
 * expect GPIO numbers.
 */


static int sda_kpin = -1;
static int scl_kpin = -1;

static int scl_pin_in_bank = DEF_I2C_SCL_LINE;
static int sda_pin_in_bank = DEF_I2C_SDA_LINE;

static int exp_i2c = -1;
static int unexp_i2c = -1;

static int dir_sda = -1;
static int val_sda = -1;

static int dir_scl = -1;
static int val_scl = -1;

static int direction_out = 0;

static int origin0 = 0;


static int
sda_getbit(void)
{
    char b[2];
    int res;

    memset(b, 0,  sizeof(b));
    if ((res = pread(val_sda, b, 1, 0)) <= 0) {
        if (0 == res) {
            if (verbose > 2)
                fprintf(stderr, "%s: pread returns 0\n", __FUNCTION__ );
        } else if (verbose)
            fprintf(stderr, "%s: pread err: %s\n", __FUNCTION__ ,
                    strerror(errno));
        return 1;
    }
    return ('0' != b[0]);
}

static void
set_direction_out(void)
{
    int res;

    res = pwrite(dir_sda, "out", 3, 0);
    if (res < 0)
        perror("set_direction_out: pwrite");
    direction_out = 1;
}

static void
set_direction_in(void)
{
    int res;

    res = pwrite(dir_sda, "in", 2, 0);
    if (res < 0)
        perror("set_direction_in: pwrite");
    direction_out = 0;
}

static void
set_sda(int state)
{
    if (state) {
        if (direction_out)
            set_direction_in();
    } else {
        if (! direction_out)
            set_direction_out();
    }
}

static void
set_scl(int state)
{
    int res = 0;

    if (state) {
        if (force_scl_high)
            res = pwrite(val_scl, "1", 1, 0);
        else
            res = pwrite(dir_scl, "in", 2, 0);
    } else {
        if (force_scl_high)
            res = pwrite(val_scl, "0", 1, 0);
        else
            res = pwrite(dir_scl, "low", 3, 0);
    }
    if (res < 0)
        perror("set_scl: pwrite");
    half_delay();
    half_delay();
}

static void
scl_direction_out(void)
{
    int res = 0;

    res = pwrite(dir_scl, "high", 4, 0);
    if (res < 0)
        perror("scl_direction_out: pwrite");
}

// Return -1 for problems, 0 for okay
static int
init_atmel_pins(void)
{
    struct stat sb;
    char b[256];
    char base_dir[128];

    exp_i2c = open(EXPORT_FILE, O_WRONLY);
    if (exp_i2c < 0) {
        fprintf(stderr, "Open %s: %s\n", EXPORT_FILE, strerror(errno));
        return -1;
    }
    unexp_i2c = open(UNEXPORT_FILE, O_WRONLY);
    if (unexp_i2c < 0) {
        fprintf(stderr, "Open %s: %s\n", UNEXPORT_FILE, strerror(errno));
        return -1;
    }
    snprintf(b, sizeof(b), "%d", scl_kpin);
    if (pwrite(exp_i2c, b, strlen(b), 0) < 0) {
        fprintf(stderr, "Unable to export SCL [pin %d] (already in use?): "
                "%s\n", scl_kpin, strerror(errno));
        return -1;
    }
    /* check if /sys/class/gpio/gpio<knum> directory exists */
    snprintf(base_dir, sizeof(base_dir), "%s%d", GPIO_BASE_FILE, scl_kpin);
    if (stat(base_dir, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so continue in original manner\n",
                    base_dir);
    } else {
        if (verbose > 2)
            fprintf(stderr, "%s not found, now check for pinctrl "
                    "convention\n", base_dir);
        /* check if /sys/class/gpio/pio<bank><bn> directory exists */
        snprintf(base_dir, sizeof(base_dir), "%s%c%d", PIO_BASE_FILE,
                 scl_port, scl_pin_in_bank);
        if (stat(base_dir, &sb) >= 0) {
            if (verbose > 1)
                fprintf(stderr, "%s found so pinctrl convention\n", base_dir);
        } else {
            fprintf(stderr, "Unable to find sysfs directory %s (for "
                    "direction)\n", base_dir);
            return -1;
        }
    }
    snprintf(b, sizeof(b), "%s/direction", base_dir);
    dir_scl = open(b, O_RDWR);
    if (dir_scl < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }
    snprintf(b, sizeof(b), "%s/value", base_dir);
    val_scl = open(b, O_RDWR);
    if (val_scl < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }

    snprintf(b, sizeof(b), "%d", sda_kpin);
    if (pwrite(exp_i2c, b, strlen(b), 0) < 0) {
        fprintf(stderr, "Unable to export SDA [pin %d] (already in use?): "
                "%s\n", sda_kpin, strerror(errno));
        return -1;
    }
    /* check if /sys/class/gpio/gpio<knum> directory exists */
    snprintf(base_dir, sizeof(base_dir), "%s%d", GPIO_BASE_FILE, sda_kpin);
    if (stat(base_dir, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so continue in original manner\n",
                    base_dir);
    } else {
        if (verbose > 2)
            fprintf(stderr, "%s not found, now check for pinctrl "
                    "convention\n", base_dir);
        /* check if /sys/class/gpio/pio<bank><bn> directory exists */
        snprintf(base_dir, sizeof(base_dir), "%s%c%d", PIO_BASE_FILE,
                 sda_port, sda_pin_in_bank);
        if (stat(base_dir, &sb) >= 0) {
            if (verbose > 1)
                fprintf(stderr, "%s found so pinctrl convention\n", base_dir);
        } else {
            fprintf(stderr, "Unable to find sysfs directory %s (for "
                    "direction)\n", base_dir);
            return -1;
        }
    }
    snprintf(b, sizeof(b), "%s/direction", base_dir);
    dir_sda = open(b, O_RDWR);
    if (dir_sda < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }
    snprintf(b, sizeof(b), "%s/value", base_dir);
    val_sda = open(b, O_RDWR);
    if (val_sda < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }
    return 0;
}

static void
cleanup_atmel_pins(void)
{
    char b[256];

    if (val_sda >= 0)
        close(val_sda);
    if (dir_sda >= 0)
        close(dir_sda);
    if (val_scl >= 0)
        close(val_scl);
    if (dir_scl >= 0)
        close(dir_scl);
    if (unexp_i2c >= 0) {
        if (sda_kpin >= 0) {
            snprintf(b, sizeof(b), "%d", sda_kpin);
            if (pwrite(unexp_i2c, b, strlen(b), 0) < 0)
                fprintf(stderr, "Unable to unexport sda: %s\n",
                        strerror(errno));
        }
        if (scl_kpin >= 0) {
            snprintf(b, sizeof(b), "%d", scl_kpin);
            if (pwrite(unexp_i2c, b, strlen(b), 0) < 0)
                fprintf(stderr, "Unable to unexport scl: %s\n",
                        strerror(errno));
        }
        close(unexp_i2c);
    }
    if (exp_i2c >= 0)
        close(exp_i2c);
}

/*
 * Each clock cycle will end up calling half delay four times.
 * Calling an external function that does something like rand()
 * defeats compiler optimization that can skip traditional
 * timing loops.
 * The DELAY_LOOP_COUNT is chosen so the generate SCL will be
 * around 100 kHz (or less) for this SoC. Note that this user
 * space process could be scheduled out to let the SoC do other
 * work. This will elongate SCL cycles but the I2C protocol should
 * be able to handle that.
 */
static int
half_delay(void)
{
    int k, n;

    if (skip_delay)
        return 0;
    for (k = 0, n = 0; k < (DELAY_LOOP_COUNT / 2); ++k) {
        n += rand();
    }
    return n;
}

// Read a byte from I2C bus and send the ack sequence
// Put islast = 1 if this is the last byte to receive from the slave
static unsigned char
i2c_inbyte(int islast)
{
    unsigned char value = 0;
    int bitvalue;
    int k;

    // Read data byte
    set_scl(0);
    set_direction_in();

    for (k = 0; k < 8; ++k) {
        set_scl(1);

        bitvalue = sda_getbit();
        value |= bitvalue;
        if (k < 7)
            value <<= 1;
        set_scl(0);
    }

    if (0 == islast) {
        // Send Ack if is not the last byte to read
        set_direction_out();
        set_sda(0);
        set_scl(1);
        set_scl(0);
        set_direction_in();
    } else {
        // Doesn't send Ack if is the last byte to read
        set_direction_in();
        set_scl(1);
        set_scl(0);
    }
    return value;
}


// Initializate the I2C bus
static void
i2c_init(void)
{
    set_direction_in();
    set_scl(1);
}

// Send a start sequence to I2C bus
static void
i2c_start(void)
{
    set_scl(0);
    set_sda(1);
    set_scl(1);
    set_sda(0);
}

// Send a stop sequence to I2C bus
static void
i2c_stop(void)
{
    set_scl(0);
    set_sda(0);
    set_scl(1);
    set_sda(1);
}

// Send a byte to the I2C bus and return the ack sequence from slave
// rtc
//  0 = NAK, 1 = ACK
static int
i2c_outbyte(unsigned char x)
{
    int k;
    int ack_low;

    set_scl(0);
    for (k = 0; k < 8; ++k) {
        if (x & 0x80)
            set_sda(1);
        else
            set_sda(0);
        set_scl(1);
        set_scl(0);
        x <<= 1;
    }
    set_direction_in();
    set_scl(1);
    ack_low = sda_getbit();
    set_scl(0);
    if (0 == ack_low)
        return 1;
    else
        return 0;
}


/* Read comma (or single space) separated ASCII hex bytes from 'inp' into
 * 'arr'. Number written placed in *arr_len. If first char in 'inp' is "-"
 * reads from stdin instead. With stdin whitespace may be a separator and
 * lines starting with "#" ignored. Returns 0 is successful, else 1 .
 */
static int
read_hex(const char * inp, unsigned char * arr, int max_arr_len,
         int * arr_len)
{
    int in_len, k, j, m, off;
    unsigned int h;
    const char * lcp;
    const char * cp;
    const char * c2p;
    char line[512];

    if ((NULL == inp) || (NULL == arr) || (NULL == arr_len))
        return 1;
    lcp = inp;
    in_len = strlen(inp);
    if (0 == in_len) {
        *arr_len = 0;
    }
    if ('-' == inp[0]) {        /* read from stdin */
        for (j = 0, off = 0; j < 512; ++j) {
            if (NULL == fgets(line, sizeof(line), stdin))
                break;
            in_len = strlen(line);
            if (in_len > 0) {
                if ('\n' == line[in_len - 1]) {
                    --in_len;
                    line[in_len] = '\0';
                }
            }
            if (0 == in_len)
                continue;
            lcp = line;
            m = strspn(lcp, " \t");
            if (m == in_len)
                continue;
            lcp += m;
            in_len -= m;
            if ('#' == *lcp)
                continue;
            k = strspn(lcp, "0123456789aAbBcCdDeEfF ,\t");
            if (in_len != k) {
                fprintf(stderr, "read_hex: syntax error at "
                        "line %d, pos %d\n", j + 1, m + k + 1);
                return 1;
            }
            for (k = 0; k < 1024; ++k) {
                if (1 == sscanf(lcp, "%x", &h)) {
                    if (h > 0xff) {
                        fprintf(stderr, "read_hex: hex number "
                                "larger than 0xff in line %d, pos %d\n",
                                j + 1, (int)(lcp - line + 1));
                        return 1;
                    }
                    if ((off + k) >= max_arr_len) {
                        fprintf(stderr, "read_hex: array length exceeded\n");
                        return 1;
                    }
                    arr[off + k] = h;
                    lcp = strpbrk(lcp, " ,\t");
                    if (NULL == lcp)
                        break;
                    lcp += strspn(lcp, " ,\t");
                    if ('\0' == *lcp)
                        break;
                } else {
                    fprintf(stderr, "read_hex: error in "
                            "line %d, at pos %d\n", j + 1,
                            (int)(lcp - line + 1));
                    return 1;
                }
            }
            off += k + 1;
        }
        *arr_len = off;
    } else {        /* hex string on command line */
        k = strspn(inp, "0123456789aAbBcCdDeEfF, ");
        if (in_len != k) {
            fprintf(stderr, "read_hex: error at pos %d\n",
                    k + 1);
            return 1;
        }
        for (k = 0; k < max_arr_len; ++k) {
            if (1 == sscanf(lcp, "%x", &h)) {
                if (h > 0xff) {
                    fprintf(stderr, "read_hex: hex number larger "
                            "than 0xff at pos %d\n", (int)(lcp - inp + 1));
                    return 1;
                }
                arr[k] = h;
                cp = strchr(lcp, ',');
                c2p = strchr(lcp, ' ');
                if (NULL == cp)
                    cp = c2p;
                if (NULL == cp)
                    break;
                if (c2p && (c2p < cp))
                    cp = c2p;
                lcp = cp + 1;
            } else {
                fprintf(stderr, "read_hex: error at pos %d\n",
                        (int)(lcp - inp + 1));
                return 1;
            }
        }
        *arr_len = k + 1;
         if (k >= max_arr_len) {
            fprintf(stderr, "read_hex: array length exceeded\n");
            return 1;
        }
    }
    return 0;
}


int
main(int argc, char ** argv)
{
    int ch, opt, k, byte0;
    unsigned char command[1024];
    int i2c_slave_addr = -1;
    int scl_timer = 0;
    int ignore_nak = 0;
    int cmd_len = 0;
    int wait_usecs = 0;
    int zero_test = 0;
    int retries = 0;
    int ret = 1;
    const char * cp;
    unsigned char arr[1024];
    struct timespec wait_req;
    struct stat sb;
    int report = 0;

    while ((opt = getopt(argc, argv, "c:C:d:D:Fhi:Ir:R:s:tvVw:z")) != -1) {
        switch (opt) {
        case 'c':
            cp = optarg;
            if (isalpha(cp[0])) {
                if ('P' == toupper(*cp))
                    ++cp;
                ch = toupper(*cp);
                if ((ch >= 'A') && (ch <= 'E'))
                    scl_port = ch;
                else {
                    fprintf(stderr, "'-c' expects a letter ('A' to 'E')\n");
                    exit(EXIT_FAILURE);
                }
                ++cp;
            }
            k = atoi(cp);
            if ((k < 0) || (k > 31)) {
                fprintf(stderr, "'-c' expects a bit number from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            scl_pin_in_bank = k;
            break;
        case 'C':
            if (isalpha(*optarg)) {
                switch (*optarg) {
                case 'A': case 'a' :
                    scl_port = GPIO_PORTA;
                    break;
                case 'B': case 'b' :
                    scl_port = GPIO_PORTB;
                    break;
                case 'C': case 'c' :
                    scl_port = GPIO_PORTC;
                    break;
                case 'D': case 'd' :
                    scl_port = GPIO_PORTD;
                    break;
                default:
                    fprintf(stderr, "'-C' expects a letter ('A' to 'D' or "
                            "'G')\n");
                    exit(EXIT_FAILURE);
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 511)) {
                    fprintf(stderr, "'-C' expects a letter or a number "
                            "0 or greater\n");
                    exit(EXIT_FAILURE);
                }
                scl_kpin = k;
            } else {
                fprintf(stderr, "'-C' expects a letter ('A' to 'D' or 'G') "
                        "or a number\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'd':
            cp = optarg;
            if (isalpha(cp[0])) {
                if ('P' == toupper(*cp))
                    ++cp;
                ch = toupper(*cp);
                if ((ch >= 'A') && (ch <= 'D'))
                    sda_port = ch;
                else {
                    fprintf(stderr, "'-d' expects a letter ('A' to 'D')\n");
                    exit(EXIT_FAILURE);
                }
                ++cp;
            }
            k = atoi(cp);
            if ((k < 0) || (k > 31)) {
                fprintf(stderr, "'-d' expects a bit number from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            sda_pin_in_bank = k;
            break;
        case 'D':
            if (isalpha(*optarg)) {
                switch (*optarg) {
                case 'A': case 'a' :
                    sda_port = GPIO_PORTA;
                    break;
                case 'B': case 'b' :
                    sda_port = GPIO_PORTB;
                    break;
                case 'C': case 'c' :
                    sda_port = GPIO_PORTC;
                    break;
                case 'D': case 'd' :
                    sda_port = GPIO_PORTD;
                    break;
                default:
                    fprintf(stderr, "'-D' expects a letter ('A' to 'D' or "
                            "'G')\n");
                    exit(EXIT_FAILURE);
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 511)) {
                    fprintf(stderr, "'-D' expects a letter or a number "
                            "0 or greater\n");
                    exit(EXIT_FAILURE);
                }
                sda_kpin = k;
            } else {
                fprintf(stderr, "'-D' expects a letter ('A' to 'D' or 'G') "
                        "or a number\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'F':
            ++force_scl_high;
            break;
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        case 'i':
            if (read_hex(optarg, command, sizeof(command), &cmd_len)) {
                fprintf(stderr, "failed reading arguments to '-i'\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'I':
            ++ignore_nak;
            break;
        case 'r':
            k = atoi(optarg);
            if ((k < 0) || (k > 31)) {
                fprintf(stderr, "'-r' expects a length from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            response_len = k;
            break;
        case 'R':
            k = atoi(optarg);
            if ((k < 0) || (k > 1024)) {
                fprintf(stderr, "'-R' expects a length from 0 to 1024\n");
                exit(EXIT_FAILURE);
            }
            retries = k;
            break;
        case 's':
            if ((1 != sscanf(optarg, "%x", &k)) || (k > 127)) {
                fprintf(stderr, "'-s' expects a hex number from 0 to 7f "
                        "(inclusive)\n");
                exit(EXIT_FAILURE);
            }
            i2c_slave_addr = k;
            break;
        case 't':
            ++scl_timer;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
        case 'w':
            k = atoi(optarg);
            if (k < 0) {
                fprintf(stderr, "'-w' expects a non-negative value\n");
                exit(EXIT_FAILURE);
            }
            wait_usecs = k;
            break;
        case 'z':
            ++zero_test;
            break;
        default: /* '?' */
            usage();
            exit(EXIT_FAILURE);
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            usage();
            exit(EXIT_FAILURE);
        }
    }

    if (((scl_kpin < 0) && (scl_pin_in_bank  < 0)) ||
        ((sda_kpin < 0) && (sda_pin_in_bank  < 0))) {
        fprintf(stderr, "Need both GPIOs defined for SCL and SDA\n");
        usage();
        exit(EXIT_FAILURE);
    }
    if ((scl_port < 'A') || (sda_port < 'A')) {
        fprintf(stderr, "SCL or SDA port missing, try something like "
                "'c PC7'\n");
        usage();
        exit(EXIT_FAILURE);
    }

    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so kernel pin numbers start at 0 "
                    "(for PA0)\n", GPIO_BANK_ORIGIN);
        origin0 = 1;
    } else if (verbose > 2)
        fprintf(stderr, "%s not found so kernel pin numbers start at 32 "
                "(for PA0)\n", GPIO_BANK_ORIGIN);
    if (scl_kpin < 0)
        scl_kpin = ((scl_port - 'A' + (! origin0)) * 32) + scl_pin_in_bank;
    else {
        scl_port = (scl_kpin / 32) + (! origin0) + 'A';
        scl_pin_in_bank = (scl_kpin % 32);
        report = 1;
    }
    if (sda_kpin < 0)
        sda_kpin = ((sda_port - 'A' + (! origin0)) * 32) + sda_pin_in_bank;
    else {
        sda_port = (sda_kpin / 32) + (! origin0) + 'A';
        sda_pin_in_bank = (sda_kpin % 32);
        report = 1;
    }
    if (report) {
        printf("Note: SCL is P%c%d and SDA is P%c%d\n", scl_port,
               scl_pin_in_bank, sda_port, sda_pin_in_bank);
        if (verbose)
            printf("  and the corresponding gpio kernel pin numbers: "
                   "SCL=%d, SDA=%d\n", scl_kpin, sda_kpin);
    } else {
        if (verbose)
            fprintf(stderr, "gpio kernel pin numbers: SCL=%d, SDA=%d\n",
                    scl_kpin, sda_kpin);
    }
    if (init_atmel_pins() < 0)
        goto bad;

    scl_direction_out();  /* SCL may not be set for output */

    if (zero_test) {
        fprintf(stderr, "drive SCL and SDA lines low, wait 60 seconds "
                "then exit\n");
        set_scl(0);
        set_direction_out();
        set_sda(0);
        sleep(60);
        goto the_end;
    }
    if (verbose > 2)
        fprintf(stderr, "read_hex read %d bytes from '-i' arguments\n",
                cmd_len);

    if (scl_timer) {
        if (2 == scl_timer) {
            fprintf(stderr, "start SCL timing, without IO\n");
            ch = 0;
            for (k = 0; k < 10000000; ++k) {
                ch += half_delay() + half_delay() + half_delay() +
                      half_delay();
            }
            fprintf(stderr, "cumulative_loop_count=%d\n", ch);
        } else {
            if (3 == scl_timer)
                ++skip_delay;
            fprintf(stderr, "start SCL timing%s\n",
                    (skip_delay ? ", skip delay" : ""));
            for (k = 0; k < 10000000; ++k) {
                set_scl(0);
                set_scl(1);
            }
        }
        fprintf(stderr, "finish SCL timing\n");
        goto the_end;
    }
    if (i2c_slave_addr < 0) {
        if (cmd_len <= 0) {
            fprintf(stderr, "'-i' option required, use '-h' for help\n");
            exit(EXIT_FAILURE);
        }
        i2c_slave_addr = command[0] & 0x7f;
        command[0] <<= 1;    /* since I2C_CMD_WRITE is 0 in bottom bit */
    } else {
        if (cmd_len)
            memmove(command + 1, command, cmd_len);
        ++cmd_len;
        command[0] = (i2c_slave_addr << 1) | I2C_CMD_WRITE;
    }
    if (response_len > (int)sizeof(arr)) {
        fprintf(stderr, "'-r' argument (%d) exceeds allowed size (%d)\n",
                response_len, (int)sizeof(arr));
        exit(EXIT_FAILURE);
    }

    i2c_init();
    i2c_start();

    if (verbose > 2) {
        fprintf(stderr, "About to send these bytes to slave:\n");
        for (k = 0; k < cmd_len; ++k) {
            if ((k > 0) && (0 == (k % 16)))
                fprintf(stderr, "\n");
            fprintf(stderr, " %02x", command[k]);
        }
        fprintf(stderr, "\n");
    }
    for (k = 0; k < cmd_len; ++k) {
        if ((0 == i2c_outbyte(command[k])) && (0 == ignore_nak)) {
            printf("NAK received for pos %d [value=0x%x] from '-i'\n",
                   k + 1, command[k]);
            i2c_stop();
            ret = 1;
            goto bad;
        }
    }
    i2c_stop();

    if (response_len > 0) {
        if (wait_usecs > 0) {
            wait_req.tv_sec = wait_usecs / 1000000;
            wait_req.tv_nsec = (wait_usecs % 1000000) * 1000;
            nanosleep(&wait_req, NULL);
        }
        i2c_start();
        while (1) {
            byte0 = (i2c_slave_addr << 1) | I2C_CMD_READ;
            if (i2c_outbyte(byte0))
                break;
            if ((retries <= 0) || (verbose > 3))
                printf("NACK received for 'read' command [writing "
                       "value=0x%x]\n", byte0);
            if (retries <= 0) {
                i2c_stop();
                ret = 1;
                goto bad;
            } else
                -- retries;
            i2c_stop();
            if (wait_usecs <= 0) {
                wait_req.tv_sec = 0;
                wait_req.tv_nsec = 1000000;     /* 1 ms */
            }
            nanosleep(&wait_req, NULL);
            i2c_start();
        }
        if (verbose > 2)
            fprintf(stderr, "Received these bytes from slave:\n");
        for (k = 0; k < (response_len - 1); ++k)
            arr[k] = i2c_inbyte(0);
        arr[k] = i2c_inbyte(1);
        i2c_stop();

        for (k = 0; k < response_len; ++k) {
            if ((k > 0) && (0 == (k % 16)))
                printf("\n");
            printf(" %02x", arr[k]);
        }
        printf("\n");
    }
the_end:
    ret = 0;

bad:

    cleanup_atmel_pins();
    return ret;
}
