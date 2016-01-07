/*
 * Copyright (c) 2010-2013 Douglas Gilbert.
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
 * i2c_devtest.c
 *
 * I2C test program that uses the linux /dev/i2c-<num> interface.
 * The SDA and the SCL lines probably need pull-up resistors on them
 * (from 1.8 to 47 kohm, 4K7 works). This utility runs the master
 * end of I2C protocol via a pass-through to the i2c-dev driver.
 * Apart from the i2c_core driver there will be another driver
 * associated with the hardware being used: that might be either
 * i2c-gpio (implying a kernel driver "bit banging") gpio lines
 * or say, the i2c-at91 driver (for silicon in the AT91 series chips).
 *
 * In the SAMA5D3 family /dev/i2c-0 <-> TWD0(PA30),TWCK0(PA31) and
 * /dev/i2c-1 <-> TWD1(PC26),TWCK1(PC27)
 *
 ****************************************************/

#define _POSIX_C_SOURCE 200112L

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>

/* Debian 6.0 has these headers in the linux-libc-dev package. */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static const char * version_str = "2.01 20130724";

#ifndef I2C_FUNC_NOSTART
#define I2C_FUNC_NOSTART 0x00000010
#endif


/* Example of this utility with a DS1307 RTC
 * # Read the stored date time stamp
 * > i2c_devtest -d0 -i "68 0" -r 8
 * 27 47 22 05 02 04 09 03
 * # Means: (20)090402 22:47:27 Thursday (5th day of week)
 * to set that datetime:
 * > i2c_devtest -d0 -i "68 0 27 47 22 05 02 04 09"
 */
#define DS1307_I2C_ADDR 0x68

/* Example of this utility with a DS1621 and 1631 thermometer
 * # 1631 first, read the control register:
 * > i2c_devtest -d0 -i "48 ac" -r 1
 * 8c
 * # start convert (control register showed in continuous mode):
 * > i2c_devtest -d0 -i "48 51"
 * # read current temperature:
 * > i2c_devtest -d0 -i "48 aa" -r 2
 * 17 40
 * # 16 bit two's complement with decimal point between bytes -> 23.25C
 *
 * # 1621, read the control register:
 * > i2c_devtest -d0 -i "48 ac" -r 1
 * 8c
 * # start convert (control register showed in continuous mode):
 * # note: differs from DS1631 which uses 0x51 for start convert
 * > i2c_devtest -d0 -i "48 ee"
 * # read current temperature:
 * > i2c_devtest -d0 -i "48 aa" -r 2
 * 17 80
 * # 16 bit two's complement with decimal point between bytes -> 23.5C */
#define DS1621_I2C_ADDR 0x48
#define DS1631_I2C_ADDR 0x48

/* Example of this utility with a BlinkM smart LED
 * > i2c_devtest -d0 -i "09 70 6 0 0"
 * Turns on "cyan flash" pattern indefinitely
 * > i2c_devtest -d0 -i "09 70 0 0 0"
 * Turns on default pattern (white->red->blue->green->off) indefinitely */
#define BLINKM_I2C_ADDR 0x9

/* Example of this utility with a 24LC256 EEPROM (256 Kbit)
 * The byte addressing range is 0 to 0x7fff
 * > i2c_devtest -d0 -i "50 1 23 55"
 * writes 0x55 into address 0x123
 * > i2c_devtest -d0 -i "50 1 23" -r 1
 * should read address 0x123 and hence return 55 (0x55)  */
#define EEPROM_24LC_I2C_ADDR 0x50 /* with A0, A1 and A2 low or floating */



static void
usage(int help_val)
{
    if (help_val > 1) {
        fprintf(stderr, "Some examples using i2c_devtest:\n");
        fprintf(stderr, "# Example of this utility with a DS1307 RTC to\n");
        fprintf(stderr, "# read the stored date time stamp:\n");
        fprintf(stderr, " > i2c_devtest -d0 -i \"68 0\" -r 8\n");
        fprintf(stderr, " 27 47 22 05 02 04 09 03\n");
        fprintf(stderr, "# Means: (20)090402 22:47:27 Thursday (5th day of "
                "week)\n");
        fprintf(stderr, "# Invocation also could have been:\n");
        fprintf(stderr, " > i2c_devtest -d0 -s 68 -i \"0\" -r 8\n\n");
        fprintf(stderr, "# Example of this utility with a 24LC256 EEPROM "
                "(256 Kbit)\n");
        fprintf(stderr, "# The byte addressing range is 0 to 0x7fff\n");
        fprintf(stderr, " > i2c_devtest -d0 -s 50 -i \"1 23 55\"\n");
        fprintf(stderr, "# writes 0x55 into address 0x123\n");
        fprintf(stderr, " > i2c_devtest -d0 -s 50 -i \"1 23\" -r 1\n");
        fprintf(stderr, " 55\n");
        fprintf(stderr, "# reads 55 (0x55) from address 0x123\n");
        return;
    }
    fprintf(stderr, "Usage: "
            "i2c_devtest [-d <dev>] [-F] [-h] [-H] -i <H,H...> [-I] "
            "[-r <num>]\n"
            "                   [-s <sa>] [-t] [-T] [-v] [-V] [-w <usec>]\n"
            "  where:\n"
            "    -d <dev>     if <dev> starts with digit then open device\n"
            "                 '/dev/i2c-<num>' else open device '<dev>'\n"
            "                 (default: '/dev/i2c-0')\n"
            "    -F           print functionality of I2C master; use twice "
            "to\n"
            "                 additionally show (indented) what is not "
            "available\n"
            "    -h           print usage message; use twice for examples\n"
            "    -H           print functionality as a hex number\n"
            "    -i <H,H...>    send this string to device where 'H' is an "
            "ASCII hex\n"
            "                   byte. If '-s' not given then the slave "
            "address must\n"
            "                   be lower 7 bits in first byte (top bit "
            "ignored)\n"
            "    -I           ignore NAKs (twice: ignore NAKs on write "
            "transfer)\n"
            "    -r <num>     number of bytes to request from slave (def: "
            "0)\n"
            "                 Uses slave address from '-i' or '-s' option\n"
            "    -R <times>   repetition: number of times to send <H,H...> "
            "string\n"
            "                 def: 1; max: 10 times. For testing repeated "
            "start\n"
            "    -s <sa>      slave address in hex\n"
            "    -t           test: write 0x55 byte 1024 times; use twice "
            "to write\n"
            "                 0xaa byte instead\n"
            "    -T           ten bit slave address, must also use "
            "'-s <sa>'\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n"
            "    -w <usec>    wait prior to getting response (def: 0 "
            "microseconds)\n\n"
            "I2C device test program. The (7 bit) slave address can be "
            "given either\nas the first byte of the '-i' list or "
            "with the '-s' option.\nExample: DS1307 slave_address=68h, so "
            "either -i '68,...' or '-s 68'\n");
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

/* perhaps the kernel doesn't like writing back to the stack */
static unsigned char command[1024];
static unsigned char arr[1024 + 4];

int
main(int argc, char ** argv)
{
    int opt, k, fd;
    char dev_name[256];
    int i2c_slave_addr = -1;
    int i2c_response_len = 0;
    int functionality = 0;
    int do_help = 0;
    int do_hex = 0;
    int ignore_nak = 0;
    int test = 0;
    int times = 1;
    int ten_bit_sa = 0;
    int cmd_len = 0;
    int verbose = 0;
    int wait_usecs = 0;
    unsigned long funcs;
    struct i2c_msg msg[11];
    struct i2c_rdwr_ioctl_data rdwr_arg;
    struct timespec wait_req, rem;

    memset(dev_name, 0, sizeof(dev_name));
    memset(msg, 0, sizeof(msg));
    while ((opt = getopt(argc, argv, "d:FhHi:Ir:R:s:tTvVw:")) != -1) {
        switch (opt) {
        case 'd':
            if (isdigit(*optarg))
                snprintf(dev_name, sizeof(dev_name), "/dev/i2c-%s", optarg);
            else
                strncpy(dev_name, optarg, sizeof(dev_name) - 1);
            break;
        case 'F':
            ++functionality;
            break;
        case 'h':
            ++do_help;
            break;
        case 'H':
            ++do_hex;
            break;
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
            if ((k < 0) || (k > 1024)) {
                fprintf(stderr, "'-r' expects a length from 0 to 1024\n");
                exit(EXIT_FAILURE);
            }
            i2c_response_len = k;
            break;
        case 'R':
            k = atoi(optarg);
            if ((k < 1) || (k > 10)) {
                fprintf(stderr, "'-R' expects an argument from 1 to 10\n");
                exit(EXIT_FAILURE);
            }
            times = k;
            break;
        case 's':
            if ((1 != sscanf(optarg, "%x", &k)) || (k > 1023)) {
                fprintf(stderr, "'-s' expects a hex number from 0 to 3ff "
                        "(inclusive)\n");
                exit(EXIT_FAILURE);
            }
            i2c_slave_addr = k;
            break;
        case 't':
            ++test;
            break;
        case 'T':
            ++ten_bit_sa;
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
        default: /* '?' */
            usage(1);
            exit(EXIT_FAILURE);
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            usage(1);
            exit(EXIT_FAILURE);
        }
    }

    if (do_help) {
        usage(do_help);
        exit(EXIT_SUCCESS);
    }

    if (0 == dev_name[0])
        strcpy(dev_name, "/dev/i2c-0");

    if (functionality) {
        if ((fd = open(dev_name, O_RDWR)) < 0) {
            perror("open failed");
            fprintf(stderr, "Tried to open %s; may need to load modules "
                    "i2c_dev and/or i2c_gpio\n", dev_name);
            exit(EXIT_FAILURE);
        }
        if (ioctl(fd, I2C_FUNCS, &funcs) < 0) {
            perror("ioctl(I2C_FUNCS) failed");
            exit(EXIT_FAILURE);
        }
        if (do_hex)
            printf("functionality=0x%lx\n", funcs);
        else {
            printf("Functionality of master:\n");
            if (I2C_FUNC_I2C & funcs)
                printf("  I2C_FUNC_I2C set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_I2C clear\n");
            if (I2C_FUNC_10BIT_ADDR & funcs)
                printf("  I2C_FUNC_10BIT_ADDR set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_10BIT_ADDR clear\n");
            if (I2C_FUNC_PROTOCOL_MANGLING & funcs)
                printf("  I2C_FUNC_PROTOCOL_MANGLING set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_PROTOCOL_MANGLING clear\n");
            if (I2C_FUNC_SMBUS_PEC & funcs)
                printf("  I2C_FUNC_SMBUS_PEC set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_PEC clear\n");
            if (I2C_FUNC_NOSTART & funcs)
                printf("  I2C_FUNC_NOSTART set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_NOSTART clear\n");
            if (I2C_FUNC_SMBUS_BLOCK_PROC_CALL & funcs)
                printf("  I2C_FUNC_SMBUS_BLOCK_PROC_CALL set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_BLOCK_PROC_CALL clear\n");
            if (I2C_FUNC_SMBUS_QUICK & funcs)
                printf("  I2C_FUNC_SMBUS_QUICK set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_QUICK clear\n");
            if (I2C_FUNC_SMBUS_READ_BYTE & funcs)
                printf("  I2C_FUNC_SMBUS_READ_BYTE set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_READ_BYTE clear\n");
            if (I2C_FUNC_SMBUS_WRITE_BYTE & funcs)
                printf("  I2C_FUNC_SMBUS_WRITE_BYTE set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_WRITE_BYTE clear\n");
            if (I2C_FUNC_SMBUS_READ_BYTE_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_READ_BYTE_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_READ_BYTE_DATA clear\n");
            if (I2C_FUNC_SMBUS_WRITE_BYTE_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_WRITE_BYTE_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_WRITE_BYTE_DATA clear\n");
            if (I2C_FUNC_SMBUS_READ_WORD_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_READ_WORD_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_READ_WORD_DATA clear\n");
            if (I2C_FUNC_SMBUS_WRITE_WORD_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_WRITE_WORD_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_WRITE_WORD_DATA clear\n");
            if (I2C_FUNC_SMBUS_PROC_CALL & funcs)
                printf("  I2C_FUNC_SMBUS_PROC_CALL set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_PROC_CALL clear\n");
            if (I2C_FUNC_SMBUS_READ_BLOCK_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_READ_BLOCK_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_READ_BLOCK_DATA clear\n");
            if (I2C_FUNC_SMBUS_WRITE_BLOCK_DATA & funcs)
                printf("  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_WRITE_BLOCK_DATA clear\n");
            if (I2C_FUNC_SMBUS_READ_I2C_BLOCK & funcs)
                printf("  I2C_FUNC_SMBUS_READ_I2C_BLOCK set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_READ_I2C_BLOCK clear\n");
            if (I2C_FUNC_SMBUS_WRITE_I2C_BLOCK & funcs)
                printf("  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK set\n");
            else if (functionality > 1)
                printf("    I2C_FUNC_SMBUS_WRITE_I2C_BLOCK clear\n");
        }
        close(fd);
        return 0;
    }

    if ((1 != times) && ((i2c_response_len > 0) || (wait_usecs > 0))) {
        fprintf(stderr, "when '-R <times>' is other than 1, '-r <num>' "
                "and '-w <usec>' options\nare not accepted.\n");
        exit(EXIT_FAILURE);
    }

    if (verbose > 2)
        fprintf(stderr, "read_hex read %d bytes from '-i' arguments\n",
                cmd_len);

    if (ten_bit_sa && (i2c_slave_addr < 0)) {
        fprintf(stderr, "In ten bit slave address mode '-s <sa>' must be "
                "given\n");
        exit(EXIT_FAILURE);
    }
    if ((! ten_bit_sa) && (i2c_slave_addr > 0x77)) {
        fprintf(stderr, "In seven bit slave address mode <sa> (%x) must not "
                "exceed 77 (hex)\n", i2c_slave_addr);
        exit(EXIT_FAILURE);
    }

    if ((i2c_slave_addr < 0) && (0 == test)) {
        if (cmd_len <= 0) {
            fprintf(stderr, "'-i' or '-F' option required, use '-h' for "
                    "help\n");
            exit(EXIT_FAILURE);
        }
        i2c_slave_addr = command[0] & 0x7f;
        memmove(command, command + 1, --cmd_len);
    }
    if (i2c_response_len > (int)sizeof(arr)) {
        fprintf(stderr, "'-r' argument (%d) exceeds allowed size (%d)\n",
                i2c_response_len, (int)sizeof(arr));
        exit(EXIT_FAILURE);
    }
    if (test) {
        if (verbose)
            fprintf(stderr, "In test mode, sending 1024 bytes of 0x%x\n",
                    (test > 1) ? 0xaa : 0x55);
        for (k = 0; k < (int)sizeof(command); ++k)
            command[k] = (test > 1) ? 0xaa : 0x55;
        cmd_len = sizeof(command);
        if (i2c_slave_addr < 0) {
            if (verbose)
                fprintf(stderr, "In test mode, use 0x%x for slave address\n",
                        (test > 1) ? 0x2a : 0x55);
            i2c_slave_addr = (test > 1) ? 0x2a : 0x55;
        }
    }

    if ((fd = open(dev_name, O_RDWR)) < 0) {
        perror("open failed");
        fprintf(stderr, "Tried to open %s; may need to load modules "
                "i2c_dev and/or i2c_gpio\n", dev_name);
        exit(EXIT_FAILURE);
    }
    if (verbose > 2) {
        fprintf(stderr, "About to send these bytes to slave_addr=%x:\n",
                i2c_slave_addr);
        for (k = 0; k < cmd_len; ++k) {
            if ((k > 0) && (0 == (k % 16)))
                fprintf(stderr, "\n");
            fprintf(stderr, " %02x", command[k]);
        }
        fprintf(stderr, "\n");
    }
    for (k = 0; k < times; ++k) {
        msg[k].addr = i2c_slave_addr;
        msg[k].flags = ignore_nak ? I2C_M_IGNORE_NAK : 0;   /* write */
        if (ten_bit_sa)
            msg[k].flags |= I2C_M_TEN;
        msg[k].len = cmd_len;
        msg[k].buf = /* (char *) */ command;
    }
    rdwr_arg.nmsgs = k;

    if ((i2c_response_len > 0) && (0 == wait_usecs)) {
        msg[1].addr = i2c_slave_addr;
        msg[1].flags = I2C_M_RD;        /* read */
        if (ignore_nak)
            msg[1].flags |= I2C_M_IGNORE_NAK;
        if (ten_bit_sa)
            msg[1].flags |= I2C_M_TEN;
        msg[1].len = i2c_response_len;
        msg[1].buf = /* (char *) */ arr;
        rdwr_arg.nmsgs = 2;
    }

    rdwr_arg.msgs = msg;

    if (ioctl(fd, I2C_RDWR, &rdwr_arg) < 0) {
        perror("ioctl(I2C_RDWR) [a] failed");
        exit(EXIT_FAILURE);
    }

   if ((i2c_response_len > 0) && (wait_usecs > 0)) {
        wait_req.tv_sec = wait_usecs / 1000000;
        wait_req.tv_nsec = (wait_usecs % 1000000) * 1000;
        while ((nanosleep(&wait_req, &rem) < 0) && (EINTR == errno))
            wait_req = rem;
        nanosleep(&wait_req, NULL);
        msg[0].addr = i2c_slave_addr;
        msg[0].flags = I2C_M_RD;
        if (ignore_nak)
            msg[0].flags |= I2C_M_IGNORE_NAK;
        if (ten_bit_sa)
            msg[0].flags |= I2C_M_TEN;
        msg[0].len = i2c_response_len;
        msg[0].buf = /* (char *) */ arr;
        rdwr_arg.nmsgs = 1;
        rdwr_arg.msgs = msg;
        if (ioctl(fd, I2C_RDWR, &rdwr_arg) < 0) {
            perror("ioctl(I2C_RDWR) [b] failed");
            exit(EXIT_FAILURE);
        }
    }

    for (k = 0; k < i2c_response_len; ++k) {
        if ((k > 0) && (0 == (k % 16)))
            printf("\n");
        printf(" %02x", arr[k]);
    }
    printf("\n");
    close(fd);
    return 0;
}
