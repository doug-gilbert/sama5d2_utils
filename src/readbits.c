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
 * readbits.c
 *
 * Utility for reading a GPIO line on a AT91SAM9G20/25/45 and SAMA5D3 in
 * Linux. Uses sysfs interface. The target hardware is a FoxG20, the Aria
 * G25 and SAMA5D3 family. This utility mimics the actions of a utility of
 * the same name for the FoxLX board. The Aria G25, FoxG20 and FoxLX
 * boards are made by Acme Systems. The SoCs are made by Atmel.
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
#include <time.h>
#include <errno.h>


static const char * version_str = "1.07 20131124";

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


static int verbose = 0;
static const char * gpio_name = NULL;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "readbits [-b BN] [-h] [-i] [-p PORT] [-r] [-u] [-U] [-v] [-V]\n"
            "  where:\n"
            "    -b BN        bit number within a port (0 to 31). Also\n"
            "                 accepts prefix like 'pb' or just 'b' for "
            "PORT.\n"
            "    -h           print usage message\n"
            "    -i           ignore line direction before reading (def: "
            "make input)\n"
            "    -p PORT      port ('a' to 'e') or gpio kernel line number "
            "(0 or more)\n"
            "    -r           print bit value to stdout (which is default "
            "action)\n"
            "                 used twice: exit value 0 for low, 1 for "
            "high\n"
            "    -u           unexport gpio line prior to reading bit\n"
            "    -U           leave line exported on exit\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n\n"
            "Read GPIO line state on AT91SAM9G20+G25/SAMA5D3 using sysfs. "
            "For the Aria G25\nand FoxG20 boards. This utility is similar "
            "to a FoxLX utility of the same\nname. Note: in earlier Linux "
            "kernels gpio kernel line numbers started at\n32 (for PA0), "
            "recent kernels start at 0 (for PA0), use '-vv' to check.\n"
            "Example: 'readbits -b PC7'\n");
}

static char *
get_best_gpio_name(int knum, char * b, int blen)
{
    if (gpio_name)
        snprintf(b, blen, "%s [kn=%d]", gpio_name, blen);
    else
        snprintf(b, blen, "knum=%d", knum);
    return b;
}


int
main(int argc, char ** argv)
{
    struct stat sb;
    int opt, k, exp_fd, unexp_fd, direction_fd, val_fd;
    int bn = -1;
    int ignore_dir = 0;
    int origin0 = 0;
    int read_val = 0;
    int knum = -1;
    int unexport = 0;
    int exported_on_exit = 0;
    int exported = 0;
    int ret = -1;
    const char * cp;
    char b[256];
    char base_dir[128];
    char ch;
    char bank = '\0';

    while ((opt = getopt(argc, argv, "b:hip:ruUvV")) != -1) {
        switch (opt) {
        case 'b':
            cp = optarg;
            if (isalpha(cp[0])) {
                if ('P' == toupper(*cp))
                    ++cp;
                ch = toupper(*cp);
                if ((ch >= 'A') && (ch <= 'E'))
                    bank = ch;
                else {
                    fprintf(stderr, "'-b' expects a letter ('A' to 'E')\n");
                    exit(EXIT_FAILURE);
                }
                gpio_name = optarg;
                ++cp;
            }
            k = atoi(cp);
            if ((k < 0) || (k > 31)) {
                fprintf(stderr, "'-b' expects a bit number from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            bn = k;
            break;
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
            break;
        case 'i':
            ++ignore_dir;
            break;
        case 'p':
            if (isalpha(*optarg)) {
                ch = toupper(*optarg);
                if ((ch >= 'A') && (ch <= 'E'))
                    bank = ch;
                else {
                    fprintf(stderr, "'-p' expects a letter ('A' to 'E') or "
                            "a number\n");
                    exit(EXIT_FAILURE);
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 511)) {
                    fprintf(stderr, "'-p' expects a letter or a number "
                            "0 or greater\n");
                    exit(EXIT_FAILURE);
                }
                knum = k;
            } else {
                fprintf(stderr, "'-p' expects a letter ('A' to 'E') or "
                        "a number\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'u':
            ++unexport;
            break;
        case 'U':
            ++exported_on_exit;
            break;
        case 'r':
            ++read_val;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
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

    if (! ((knum >= 0) || ((bank >= 'A') && (bn >= 0)))) {
        fprintf(stderr, "Expect either '-p PORT' or '-b BN'\n");
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
    if (knum < 0) {
        knum = ((! origin0) + bank - 'A') * 32 + bn;
        if (verbose)
            fprintf(stderr, "%c%d becomes kernel pin number %d\n", bank,
                    bn, knum);
    } else if ((! origin0) && (knum < 32)) {
        fprintf(stderr, "since %s not found assume kernel pin numbers "
                "start at 32\n(for PA0) so %d is too low\n",
                GPIO_BANK_ORIGIN, knum);
        exit(EXIT_FAILURE);
    }
    if ('\0' == bank) {
        bank = (char)((knum / 32) + 'A' - (! origin0));
        bn = knum % 32;
    }

    exp_fd = -1;
    unexp_fd = -1;
    direction_fd = -1;
    val_fd = -1;
    exp_fd = open(EXPORT_FILE, O_WRONLY);
    if (exp_fd < 0) {
        perror(EXPORT_FILE);
        goto bad;
    }
    unexp_fd = open(UNEXPORT_FILE, O_WRONLY);
    if (unexp_fd < 0) {
        perror(UNEXPORT_FILE);
        goto bad;
    }

    if (unexport) {
        snprintf(b, sizeof(b), "%d", knum);
        if (pwrite(unexp_fd, b, strlen(b), 0) < 0) {
            fprintf(stderr, "Unable to unexport %s: %s\n",
                    get_best_gpio_name(knum, b, sizeof(b)), strerror(errno));
            fprintf(stderr, "continue ...\n");
        }
    }
    snprintf(b, sizeof(b), "%d", knum);
    if (pwrite(exp_fd, b, strlen(b), 0) < 0) {
        fprintf(stderr, "Unable to export %s (already in use?): %s\n",
                get_best_gpio_name(knum, b, sizeof(b)), strerror(errno));
        goto bad;
    }
    exported = 1;
    /* check if /sys/class/gpio/gpio<knum> directory exists */
    snprintf(base_dir, sizeof(base_dir), "%s%d", GPIO_BASE_FILE, knum);
    if (stat(base_dir, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so continue in original manner\n",
                    base_dir);
    } else {
        if (verbose > 2)
            fprintf(stderr, "%s not found, now check for pinctrl "
                    "convention\n", base_dir);
        /* check if /sys/class/gpio/pio<bank><bn> directory exists */
        snprintf(base_dir, sizeof(base_dir), "%s%c%d", PIO_BASE_FILE, bank,
                 bn);
        if (stat(base_dir, &sb) >= 0) {
            if (verbose > 1)
                fprintf(stderr, "%s found so pinctrl convention\n", base_dir);
        } else {
            fprintf(stderr, "Unable to find sysfs directory %s (for "
                    "direction)\n", base_dir);
            goto bad;
        }
    }

    snprintf(b, sizeof(b), "%s/direction", base_dir);
    direction_fd = open(b, O_WRONLY);
    if (direction_fd < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        goto bad;
    }
    if (0 == ignore_dir) {
        if (pwrite(direction_fd, "in", 2, 0) < 0) {
            fprintf(stderr, "Unable to write 'in' to direction_fd: %s\n",
                    strerror(errno));
            goto bad;
        }
    }
    snprintf(b, sizeof(b), "%s/value", base_dir);
    val_fd = open(b, O_RDONLY);
    if (val_fd < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        goto bad;
    }
    k = pread(val_fd, b, 1, 0);
    if (k < 0) {
        fprintf(stderr, "pread() failed: %s\n", strerror(errno));
        goto bad;
    } else if (0 == k) {
        fprintf(stderr, "pread could not find a value\n");
        goto bad;
    }
    printf("%c\n", b[0]);
    if (read_val > 1)
        ret = ('0' !=  b[0]);
    else
        ret = 0;

bad:
    if (val_fd >= 0)
        close(val_fd);
    if (direction_fd >= 0)
        close(direction_fd);
    if (unexp_fd >= 0) {
        if (exported && (! exported_on_exit)) {
            snprintf(b, sizeof(b), "%d", knum);
            k = pwrite(unexp_fd, b, strlen(b), 0);
            /* assign to k to suppress warning */
        }
        close(unexp_fd);
    }
    if (exp_fd >= 0)
        close(exp_fd);
    if (ret)
        return 1;
    else
        return 0;
}
