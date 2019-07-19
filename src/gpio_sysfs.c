/*
 * Copyright (c) 2010-2018 Douglas Gilbert.
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

/************************************************************************
 * gpio_sysfs.c
 *
 * Utility for testing SAMA5D2 family GPIO pins. Can read, set and toggle
 * GPIO lines. The SAMA5D2 family has 4 banks of 32 pins: PA0-PA31,
 * PB0-PB31, PC0-PC31 and PD0-31. Some GPIO pins may be committed to other
 * uses thus not available. Uses sysfs gpio(lib) interface.
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
#include <poll.h>
#include <sched.h>


static const char * version_str = "1.12 20180730";

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

#define DEF_NUM_TOGGLE 1000000
#define PIO_BANKS_SAMA5D2 4
#define LINES_PER_BANK 32


static int verbose = 0;
static const char * gpio_name = NULL;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "gpio_sysfs [-b BN] [-c] [-d USEC] [-e] [-f] [-h] [-n NUM]\n"
            "                  [-p PORT] [-r] [-R] [-s 0|1] [-t] "
            "[-u] [-U]\n"
            "                  [-v] [-V]\n"
            "  where:\n"
            "    -b BN        bit number within port (0 to 31). "
            "Also\n"
            "                 accepts prefix like 'pb' or just 'b' for "
            "PORT.\n"
            "                 Example: '-b PC7' equivalent to '-p c "
            "-b 7'\n"
            "    -c           count rising edges for USEC (def: 1 sec), "
            "twice:\n"
            "                 count falling edges, thrice ('-ccc'): count "
            "all edges\n"
            "    -d USEC      with '-t': delay after each transition (def: "
            "0)\n"
            "                 with '-c': duration to count (def: 1000000 "
            "(1 second))\n"
            "    -e           enumerate pin names with corresponding "
            "kernel pin\n"
            "    -f           force (drive) high when toggling (default "
            "for high\n"
            "                 is input mode and assume pullup)\n"
            "    -h           print usage message\n"
            "    -n NUM       number of cycles to toggle gpio line (def: "
            "1000000)\n"
            "    -p PORT      port bank ('A' to 'E') or gpio kernel line "
            "number\n"
            "    -r           read gpio line and send value to stdout\n"
            "                 used twice: exit value 0 for low, 1 for "
            "high\n"
            "    -R           realtime scheduling: SCHED_FIFO (def: non "
            "realtime)\n"
            "                 use twice for realtime scheduling: "
            "SCHED_RR\n"
            "    -s 0|1       set GPIO line to 0 (low) or 1 (high)\n"
            "    -t           toggle gpio line NUM times\n"
            "    -u           unexport gpio line before other actions\n"
            "    -U           leave line exported on exit (ignored if "
            "with '-S')\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n\n"
            "SAMA5D2 SoC family GPIO test program. Uses sysfs interface.\n"
            "Can set and read lines. Can toggle line ('-t') NUM times with "
            "USEC\ndelay after to each transition. Beware: counting over "
            "20,000\nevents per second may starve (freeze) the kernel.\n");
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

static int
gs_export(int * exp_fdp, int knum)
{
    int res;
    char b[80];

    snprintf(b, sizeof(b), "%d", knum);
    res = pwrite(*exp_fdp, b, strlen(b), 0);
    if (res < 0)
        fprintf(stderr, "Unable to export %s (already in use?): %s\n",
                get_best_gpio_name(knum, b, sizeof(b)), strerror(errno));
    return res;
}

static int
gs_unexport(int * unexp_fdp, int knum)
{
    int res;
    char b[80];

    snprintf(b, sizeof(b), "%d", knum);
    res = pwrite(*unexp_fdp, b, strlen(b), 0);
    if (res < 0)
        fprintf(stderr, "Unable to unexport %s: %s\n",
                get_best_gpio_name(knum, b, sizeof(b)), strerror(errno));
    return res;
}

static void
calc_finish_time(struct timespec * resultp, const struct timespec * periodp)
{
    long ns;
    struct timespec now_ts;

    /* clock_gettime() needs '-lrt' on the link line */
    if (clock_gettime(CLOCK_REALTIME, &now_ts) < 0) {
        fprintf(stderr, "clock_gettime(CLOCK_REALTIME) failed: %s\n",
                strerror(errno));
        resultp->tv_sec = 0;
        resultp->tv_nsec = 0;
        return;
    }
    resultp->tv_sec = now_ts.tv_sec + periodp->tv_sec;
    ns = now_ts.tv_nsec + periodp->tv_nsec;
    if (ns > 999999999) {
        ns -= 1000000000;
        ++resultp->tv_sec;
    }
    resultp->tv_nsec = ns;
}

/* Return 0 if fin_tsp has been reached. Else returns number of
 * milliseconds until finish time */
static int
millisecs_rem(const struct timespec * fin_tsp)
{
    struct timespec now_ts;
    int ms;

    clock_gettime(CLOCK_REALTIME, &now_ts);
    if (now_ts.tv_sec >= fin_tsp->tv_sec) {
        if (now_ts.tv_sec > fin_tsp->tv_sec)
            return 0;
        if (now_ts.tv_nsec >= fin_tsp->tv_nsec)
            return 0;
        ms = (fin_tsp->tv_nsec - now_ts.tv_nsec) / 1000000;
        if (0 == ms)
            ++ms;       /* round up to 1 ms if small */
        return ms;
    }
    return (int)(((fin_tsp->tv_sec - now_ts.tv_sec) * 1000) +
                ((fin_tsp->tv_nsec - now_ts.tv_nsec) / 1000000));
}

/* Returns the number of edges detected (param=1: rising; param=2: falling;
 * otherwise: both) of -1 if problem. See Linux kernel source file:
 * Documentation/gpio.txt for explanation. */
static int
process_count(int param, const char * base_dp,
              const struct timespec * periodp, int * direction_fdp,
              int *val_fdp)
{
    int res, ret, edge_fd, edges, ms, k;
    struct timespec fin_ts;
    struct pollfd a_poll;
    const char * cp;
    char b[140];
    char vfn[128];

    ret = -1;
    edge_fd = -1;
    snprintf(b, sizeof(b), "%s/direction", base_dp);
    *direction_fdp = open(b, O_WRONLY);
    if (*direction_fdp < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        goto bbad;
    }
    snprintf(b, sizeof(b), "%s/edge", base_dp);
    edge_fd = open(b, O_WRONLY);
    if (edge_fd < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        goto bbad;
    }
    snprintf(b, sizeof(b), "%s/value", base_dp);
    *val_fdp = open(b, O_RDONLY);
    if (*val_fdp < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        goto bbad;
    }
    strcpy(vfn, b);
    if (pwrite(*direction_fdp, "in", 2, 0) < 0) {
        fprintf(stderr, "Unable to write in to direction_fd: %s\n",
                strerror(errno));
        goto bbad;
    }
    switch (param) {
    case 1:
        cp = "rising";
        break;
    case 2:
        cp = "falling";
        break;
    default:
        cp = "both";
        break;
    }
    if (pwrite(edge_fd, cp, strlen(cp), 0) < 0) {
        fprintf(stderr, "Unable to write '%s' to edge_fd: %s\n", cp,
                strerror(errno));
        goto bbad;
    }
    a_poll.fd = *val_fdp;
    a_poll.events = POLLPRI;        /* magic */
    a_poll.revents = 0;
    edges = 0;
    k = pread(*val_fdp, b, 1, 0);
    calc_finish_time(&fin_ts, periodp);
    while ((ms = millisecs_rem(&fin_ts)) > 0) {
        res = poll(&a_poll, 1, ms);
        if (res < 0) {
            fprintf(stderr, "poll() failed: %s\n", strerror(errno));
            goto bbad;
        } else if (0 == res)
            break;
        if (a_poll.revents & (POLLPRI | POLLERR)) {
#if 0
            /* check levels manually */
#if 0
            /* lseek seems faster than close+re-open */
            if (lseek(*val_fdp, 0, SEEK_SET) < 0) {
                fprintf(stderr, "lseek to start of value fd failed: %s\n",
                        strerror(errno));
                goto bbad;
            }
#else
            close(*val_fdp);
            *val_fdp = open(vfn, O_RDONLY);
            a_poll.fd = *val_fdp;
#endif
            b[0] = '0';
            k = pread(*val_fdp, b, 1, 0);
            if (param > 2)
                ++edges;
            else {
                if ((('0' == b[0]) && (2 == param)) ||
                    (('1' == b[0]) && (1 == param)))
                    ++edges;
            }
#else
            /* believe edge interrupts, don't check levels which since
             * that is slow and racy. N.B. the "value" file still needs
             * to be read to clear the event. */
            ++edges;
#if 1
            if (lseek(*val_fdp, 0, SEEK_SET) < 0) {
                fprintf(stderr, "lseek to start of value fd failed: %s\n",
                        strerror(errno));
                goto bbad;
            }
            k = pread(*val_fdp, b, 1, 0);
#else
            close(*val_fdp);
            *val_fdp = open(vfn, O_RDONLY);
            a_poll.fd = *val_fdp;
            k = pread(*val_fdp, b, 1, 0);
#endif

#endif
        }
        a_poll.events = POLLPRI;
        a_poll.revents = 0;
    }
    ret = edges;
bbad:
    if (edge_fd >= 0) {
        k = pwrite(edge_fd, "none", 4, 0);
        close(edge_fd);
    }
    if (k) { }   /* suppress warning */
    return ret;
}

/* Returns 0 if ok, else -1 */
static int
process_toggle(const char * base_dp, int num, int force, int have_delay,
               const struct timespec * delayp, int * direction_fdp,
               int * val_fdp)
{
    int k;
    char b[140];

    snprintf(b, sizeof(b), "%s/direction", base_dp);
    *direction_fdp = open(b, O_WRONLY);
    if (*direction_fdp < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }
    snprintf(b, sizeof(b), "%s/value", base_dp);
    *val_fdp = open(b, O_WRONLY);
    if (*val_fdp < 0) {
        fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
        return -1;
    }
    if (pwrite(*direction_fdp, "out", 3, 0) < 0) {
        fprintf(stderr, "Unable to write out to direction_fd: %s\n",
                strerror(errno));
        return -1;
    }
    if (have_delay && verbose)
        fprintf(stderr, "After each edge delay for %d.%06ld seconds\n",
                (int)delayp->tv_sec, delayp->tv_nsec / 1000);

    for (k = 0; k < num; ++k) {
        if (force) {
            if (pwrite(*val_fdp, "0", 1, 0) < 0) {
                fprintf(stderr, "Unable to pwrite 0 to val_fd: %s\n",
                        strerror(errno));
                return -1;
            }
            if (have_delay)
                nanosleep(delayp, NULL);
            if (pwrite(*val_fdp, "1", 1, 0) < 0) {
                fprintf(stderr, "Unable to pwrite 1 to val_fd: %s\n",
                        strerror(errno));
                return -1;
            }
            if (have_delay)
                nanosleep(delayp, NULL);
        } else {
            if (pwrite(*direction_fdp, "low", 3, 0) < 0) {
                fprintf(stderr, "Unable to pwrite low to direction_fd: %s\n",
                        strerror(errno));
                return -1;
            }
            if (have_delay)
                nanosleep(delayp, NULL);
            if (pwrite(*direction_fdp, "in", 2, 0) < 0) {
                fprintf(stderr, "Unable to pwrite in to direction_fd: %s\n",
                        strerror(errno));
                return -1;
            }
            if (have_delay)
                nanosleep(delayp, NULL);
        }
    }
    return 0;
}


int
main(int argc, char ** argv)
{
    int opt, k, j, n, num, res, exp_fd, unexp_fd, direction_fd, val_fd, edges;
    int count_opt = 0;
    int have_delay = 0;
    int enumerate = 0;
    int force = 0;
    int knum = -1;
    int num_toggle = DEF_NUM_TOGGLE;
    int origin0 = 0;
    int read_val = 0;
    int rt_sched = 0;
    int state = -1;
    int toggle = 0;
    int unexport = 0;
    int exported_on_exit = 0;
    int ret = -1;
    int bit_num = -1;
    const char * cp;
    char b[256];
    char base_dir[128];
    char ch;
    char bank = '\0';
    struct timespec delay_req;
    struct stat sb;
    struct sched_param spr;

    delay_req.tv_sec = 0;
    delay_req.tv_nsec = 0;
    while ((opt = getopt(argc, argv, "b:cd:efhn:p:rRs:tuUvV")) != -1) {
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
            bit_num = k;
            break;
        case 'c':
            ++count_opt;
            break;
        case 'd':
            k = atoi(optarg);
            if (k < 0) {
                fprintf(stderr, "'-d' expects a delay in microseconds from "
                        "0 to 2147483647\n");
                exit(EXIT_FAILURE);
            }
            delay_req.tv_sec = k / 1000000;     /* seconds component */
            delay_req.tv_nsec = (k % 1000000) * 1000;  /* nanoseconds */
            have_delay = (delay_req.tv_sec || delay_req.tv_nsec);
            break;
        case 'e':
            ++enumerate;
            break;
        case 'f':
            ++force;
            break;
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
            break;
        case 'n':
            num_toggle = atoi(optarg);
            break;
        case 'p':
            if (isalpha(*optarg)) {
                ch = toupper(*optarg);
                if ((ch >= 'A') && (ch <= 'E'))
                    bank = ch;
                else {
                    fprintf(stderr, "'-p' expects a letter ('A' to 'E')\n");
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
        case 'r':
            ++read_val;
            break;
        case 'R':
            ++rt_sched;
            break;
        case 's':
            state = atoi(optarg);
            if ((state < 0) || (state > 1)) {
                fprintf(stderr, "'-s' expects '0' or '1'\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 't':
            ++toggle;
            break;
        case 'u':
            ++unexport;
            break;
        case 'U':
            ++exported_on_exit;
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

    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so kernel pin numbers start at 0 "
                    "(for PA0)\n", GPIO_BANK_ORIGIN);
        origin0 = 1;
    } else if (verbose > 2)
        fprintf(stderr, "%s not found so kernel pin numbers start at 32 "
                "(for PA0)\n", GPIO_BANK_ORIGIN);

    if (enumerate) {
        num = PIO_BANKS_SAMA5D2;
        for (k = 0; k < LINES_PER_BANK; ++k) {
            for (j = 0; j < num; ++j) {
                n = ((j + (! origin0)) * 32) + k;
                printf("%sP%c%d: %d   ", (j ? "\t" : ""), 'A' + j, k, n);
            }
            printf("\n");
        }
        return 0;
    }

    if ((knum >= 0) && (bit_num >= 0)) {
        fprintf(stderr, "Give either '-p PORT' or '-b BN' but not both\n");
        exit(EXIT_FAILURE);
    } else if (bank) {
        if (bit_num < 0) {
            fprintf(stderr, "If '-p PORT' given then also need "
                    "'-b BN'\n");
            exit(EXIT_FAILURE);
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        fprintf(stderr, "Need to give gpio line with '-p PORT' and/or "
                "'-b BN'\n");
        usage();
        exit(EXIT_FAILURE);
    }
    if ('\0' == bank) {
        bank = (char)((knum / 32) + 'A' - (! origin0));
        bit_num = knum % 32;
    }

    if (! (count_opt || toggle || unexport || read_val ||
          (state >= 0))) {
        fprintf(stderr, "Expect at least '-c', '-r', '-s', '-t' or "
                "'-u' option, use '-h' for usage\n");
        usage();
        exit(EXIT_FAILURE);
    }
    if (read_val && (toggle || (state >= 0))) {
        fprintf(stderr, "Can't have '-r' with '-s' or '-t'\n");
        usage();
        exit(EXIT_FAILURE);
    }

    if (rt_sched) {
        k = sched_get_priority_min((1 == rt_sched) ? SCHED_FIFO : SCHED_RR);
        if (k < 0)
            fprintf(stderr, "sched_get_priority_min: %s\n",
                    strerror(errno));
        else {
            spr.sched_priority = k;
            if (sched_setscheduler(0, ((1 == rt_sched) ? SCHED_FIFO :
                                                         SCHED_RR), &spr) < 0)
                fprintf(stderr, "sched_setscheduler: %s\n",
                        strerror(errno));
        }
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
        if (gs_unexport(&unexp_fd, knum) < 0)
            fprintf(stderr, "continue ...\n");
    }

    if ((knum >= 0) && gs_export(&exp_fd, knum) < 0)
        goto bad;
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
        /* check if /sys/class/gpio/pio<PORT><BN> directory exists */
        snprintf(base_dir, sizeof(base_dir), "%s%c%d", PIO_BASE_FILE,
                 bank, bit_num);
        if (stat(base_dir, &sb) >= 0) {
            if (verbose > 1)
                fprintf(stderr, "%s found so pinctrl convention\n",
                        base_dir);
        } else {
            fprintf(stderr, "Unable to find sysfs directory %s (for "
                    "direction)\n", base_dir);
            goto bad;
        }
    }

    if (count_opt) {
        if (0 == have_delay)
            delay_req.tv_sec = 1;       /* default to count for 1 second */
        edges = process_count(count_opt, base_dir, &delay_req, &direction_fd,
                              &val_fd);
        if (! exported_on_exit) {
            if (gs_unexport(&unexp_fd, knum) < 0)
                goto bad;
        }
        if (edges < 0)
            goto bad;
        printf("Count=%d\n", edges);
        ret = 0;
    }

    if (toggle) {
        if (verbose)
            fprintf(stderr, "Toggling %s\n",
                    get_best_gpio_name(knum, b, sizeof(b)));
        res = process_toggle(base_dir, num_toggle, force, have_delay,
                             &delay_req, &direction_fd, &val_fd);
        if (res < 0)
            goto bad;
        if (state >= 0) {
            if (0 == state)
                res = pwrite(direction_fd, "low", 3, 0);
            else
                res = pwrite(direction_fd, "high", 4, 0);
            if (res < 0) {
                fprintf(stderr, "Unable to pwrite low/high to direction_fd: "
                        "%s\n", strerror(errno));
                goto bad;
            }
        }
        if (! exported_on_exit) {
            if (gs_unexport(&unexp_fd, knum) < 0)
                goto bad;
        }
        ret = 0;
    } else if (state >= 0) {
        snprintf(b, sizeof(b), "%s/direction", base_dir);
        direction_fd = open(b, O_WRONLY);
        if (direction_fd < 0) {
            fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
            goto bad;
        }
        if (0 == state)
            res = pwrite(direction_fd, "low", 3, 0);
        else
            res = pwrite(direction_fd, "high", 4, 0);
        if (res < 0) {
            fprintf(stderr, "Unable to pwrite low/high to direction_fd: %s\n",
                    strerror(errno));
            goto bad;
        }
        if (! exported_on_exit) {
            if (gs_unexport(&unexp_fd, knum) < 0)
                goto bad;
        }
        ret = 0;
    } else if (read_val) {
        snprintf(b, sizeof(b), "%s/direction", base_dir);
        direction_fd = open(b, O_WRONLY);
        if (direction_fd < 0) {
            fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
            goto bad;
        }
        snprintf(b, sizeof(b), "%s/value", base_dir);
        val_fd = open(b, O_RDONLY);
        if (val_fd < 0) {
            fprintf(stderr, "Open %s: %s\n", b, strerror(errno));
            goto bad;
        }
        if (pwrite(direction_fd, "in", 2, 0) < 0) {
            fprintf(stderr, "Unable to pwrite in to direction_fd: %s\n",
                    strerror(errno));
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
        if (! exported_on_exit) {
            if (gs_unexport(&unexp_fd, knum) < 0) {
                ret = 1;
                goto bad;
            }
        }
    } else
        ret = 0;

bad:
    if (val_fd >= 0)
        close(val_fd);
    if (direction_fd >= 0)
        close(direction_fd);
    if (unexp_fd >= 0)
        close(unexp_fd);
    if (exp_fd >= 0)
        close(exp_fd);
    if (ret)
        exit(EXIT_FAILURE);
    else
        return 0;
}
