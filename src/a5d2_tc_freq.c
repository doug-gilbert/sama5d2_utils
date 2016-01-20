/*
 * Copyright (c) 2016 Douglas Gilbert.
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
 * a5d2_tc_freq.c
 *
 * Utility to produce the given frequencies for the given durations.
 * Uses the SAMA5D2 timer-counter (TC) unit to generate the
 * frequencies on PIO lines corresponding to TIOA0-5 and TIOB0-5.
 * The SAMA5D2 has two TC blocks, each with 3 TCs; TCB0 contains TC0, TC1,
 * and TC2 while TCB1 contains TC3, TC4 and TC4.
 * Note the Linux kernel uses TC0 for internal purposes.
 *
 ****************************************************************/


#define _XOPEN_SOURCE 500
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <limits.h>
#include <syslog.h>
#include <signal.h>
#include <sched.h>

// #include <sys/ioctl.h>


static const char * version_str = "1.00 20160119";

#define MAX_ELEMS 512
#define DEV_MEM "/dev/mem"
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)

/* On the SAMA5D2 each TCB has a separate peripheral identifier:
 * TCB0 is 35 and TCB1 is 36. Since the Linux kernel uses TC0 which is
 * in TCB0 then it should be enabled. */
#define PMC_PCER0  0xf0014010   /* peripheral clock enable, reg 0 */
#define PMC_PCDR0  0xf0014014   /* peripheral clock disable, reg 0 */
#define PMC_PCSR0  0xf0014018   /* peripheral clock status, reg 0 */
#define PMC_PCER1  0xf0014100   /* peripheral clock enable, reg 1 */
#define PMC_PCDR1  0xf0014104   /* peripheral clock disable, reg 1 */
#define PMC_PCSR1  0xf0014108   /* peripheral clock status, reg 1 */
#define PMC_PCR    0xf001410c   /* peripheral control register */
                                /* - for peripheral divider */
#define PMC_PCR_GCKDIV_MSK 0xff00000
#define PMC_PCR_GCKDIV_SHIFT 20

#define SAMA5D2_PERI_ID_TCB0 35 /* contains TC0, TC1 and TC2 */
#define SAMA5D2_PERI_ID_TCB1 36 /* contains TC3, TC4 and TC5 */

// wave=1, wavesel=2, T_CLK1 EEVT=1
#define TC_CMR_VAL_CLK1  0x0000c400
// wave=1, wavesel=2, T_CLK5 EEVT=1
#define TC_CMR_VAL_CLK5  0x0000c404
#define TC_CMR_TCCLKS_DEF 0     /* Generic clock from PMC (divided by 1) */

// BSWTRG=1 BCPC=1 BCPB=2, ASWTRG=2 ACPC=2 ACPA=1 : TIOA? leads with mark
#define TC_CMR_MS_MASK  0x46890000
// BSWTRG=2 BCPC=2 BCPB=1, ASWTRG=1 ACPC=1 ACPA=2 : TIOA? leads with space
#define TC_CMR_MS_INV_MASK  0x89460000

#define TC_CCR_SWTRG 4          /* Software trigger */
#define TC_CCR_CLKDIS 2         /* Clock disable */
#define TC_CCR_CLKEN 1          /* Clock enable, if TC_CCR_CLKDIS not given */

/* TIMER_CLOCK1 is the generic clock (per TCB block) from the PMC which is
 * often the "master" clock (166 MHz). The main clock is often 12 MHz.
 */
#define TIMER_CLOCK1 166000000 /* master clock is 166 MHz */
#define TIMER_CLOCK5 32768

#define A5D2_TCB_WPKEY 0x54494D  /* "TIM" in ASCII */


/* N.B. the SAMA5D2 Timer Counter unit has 32 bit counters. Some earlier
 * members of the AT91 family had 16 bit counters. */

struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    bool mmap_ok;
};

/* when both members are zero then end of elem_arr */
struct elem_t {
    int frequency;      /* > 0 then Hz; < 0 then period in ms; 0 then low */
    int duration_ms;    /* > 0 then duration in ms; -1 then continual */
};

struct table_io_t {
    char tcb;                   /* 0 for TCB0 (TC0, TC1, RC2) else 1 */
    const char * tio_name;
    unsigned int tc_ccr;
    unsigned int tc_cmr;
    unsigned int tc_ra;
    unsigned int tc_rb;
    unsigned int tc_rc;
    unsigned int tc_imr;
    unsigned int tc_emr;        /* extended mode register */
    unsigned int tc_wpmr;       /* one write protect mode reg per tcb */
    int is_tioa;
};

struct value_str_t {
    int val;
    const char *str;
};


static struct elem_t elem_arr[MAX_ELEMS];

/* settings for TIOA0-5 and TIOB0-5. */
static struct table_io_t table_arr[] = {
    /* TCB0:   base 0xf800c000 */
    {0, "TIOA0", 0xf800c000, 0xf800c004, 0xf800c014,
     0xf800c018, 0xf800c01c, 0xf800c02c, 0xf800c030, 0xf800c0e4, 1},
    {0, "TIOB0", 0xf800c000, 0xf800c004, 0xf800c014,
     0xf800c018, 0xf800c01c, 0xf800c02c, 0xf800c030, 0xf800c0e4, 0},
    {0, "TIOA1", 0xf800c040, 0xf800c044, 0xf800c054,
     0xf800c058, 0xf800c05c, 0xf800c06c, 0xf800c070, 0xf800c0e4, 1},
    {0, "TIOB1", 0xf800c040, 0xf800c044, 0xf800c054,
     0xf800c058, 0xf800c05c, 0xf800c06c, 0xf800c070, 0xf800c0e4, 0},
    {0, "TIOA2", 0xf800c080, 0xf800c084, 0xf800c094,
     0xf800c098, 0xf800c09c, 0xf800c0ac, 0xf800c0b0, 0xf800c0e4, 1},
    {0, "TIOB2", 0xf800c080, 0xf800c084, 0xf800c094,
     0xf800c098, 0xf800c09c, 0xf800c0ac, 0xf800c0b0, 0xf800c0e4, 0},

    /* TCB1:   base 0xf8010000 */
    {1, "TIOA3", 0xf8010000, 0xf8010004, 0xf8010014,
     0xf8010018, 0xf801001c, 0xf801002c, 0xf8010030, 0xf80100e4, 1},
    {1, "TIOB3", 0xf8010000, 0xf8010004, 0xf8010014,
     0xf8010018, 0xf801001c, 0xf801002c, 0xf8010030, 0xf80100e4, 0},
    {1, "TIOA4", 0xf8010040, 0xf8010044, 0xf8010054,
     0xf8010058, 0xf801005c, 0xf801006c, 0xf8010070, 0xf80100e4, 1},
    {1, "TIOB4", 0xf8010040, 0xf8010044, 0xf8010054,
     0xf8010058, 0xf801005c, 0xf801006c, 0xf8010070, 0xf80100e4, 0},
    {1, "TIOA5", 0xf8010080, 0xf8010084, 0xf8010094,
     0xf8010098, 0xf801009c, 0xf80100ac, 0xf80100b0, 0xf80100e4, 1},
    {1, "TIOB5", 0xf8010080, 0xf8010084, 0xf8010094,
     0xf8010098, 0xf801009c, 0xf80100ac, 0xf80100b0, 0xf80100e4, 0},

    {0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

static struct value_str_t tcclks_arr[] = {
    {0, "TIMER_CLOCK1  Generic clock (GCLK) from PMC"},
    {1, "TIMER_CLOCK2  GCLK div 8"},
    {2, "TIMER_CLOCK3  GCLK div 32"},
    {3, "TIMER_CLOCK4  GCLK div 128"},
    {4, "TIMER_CLOCK5  slow clock"},
    {5, "XC0"},
    {6, "XC1"},
    {7, "XC2"},
    {-1, NULL},
};

static int tc_tclock1 = TIMER_CLOCK1;   /* may get divided by up to 256 */
static int cl_foreground = 1;

static int verbose = 0;


#ifdef __GNUC__
static int pr2serr(const char * fmt, ...)
        __attribute__ ((format (printf, 1, 2)));
#else
static int pr2serr(const char * fmt, ...);
#endif


static int
pr2serr(const char * fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = vfprintf(stderr, fmt, args);
    va_end(args);
    return n;
}

static void
usage(int do_help)
{
    if (do_help > 1)
        goto second_help;
    pr2serr("Usage: a5d2_tc_freq -b TIO [-c TCCLKS] [-d] [-D] [-e] [-f FN] "
            "[-h]\n"
            "                    [-i] [-I] [-m M,S] [-M] [-n] "
            "[-p F1,D1[,F2,D2...]]\n"
            "                    [-R RF] [-u] [-v] [-V] [-w WPEN]\n"
            "  where:\n"
            "    -b TIO       TIO name ('TIOA0', 'TIOB0' to 'TIOA5' or "
            "'TIOB5')\n"
            "    -c TCCLKS    clock source (def: 0 -> generic clock from "
            "PMC)\n"
            "    -d           dummy mode: decode frequency,duration pairs, "
            "print\n"
            "                 them then exit; ignore TIO\n"
            "    -D           after initial checks, run as daemon which "
            "exits after\n"
            "                 frequency(s) is produced\n"
            "    -e           enumerate TIO names and TCCLKS clock sources\n"
            "    -f FN        obtain input from file FN. A FN of '-' "
            "taken as\n"
            "                 read stdin. If '-f' not given then '-p' "
            "option expected\n"
            "    -h           print usage message\n"
            "    -i           initialize TIO for frequency output, (def: "
            "assume already\n"
            "                 set up). Line set low prior to frequency "
            "generation\n"
            "    -I           invert levels of mark and space\n"
            "    -m M,S       mark (M) space (S) ratio (def: 1,1), both "
            "should\n"
            "                 be positive; ratio inverted for TIOB*\n"
            "    -M           show TC interrupt mask register then exit\n"
            "    -n           no realtime scheduling (def: set "
            "SCHED_FIFO)\n"
            "    -p F1,D1[,F2,D2...]    one or more frequency duration "
            "pairs; frequency\n"
            "                           in Hz and the duration in "
            "milliseconds\n"
            "    -R RF        use RF as reference frequency for "
            "TIMER_CLOCK1\n"
            "    -u           disable the TIO clock prior to exiting\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n"
            "    -w WPEN      set or show write protect (WP) information "
            "for TCB.\n"
            "                 0 -> disable (def, no WP), 1 -> enable, "
            "-1 -> show\n"
            "                 WP en/disable state. Then in all cases "
            "exit\n\n"
            "Use the timer counter (TC) in the SAMA5D2 SoCs to generate "
            "frequencies.\nUse '-h' twice for more help.\n"
           );
    return;

second_help:
    pr2serr("List of frequency (Hz when positive), duration (milliseconds "
            "when positive)\npairs can be given on the command line ('-p') "
            "or in a file ('-f'). Use a\nfrequency of 0 for a delay (line "
            "put at space level (usually low: 0\nvolts)).\nThe first time "
            "this utility is called option '-i' probably should\nbe given "
            "to initialize the TC. Duration of -1 for continuous (exit and\n"
            "maintain), assumes '-i'. A negative frequency is treated as "
            "a period in\nmilliseconds, so -1 gives a period of 1 "
            "millisecond (i.e. 1000 Hz) while\n-2500, for example, gives a "
            "period of 2.5 seconds. The maximum period is\n131071.999 "
            "seconds, corresponding to -131071999 . The maximum "
            "frequency\ndepends on the master clock (MCK) and is "
            "typically 41.5 MHz.\n\nFrequencies and durations can have "
            "multiplier suffixes: ki, Mi, Gi for\n2**10, 2**20 and 2**30 "
            "respectively; or k, M, G for 10**3, 10**6 and\n10**9 "
            "respectively. 30MHz is 30*(10**6) Hz while 32 kiH is 32768 "
            "Hz.\n"
           );
}

#ifdef __cplusplus

static inline volatile unsigned int *
mmp_add(void * p, unsigned int v)
{
    return (volatile unsigned int *)((unsigned char *)p + v);
}

#else

static inline volatile unsigned int *
mmp_add(unsigned char * p, unsigned int v)
{
    return (volatile unsigned int *)(p + v);
}

#endif

void
cl_print(int priority, const char *fmt, ...)
{
    va_list ap;

    va_start(ap,fmt);
    if (cl_foreground)
        vfprintf(stderr, fmt, ap);
    else
        vsyslog(priority, fmt, ap);
    va_end(ap);
}

/* The number in 'buf' together with an optional multiplier suffix is decoded
 * or -1 is returned. Accepts a hex prefix (0x or 0X) or a decimal multiplier
 * suffix of kHz for x1000 Hz, kiHz for x1024; same pattern for MHz and GHz.
 * Ignore leading spaces and tabs; accept comma, space, tab and hash as
 * terminator. */
int
fr_get_num(const char * buf)
{
    int res, num, n, len;
    unsigned int unum;
    char * cp;
    const char * b;
    char c = 'c';
    char c2;
    char lb[16];

    if ((NULL == buf) || ('\0' == buf[0]))
        return -1;
    len = strlen(buf);
    n = strspn(buf, " \t");
    if (n > 0) {
        if (n == len)
            return -1;
        buf += n;
        len -=n;
    }
    /* following hack to keep C++ happy */
    cp = strpbrk((char *)buf, " \t,#");
    if (cp) {
        len = cp - buf;
        n = (int)sizeof(lb) - 1;
        len = (len < n) ? len : n;
        memcpy(lb, buf, len);
        lb[len] = '\0';
        b = lb;
    } else
        b = buf;
    if (('0' == b[0]) && (('x' == b[1]) || ('X' == b[1]))) {
        res = sscanf(b + 2, "%x", &unum);
        num = unum;
    } else if ('H' == toupper((int)b[len - 1])) {
        res = sscanf(b, "%x", &unum);
        num = unum;
    } else
        res = sscanf(b, "%d%c%c", &num, &c, &c2);
    if (res < 1)
        return -1LL;
    else if (1 == res)
        return num;
    else {
        if (res > 2)
            c2 = toupper((int)c2);
        switch (toupper((int)c)) {
        case 'K':
            if (2 == res)
                return num * 1000;
            if ('I' == c2)
                return num * 1024;
            else if ('H' == c2)
                return num * 1000;
            return -1;
        case 'M':
            if (2 == res)
                return num * 1000000;
            if ('I' == c2)
                return num * 1048576;
            else if ('H' == c2)
                return num * 1000000;
            return -1;
        case 'G':
            if (2 == res)
                return num * 1000000000;
            if ('I' == c2)
                return num * 1073741824;
            else if ('H' == c2)
                return num * 1000000000;
            return -1;
        default:
            pr2serr("unrecognized multiplier (expect k, M or G)\n");
            return -1;
        }
    }
}

/* Daemonize the current process. */
void
cl_daemonize(const char * name, int no_chdir, int no_varrunpid, int verbose)
{
    pid_t pid, sid, my_pid;
    char b[64];
    FILE * fp;

    /* already a daemon */
    if (getppid() == 1 )
        return;

    pid = fork();
    if (pid < 0) {
        strcpy(b, name);
        strcat(b, " fork");
        perror(b);
        exit(EXIT_FAILURE);
    }
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* Cancel certain signals */
    signal(SIGCHLD, SIG_DFL);   /* A child process dies */
    signal(SIGTSTP, SIG_IGN);   /* Various TTY signals */
    signal(SIGTTOU, SIG_IGN);
    signal(SIGTTIN, SIG_IGN);
    signal(SIGHUP, SIG_IGN);    /* Ignore hangup signal */
    signal(SIGTERM, SIG_DFL);   /* Die on SIGTERM */

    umask(0);

    sid = setsid();
    if (sid < 0) {
        cl_print(LOG_ERR, "setsid: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
    if (! no_chdir) {
        if ((chdir("/")) < 0) {
            cl_print(LOG_ERR, "chdir(/): %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    fp = freopen("/dev/null", "r", stdin);
    fp = freopen("/dev/null", "w", stdout);
    fp = freopen("/dev/null", "w", stderr);
    /* ignoring handle */

    if (! no_varrunpid) {
        my_pid = getpid();
        snprintf(b, sizeof(b), "/var/run/%s.pid", name);
        fp = fopen(b, "w+");
        if (fp) {
            snprintf(b, sizeof(b), "%d\n", my_pid);
            fputs(b, fp);
            fclose(fp);
        } else if (verbose)
            cl_print(LOG_WARNING, "Unable to open %s to put my pid(%d) in\n",
                     b, my_pid);
    }
}

/* Read pairs of numbers from command line (comma (or (single) space)
 * separated list) or from stdin or file (one or two per line, comma
 * separated list or space separated list). Numbers assumed to be decimal.
 * Returns 0 if ok, or 1 if error. The 'arr' has an element with zero
 * fields (frequency and duration_ms) as a terminator */
static int
build_arr(FILE * fp, const char * inp, struct elem_t * arr, int max_arr_len)
{
    int in_len, k, j, m, n, fr, got_freq, neg;
    unsigned int u;
    const char * lcp;
    const char * allowp;

    if ((NULL == arr) || (max_arr_len < 1))
        return 1;
    allowp = "-0123456789kKmMgGiIhHzZ ,\t";
    if (fp) {        /* read from file or stdin */
        char line[512];
        int off = 0;

        for (j = 0, fr = 0; j < 512; ++j) {
            if (NULL == fgets(line, sizeof(line), fp))
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
            k = strspn(lcp, allowp);
            if ((k < in_len) && ('#' != lcp[k])) {
                pr2serr("%s: syntax error at line %d, pos %d\n", __func__,
                        j + 1, m + k + 1);
                return 1;
            }
            for (k = 0; k < 1024; ++k) {
                if ('#' == *lcp) {
                    --k;
                    break;
                }
                neg = 0;
                if ('-' == *lcp) {
                    neg = 1;
                    ++lcp;
                }
                if ((n = fr_get_num(lcp)) == -1) {
                    pr2serr("%s: unable to decode %s as number\n", __func__,
                            lcp);
                    return 1;
                } else
                    u = (unsigned int)n;
                if ((off + k) >= max_arr_len) {
                    pr2serr("%s: array length exceeded\n", __func__);
                    return 1;
                }
                if (neg) {
                    if ((u - 1) > INT_MAX) {
                        pr2serr("%s: number too small: -%u\n", __func__, u);
                        return 1;
                    }
                    m = -((int)(u - 1));
                    --m;
                } else { /* non-negative */
                    if (u > INT_MAX) {
                        pr2serr("%s: number too large: %u\n", __func__, u);
                        return 1;
                    }
                    m = (int)u;
                }
                got_freq = 0;
                if (fr) {
                    fr = 0;
                    arr[off + k].duration_ms = m;
                } else {
                    fr = 1;
                    arr[off + k].frequency = m;
                    ++got_freq;
                    --k;
                }
                lcp = strpbrk(lcp, " ,\t");
                if (NULL == lcp)
                    break;
                lcp += strspn(lcp, " ,\t");
                if ('\0' == *lcp)
                    break;
            }
            off += (k + 1);
        }
        if (fr) {
            pr2serr("%s: got frequency but missing duration\n", __func__);
            return 1;
        }
        if (off < max_arr_len) {
            arr[off].frequency = 0;
            arr[off].duration_ms = 0;
        }
    } else if (inp) {        /* list of numbers on command line */
        lcp = inp;
        in_len = strlen(inp);
        if (0 == in_len) {
            arr[0].frequency = 0;
            arr[0].duration_ms = 0;
            return 0;
        }
        k = strspn(inp, allowp);
        if (in_len != k) {
            pr2serr("%s: error at pos %d\n", __func__, k + 1);
            return 1;
        }
        for (k = 0, fr = 0; k < max_arr_len; ++k) {
            if ('-' == *lcp) {
                neg = 1;
                ++lcp;
            } else
                neg = 0;
            if ((n = fr_get_num(lcp)) == -1) {
                pr2serr("%s: unable to decode %s as number\n", __func__, lcp);
                return 1;
            } else
                u = (unsigned int)n;
            if (neg) {
                if ((u - 1) > INT_MAX) {
                    pr2serr("%s: number too small: -%u\n", __func__, u);
                    return 1;
                }
                m = -((int)(u - 1));
                --m;
            } else { /* non-negative */
                if (u > INT_MAX) {
                    pr2serr("%s: number too large: %u\n", __func__, u);
                    return 1;
                }
                m = (int)u;
            }
            got_freq = 0;
            if (fr) {
                fr = 0;
                arr[k].duration_ms = m;
            } else {
                fr = 1;
                arr[k].frequency = m;
                ++got_freq;
                --k;
            }
            lcp = strpbrk(lcp, " ,\t");
            if (NULL == lcp)
                break;
            lcp += strspn(lcp, " ,\t");
            if ('\0' == *lcp)
                break;
        }
        if (fr) {
            pr2serr("%s: got frequency but missing duration\n", __func__);
            return 1;
        }
        if (k == max_arr_len) {
            pr2serr("%s: array length exceeded\n", __func__);
            return 1;
        }
        if (k < (max_arr_len - 1)) {
            arr[k + 1].frequency = 0;
            arr[k + 1].duration_ms = 0;
        }
    }
    return 0;
}

/* Since we map a whole page (MAP_SIZE) then that page may already be
 * mapped which means we can skip a munmap() and mmap() call. Returns
 * new or re-used pointer to mmapped page, or returns NULL if problem. */
static void *
check_mmap(int mem_fd, unsigned int wanted_addr, struct mmap_state * msp)
{
    off_t mask_addr;
    bool did_unmap = false;
    void * ommap_ptr = NULL;

    mask_addr = (wanted_addr & ~MAP_MASK);
    if ((! msp->mmap_ok) || (msp->prev_mask_addrp != mask_addr)) {
        if (msp->mmap_ok) {
            if (-1 == munmap(msp->mmap_ptr, MAP_SIZE)) {
                pr2serr("mmap_ptr=%p:\n", msp->mmap_ptr);
                perror("    munmap");
                return NULL;
            } else if (verbose > 2) {
                did_unmap = true;
                ommap_ptr =  msp->mmap_ptr;
            }
        }
        msp->mmap_ptr = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                             mem_fd, mask_addr);
        if ((void *)-1 == msp->mmap_ptr) {
            msp->mmap_ok = false;
            pr2serr("addr=0x%x, mask_addr=0x%lx :\n", wanted_addr, mask_addr);
            perror("    mmap");
            return NULL;
        }
        msp->mmap_ok = true;
        msp->prev_mask_addrp = mask_addr;
        if (verbose > 2) {
            if (did_unmap) {
                if (ommap_ptr == msp->mmap_ptr)
                    pr2serr("munmap+mmap() mask_addr=0x%lx; old,new "
                            "mmap_ptrs=%p,same\n", mask_addr, ommap_ptr);
                else
                    pr2serr("munmap+mmap() mask_addr=0x%lx; old,new "
                            "mmap_ptrs=%p,%p\n", mask_addr, ommap_ptr,
                            msp->mmap_ptr);
            } else
                pr2serr("mmap() mask_addr=0x%lx, mmap_ptr=%p\n",
                        mask_addr, msp->mmap_ptr);
        }
    }
    return msp->mmap_ptr;
}

static volatile unsigned int *
get_mmp(int mem_fd, unsigned int wanted_addr, struct mmap_state * msp)
{
    void * mmap_ptr;

    mmap_ptr = check_mmap(mem_fd, wanted_addr, msp);
    if (NULL == mmap_ptr)
        return NULL;
    return mmp_add(mmap_ptr, wanted_addr & MAP_MASK);
}

/* Looks for match of TIO name (e.g. TIOB3) in table_arr. If found returns
 * index (>= 0). If not found or error, return -1 . */
static int
find_table_index(const char * cp)
{
    char b[64];
    int k, len;
    const struct table_io_t * tp;

    if ((! cp) || ((len = strlen(cp)) >= (int)sizeof(b))) {
        pr2serr("%s: bad TIO, too long?\n", __func__);
        return -1;
    }
    for (k = 0; k < len; ++k)
        b[k] = toupper(cp[k]);
    b[k] = '\0';
    if ('T' == b[0]) {
        for (tp = table_arr, k = 0; tp->tio_name; ++tp, ++k) {
            if (0 == strcmp(b, tp->tio_name))
                return k;
        }
    } else {
        pr2serr("%s: expect a name that starts with 'T'\n", __func__);
        return -1;
   }
   return -1;
}


int
main(int argc, char * argv[])
{
    int mem_fd, k, n, opt, tc_clk_ena, want_clk, mps, peri_id;
    unsigned int rc, rms, prev_rms, new_cmr, r, pmc_s, pmc_ed;
    int got_div = 0;
    int have_continuous = 0;
    int pcr_gckdiv = 0;
    int res = 1;
    int tcclks = TC_CMR_TCCLKS_DEF;
    bool tcclks_given = false;
    int dummy = 0;
    int do_daemon = 0;
    int do_enum = 0;
    int do_help = 0;
    int do_init = 0;
    int show_imr = 0;
    int no_sched = 0;
    int ref_freq = 0;
    int do_uninit = 0;
    int mark = 1;
    int space = 1;
    int ms_invert = 0;
    int ms_given = 0;
    int wpen = 0;
    int wpen_given = 0;
    int t_ind = -1;
    const char * fname = NULL;
    const char * pstring = NULL;
    char * cp;
    char b[16];
    struct elem_t * ep;
    struct table_io_t * tp;
    volatile unsigned int * mmp;
    struct timespec request;
    FILE * input_filep = NULL;
    struct sched_param spr;
    struct mmap_state mstat;
    struct mmap_state * msp;

    msp = &mstat;
    memset(msp, 0, sizeof(mstat));
    mem_fd = -1;
    while ((opt = getopt(argc, argv, "b:c:dDef:hiIm:Mnp:R:uvVw:")) != -1) {
        switch (opt) {
            break;
        case 'b':
            t_ind = find_table_index(optarg);
            if (t_ind < 0) {
                pr2serr("Unable to match given TIO of %s with available "
                        "names.\nTIOA0-5 and TIOB0-5 are the choices\n",
                        optarg);
                return 1;
            }
            break;
        case 'c':
            k = atoi(optarg);
            if ((k < 0) || (k > 7)) {
                pr2serr("'-c' option expects 0 to 7\n");
                return 1;
            }
            tcclks = k;
            tcclks_given = true;
            break;
        case 'd':
            ++dummy;
            break;
        case 'D':
            ++do_daemon;
            break;
        case 'e':
            ++do_enum;
            break;
        case 'f':
            fname = optarg;
            break;
        case 'h':
        case '?':
            ++do_help;
            break;
        case 'i':
            ++do_init;
            break;
        case 'I':
            ++ms_invert;
            break;
        case 'm':       /* mark,space ratio for TIOA*, inverted for TIOB* */
            cp = strchr(optarg, ',');
            if ((NULL == cp) || (optarg == cp) ||
                ((cp - optarg) >= (int)sizeof(b))) {
                pr2serr("-m expects two numbers separated by a comma\n");
                return 1;
            }
            memcpy(b, optarg, cp - optarg);
            b[cp - optarg] = '\0';
            mark = atoi(b);
            space = atoi(cp + 1);
            if ((mark < 1) || (space < 1)) {
                pr2serr("-m expects both numbers to be greater than zero\n");
                return 1;
            }
            ++ms_given;
            break;
        case 'M':
            ++show_imr;
            break;
        case 'n':
            ++no_sched;
            break;
        case 'p':
            pstring = optarg;
            break;
        case 'R':
            k = fr_get_num(optarg);
            if (k <= 0) {
                pr2serr("-R expects positive frequency for reference\n");
                return 1;
            }
            ref_freq = k;
            break;
        case 'u':
            ++do_uninit;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            pr2serr("version: %s\n", version_str);
            return 0;
        case 'w':
            if (0 == strcmp("-1", optarg))
                wpen = -1;
            else {
                wpen = atoi(optarg);
                if ((wpen < 0) || (wpen > 1)) {
                    pr2serr("expect argument to '-w' to be 0, 1 or -1\n");
                    return 1;
                }
            }
            ++wpen_given;
            break;
        default:
            pr2serr("unrecognised option code 0x%x ??\n", opt);
            usage(1);
            return 1;
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                pr2serr("Unexpected extra argument: %s\n", argv[optind]);
            usage(1);
            return 1;
        }
    }

    if (do_help) {
        usage(do_help);
        return 0;
    }
    if (do_enum) {
        const struct value_str_t * vsp;

        printf("Allowable TIO acronyms:\n");
        for (tp = table_arr; tp->tio_name; ++tp)
            printf("    %s\n", tp->tio_name);

        printf("\nTCCLKS values\n");
        for (vsp = tcclks_arr; vsp->val >= 0; ++vsp)
            printf("    %d: %s\n", vsp->val, vsp->str);
        return 0;
    }

    if ((verbose > 3) && ms_given)
        pr2serr("-m option decodes mark=%d and space=%d\n", mark, space);

    if (fname) {
        if ((1 == strlen(fname)) && ('-' == fname[0]))
            input_filep = stdin;
        else {
            input_filep = fopen(fname, "r");
            if (NULL == input_filep) {
                pr2serr("failed to open %s:  ", fname);
                perror("fopen()");
                return 1;
            }
        }
    }

    if (fname || pstring) {
        n = build_arr(input_filep, pstring, elem_arr, MAX_ELEMS - 1);
        if (n) {
            if (fname)
                pr2serr("unable to decode contents of FN: %s\n", fname);
            else
                pr2serr("unable to decode '-p F1,D1[,F2,D2...]'\n");
            return 1;
        }
    }

    if (dummy || (verbose > 1)) {
        printf("build_arr after command line input processing:\n");
        for (k = 0; ((0 != elem_arr[k].frequency) ||
                     (0 != elem_arr[k].duration_ms)); ++k) {
            ep = elem_arr + k;
            if (ep->frequency > 0)
                printf("    frequency: %d Hz,", ep->frequency);
            else if (ep->frequency < 0)
                printf("    period: %d ms,", -ep->frequency);
            else
                printf("    line is low,");
            if (-1 == ep->duration_ms)
                printf("\tduration: continual\n");
            else  if (ep->duration_ms > 0)
                printf("\tduration: %d ms\n", ep->duration_ms);
            else
                printf("\tduration: %d is bad\n", ep->duration_ms);
        }
        if (dummy)
            return 0;
    }

    if ((0 == elem_arr[0].frequency) && (0 == elem_arr[0].duration_ms) &&
        (0 == do_init) && (0 == do_uninit) && (0 == wpen_given) &&
        (0 == show_imr)) {
        printf("Nothing to do so exit. Add '-h' for usage.\n");
        return 1;
    }

    if (t_ind < 0) {
        pr2serr("'-b TIO' option is required!\n");
        if ((0 == do_init) && (0 == do_uninit) && (0 == wpen_given) &&
            (0 == show_imr)) {
            pr2serr("\n");
            usage(1);
        } else
            pr2serr("Add '-h' for usage.\n");
        return 1;
    } else {
        tp = table_arr + t_ind;
        if (verbose > 2)
            pr2serr("t_ind=%d, entry points to %s, TCB%d\n", t_ind,
                    tp->tio_name, tp->tcb);
    }
    peri_id = (0 == tp->tcb) ? SAMA5D2_PERI_ID_TCB0 : SAMA5D2_PERI_ID_TCB1;

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose)
        printf("open(" DEV_MEM ", O_RDWR | O_SYNC) okay\n");

    if (wpen_given) {
        if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_wpmr, msp))))
            goto clean_up;
        if (-1 == wpen) {
            r = *mmp;
            printf("Write protect mode: %sabled\n", ((r & 1) ? "EN" : "DIS"));
        } else if ((0 == wpen) || (1 == wpen))
            *mmp = (A5D2_TCB_WPKEY << 8) | wpen;
        res = 0;
        goto clean_up;
    }

    if (show_imr) {
        if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_imr, msp))))
            goto clean_up;
        r = *mmp;
        printf("TC interrupt mask register=0x%x\n", r);
        res = 0;
        goto clean_up;
    }

    if (do_daemon)
        cl_daemonize("a5d2_tc_freq", 1, 1, verbose);

    if (0 == no_sched) {
        k = sched_get_priority_min(SCHED_FIFO);
        if (k < 0)
            cl_print(LOG_ERR, "sched_get_priority_min: %s\n",
                     strerror(errno));
        else {
            spr.sched_priority = k;
            if (sched_setscheduler(0, SCHED_FIFO, &spr) < 0)
                cl_print(LOG_ERR, "sched_setscheduler: %s\n",
                         strerror(errno));
        }
    }

    for (k = 0; (elem_arr[k].frequency || elem_arr[k].duration_ms); ++k) {
        if (-1 == elem_arr[k].duration_ms) {
            ++have_continuous;
            break;
        }
    }
    if (do_init || have_continuous) {
        if (verbose > 1)
            pr2serr("initializing TC\n");
        if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ccr, msp))))
            goto clean_up;
        *mmp = TC_CCR_CLKDIS;
        if (verbose > 1)
            pr2serr("wrote: TC_CCR addr=0x%x, val=0x%x [CLKDIS]\n",
                    tp->tc_ccr, TC_CCR_CLKDIS);

        if (peri_id < 32) {
            pmc_s = PMC_PCSR0;
            pmc_ed = PMC_PCER0;
            r = (1 << peri_id);
        } else {
            pmc_s = PMC_PCSR1;
            pmc_ed = PMC_PCER1;
            r = (1 << (peri_id - 32));
        }
        if (NULL == ((mmp = get_mmp(mem_fd, pmc_s, msp))))
            goto clean_up;
        if (verbose > 2)
            pr2serr("read: PMC_PCSR%d addr=0x%x, val=0x%x\n",
                    ((peri_id < 32) ? 0 : 1), pmc_s, *mmp);
        if (0 == (*mmp & r)) {
            if (verbose > 2)
                pr2serr("    and initializing PMC\n");
            if (NULL == ((mmp = get_mmp(mem_fd, pmc_ed, msp))))
                goto clean_up;
            *mmp = r;
            if (verbose > 1)
                pr2serr("wrote: PMC_PCER%d addr=0x%x, val=0x%x\n",
                        ((peri_id < 32) ? 0 : 1), pmc_ed, r);
            if (NULL == ((mmp = get_mmp(mem_fd, PMC_PCR, msp))))
                goto clean_up;
            /* write a read cmd for given bn (in the PID field) */
            *mmp = peri_id;
            /* now read back result, DIV field should be populated */
            r = *mmp;
            pcr_gckdiv = (PMC_PCR_GCKDIV_MSK & r) >> PMC_PCR_GCKDIV_SHIFT;
            ++got_div;
            if (verbose)
                pr2serr("read PMC_PCR got 0x%x, gckdiv=%d\n", r, pcr_gckdiv);
        }
    }

    for (k = 0, prev_rms = 0, tc_clk_ena = 0;
         (elem_arr[k].frequency || elem_arr[k].duration_ms); ++k) {
        ep = elem_arr + k;
        if (0 != ep->frequency) {
            if (ep->frequency < 0) {
                /* period = abs(ep->frequency) / 1000.0 seconds */
                if (ep->frequency <= -131072000) {
                    pr2serr("frequency[%d]=%d represent a period of %u "
                            "seconds which\nis too large (131071.999 seconds "
                            "is the limit)\n", k + 1, ep->frequency,
                            (-ep->frequency) / 1000);
                    goto clean_up;
                }
                else if ((-ep->frequency) > (INT_MAX / TIMER_CLOCK5))
                    rc = ((-ep->frequency) / 1000) * TIMER_CLOCK5;
                else
                    rc = (-ep->frequency) * TIMER_CLOCK5 / 1000 ;
                /* latter gives rc = 32768 for "freq" of -1000
                 * (i.e. a period of 1000 ms (1 second) */
                want_clk = 5;
                if (verbose > 1)
                    pr2serr("slow clocking from TIMER_CLOCK5 assumed to be %d "
                            "Hz\n", TIMER_CLOCK5);
            } else {    /* ep->frequency > 0 so it is an actual frequency */
                if (! got_div) {
                    if (NULL == ((mmp = get_mmp(mem_fd, PMC_PCR, msp))))
                        goto clean_up;
                    /* write a read cmd for given bn (in the PID field) */
                    *mmp = peri_id;
                    /* now read back result, DIV field should be populated */
                    r = *mmp;
                    pcr_gckdiv = (PMC_PCR_GCKDIV_MSK & r) >>
                                 PMC_PCR_GCKDIV_SHIFT;
                    ++got_div;
                    if (verbose)
                        pr2serr("read PMC_PCR: 0x%x, gckdiv=%d\n", r,
                                pcr_gckdiv);
                }
                if (ref_freq)
                    tc_tclock1 = ref_freq;
                else if (pcr_gckdiv > 0)
                    tc_tclock1 /= (pcr_gckdiv + 1);

                /* since the SAMA5D2 has 32 bit counters just divide down the
                 * highest speed clock (master_ clock/2: 83 MHz) */
                rc = tc_tclock1 / ep->frequency;
                if (rc < 2) {
                    pr2serr("frequency[%d]=%d too high, limit: %d Hz "
                            "(CLK1)\n", k + 1, ep->frequency, tc_tclock1 / 2);
                    goto clean_up;
                }
                want_clk = 1;       /* 1 Hz to 41.5 MHz */
                if (verbose > 1)
                    pr2serr("clocking from TIMER_CLOCK1 assumed to be %d "
                            "Hz\n", tc_tclock1);
            }
            // Check Channel Mode Register (TC_CMR), change if needed
            switch (want_clk) {
            case 1:
                new_cmr = TC_CMR_VAL_CLK1;
                break;
            case 5:
                new_cmr = TC_CMR_VAL_CLK5;
                break;
            default:
                pr2serr("frequency[%d]=%d, bad want_clk=%d\n", k + 1,
                        ep->frequency, want_clk);
                goto clean_up;
            }
            if (tcclks_given)
                new_cmr = tcclks;       /* override want_clk, think about */
            new_cmr |= ((tp->is_tioa == ms_invert) ? TC_CMR_MS_INV_MASK :
                                                     TC_CMR_MS_MASK);
            if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_cmr, msp))))
                goto clean_up;
            if (new_cmr != *mmp) {
                *mmp = new_cmr;
                if (verbose > 1)
                    pr2serr("wrote: TC_CMR addr=0x%x, val=0x%x\n", tp->tc_cmr,
                            *mmp);
            } else if (verbose > 2)
                pr2serr(" did not write TC_CMR addr=0x%x because val=0x%x "
                        "already\n", tp->tc_cmr, *mmp);
            // Caclculate the mark space ratio in order to set RA and RB
            mps = mark + space;
            if (rc > USHRT_MAX) {
                if ((mps > SHRT_MAX) || ((rc / mps) < 100)) {
                    if (mark >= space)
                        rms = (rc * space) / mps;
                    else {
                        /* (rc * space) may overflow so use mark */
                        rms = (rc * mark) / mps;
                        rms = rc - rms;
                    }
                } else {
                    if (mark >= space)
                        rms = (rc / mps) * space;
                    else {
                        rms = (rc / mps) * mark;
                        rms = rc - rms;
                    }
                }
            } else {
                if (mark >= space)
                    rms = (rc * space) / mps;
                else {
                    rms = (rc * mark) / mps;
                    rms = rc - rms;
                }
            }
            if (0 == rms) {
                pr2serr("mark+space too large, please reduce\n");
                goto clean_up;
            }
            // rms = rc / 2;           // use 1:1 mark space ratio
            // rms = rc * 1 / 5;       // use 4:1 mark space ratio
            // rms = rc * 4 / 5;       // use 1:4 mark space ratio
            if (rc > prev_rms) {
                // set up RC prior to RA and RB
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_rc, msp))))
                    goto clean_up;
                *mmp = rc;
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ra, msp))))
                    goto clean_up;
                *mmp = rms;
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_rb, msp))))
                    goto clean_up;
                *mmp = rc - rms;
                if (verbose > 1) {
                    pr2serr("TC_RC,A,B addr=0x%x,%x,%x val=%u,%u,%u",
                            tp->tc_rc, tp->tc_ra, tp->tc_rb,
                            rc, rms, rc - rms);
                    if (verbose > 2)
                        pr2serr("\n       [0x%x,0x%x,0x%x]\n", rc, rms,
                                rc - rms);
                    else
                        pr2serr("\n");
                }
            } else {
                // set up RA and RB prior to RC
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ra, msp))))
                    goto clean_up;
                *mmp = rms;
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_rb, msp))))
                    goto clean_up;
                *mmp = rc - rms;
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_rc, msp))))
                    goto clean_up;
                *mmp = rc;
                if (verbose > 1) {
                    pr2serr("TC_RA,B,C addr=0x%x,0x%x,0x%x val=%u,%u,%u",
                            tp->tc_ra, tp->tc_rb, tp->tc_rc, rms, rc - rms,
                            rc);
                    if (verbose > 2)
                        pr2serr("\n       [0x%x,0x%x,0x%x]\n", rms, rc - rms,
                                rc);
                    else
                        pr2serr("\n");
                }
            }
            prev_rms = (rms >= (rc - rms)) ? rms : (rc - rms);
            if (0 == tc_clk_ena) {
                // everything should be set up, start it ...
                if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ccr, msp))))
                    goto clean_up;
                *mmp = TC_CCR_SWTRG | TC_CCR_CLKEN;
                if (verbose > 1)
                    pr2serr("wrote: TC_CCR addr=0x%x, val=0x%x [SWTRG | "
                            "CLKEN]\n", tp->tc_ccr,
                            TC_CCR_SWTRG | TC_CCR_CLKEN);
                tc_clk_ena = 1;
            }
        } else if (tc_clk_ena) {
            // drive line low
            if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ccr, msp))))
                goto clean_up;
            *mmp = TC_CCR_SWTRG | TC_CCR_CLKDIS;
            if (verbose > 1)
                pr2serr("wrote: TC_CCR addr=0x%x, val=0x%x [SWTRG | "
                        "CLKDIS]\n", tp->tc_ccr,
                        TC_CCR_SWTRG | TC_CCR_CLKDIS);
            tc_clk_ena = 0;
        }

        if (ep->duration_ms < 0)
            break;      /* leave loop when continuous detected */
        else if (ep->duration_ms > 0) {
            request.tv_sec = ep->duration_ms / 1000;
            request.tv_nsec = (ep->duration_ms % 1000) * 1000000;
            if (nanosleep(&request, NULL) < 0) {
                perror("nanosleep");
                goto clean_up;
            }
            if (verbose > 1)
                pr2serr("slept for %d milliseconds\n", ep->duration_ms);
        }
    }

    if ((tc_clk_ena) && (0 == have_continuous)) {
        // drive line low
        if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ccr, msp))))
            goto clean_up;
        *mmp = TC_CCR_SWTRG | TC_CCR_CLKDIS;
        if (verbose > 1)
            pr2serr("wrote: TC_CCR addr=0x%x, val=0x%x [SWTRG | CLKDIS]\n",
                    tp->tc_ccr, TC_CCR_SWTRG | TC_CCR_CLKDIS);
        tc_clk_ena = 0;
    }

    if (do_uninit) {
        // disable clock within TC
        if (NULL == ((mmp = get_mmp(mem_fd, tp->tc_ccr, msp))))
            goto clean_up;
        *mmp = TC_CCR_CLKDIS;
        if (verbose > 1)
            pr2serr("wrote: TC_CCR addr=0x%x, val=0x%x [CLKDIS]\n",
                    tp->tc_ccr, TC_CCR_CLKDIS);
    }
    res = 0;

clean_up:
    if (msp->mmap_ok) {
        if (-1 == munmap(mstat.mmap_ptr, MAP_SIZE)) {
            pr2serr("mmap_ptr=%p:\n", mstat.mmap_ptr);
            perror("    munmap");
            res = 1;
        } else if (verbose > 2)
            pr2serr("trailing munmap() ok, mmap_ptr=%p\n", mstat.mmap_ptr);
    }
    if (mem_fd >= 0)
        close(mem_fd);
    return res;
}
