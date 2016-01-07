/*
 * Copyright (c) 2012-2013 Douglas Gilbert.
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
 * a5d3_tc_freq.c
 *
 * Utility to produce the given frequencies for the given durations.
 * Uses the SAMA5D3 timer-counter (TC) unit to generate the
 * frequencies on PIO lines corresponding to TIOA0-5 and TIOB0-5.
 * The SAMA5D3 has two TC blocks, each with 3 TCs; TCB0 contains TC0, TC1,
 * and TC2 while TCB1 contains TC3, TC4 and TC4.
 * Note the Linux kernel uses TC0 for internal purposes.
 *
 ****************************************************************/


#define _XOPEN_SOURCE 500
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
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


static const char * version_str = "1.01 20130723";

#define MAX_ELEMS 512
#define DEV_MEM "/dev/mem"
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)

/* On the SAMA5D3 each TCB has a separate peripheral identifier:
 * TCB0 is 26 and TCB1 is 27. Since the Linux kernel uses TC0 which is
 * in TCB0 then it should be enabled. */
#define PMC_PCER0  0xfffffc10   /* peripheral clock enable, reg 0 */
#define PMC_PCDR0  0xfffffc14   /* peripheral clock disable, reg 0 */
#define PMC_PCSR0  0xfffffc18   /* peripheral clock status, reg 0 */
#define PMC_PCR    0xfffffd0c   /* peripheral control register */
                                /* - for peripheral divider */

#define SAMA5D3_PERI_ID_TCB0 26 /* contains TC0, TC1 and TC2 */
#define SAMA5D3_PERI_ID_TCB1 27 /* contains TC3, TC4 and TC5 */

// wave=1, wavesel=2, T_CLK1 EEVT=1
#define TC_CMR_VAL_CLK1  0x0000c400
// wave=1, wavesel=2, T_CLK5 EEVT=1
#define TC_CMR_VAL_CLK5  0x0000c404

// BSWTRG=1 BCPC=1 BCPB=2, ASWTRG=2 ACPC=2 ACPA=1 : TIOA? leads with mark
#define TC_CMR_MS_MASK  0x46890000
// BSWTRG=2 BCPC=2 BCPB=1, ASWTRG=1 ACPC=1 ACPA=2 : TIOA? leads with space
#define TC_CMR_MS_INV_MASK  0x89460000

/* TIMER_CLOCK1 is MCK/2, MCK/4, MCK/8 or MCK/16 . Which one depends on the
 * DIV field PMC_PCR. Note that DIV field is common to all TCs in a TCB. */
#define TIMER_CLOCK1 66000000 /* Assume MCK=132 MHz, and DIV=0 (div by 2) */
#define TIMER_CLOCK5 32768

#define A5D3_TCB_WPKEY 0x54494D  /* "TIM" in ASCII */


/* N.B. the SAMA5D3 Timer Counter unit has 32 bit counters. Some earlier
 * members of the AT91 family had 16 bit counters. */

struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};

/* when both members are zero then end of elem_arr */
struct elem_t {
    int frequency;      /* > 0 then Hz; < 0 then period in ms; 0 then low */
    int duration_ms;    /* > 0 then duration in ms; -1 then continual */
};

struct table_io_t {
    char bank;                  /* which PIO: 'A' to 'E' */
    char bn;                    /* bit within PIO: 0 to 31 */
    char peri;                  /* 'A', 'B', 'C' or '\0' */
    char tcb;                   /* 0 for TCB0 (TC0, TC1, RC2) else 1 */
    const char * tio_name;
    unsigned int tc_ccr;
    unsigned int tc_cmr;
    unsigned int tc_ra;
    unsigned int tc_rb;
    unsigned int tc_rc;
    unsigned int tc_imr;
    unsigned int tc_wpmr;       /* one write protect mode reg per tcb */
    int is_tioa;
};


static struct elem_t elem_arr[MAX_ELEMS];

/* The following PIO registers are indexed by the bank ('A' for 0, etc) */
static unsigned int pio_per[] = {0xfffff200, 0xfffff400, 0xfffff600,
                0xfffff800, 0xfffffa00};   /* PIO Enable Register */
static unsigned int pio_pdr[] = {0xfffff204, 0xfffff404, 0xfffff604,
                0xfffff804, 0xfffffa04};   /* PIO Disable Register */
static unsigned int pio_abcdsr1[] = {0xfffff270, 0xfffff470, 0xfffff670,
                0xfffff870, 0xfffffa70}; /* Peripheral select 1 */
static unsigned int pio_abcdsr2[] = {0xfffff274, 0xfffff474, 0xfffff674,
                0xfffff874, 0xfffffa74}; /* Peripheral select 2 */

/* settings for TIOA0-5 and TIOB0-5. */
static struct table_io_t table_arr[] = {
    /* TCB0: */
    {'D', 5, 'B', 0, "TIOA0", 0xf0010000, 0xf0010004, 0xf0010014,
     0xf0010018, 0xf001001c, 0xf001002c, 0xf00100e4, 1},  /* PD5 peri_B */
    {'D', 6, 'B', 0, "TIOB0", 0xf0010000, 0xf0010004, 0xf0010014,
     0xf0010018, 0xf001001c, 0xf001002c, 0xf00100e4, 0},  /* PD6 peri_B */
    {'C', 12, 'B', 0, "TIOA1", 0xf0010040, 0xf0010044, 0xf0010054,
     0xf0010058, 0xf001005c, 0xf001006c, 0xf00100e4, 1},  /* PC12 peri_B */
    {'C', 13, 'B', 0, "TIOB1", 0xf0010040, 0xf0010044, 0xf0010054,
     0xf0010058, 0xf001005c, 0xf001006c, 0xf00100e4, 0},  /* PC13 peri_B */
    {'E', 27, 'B', 0, "TIOA2", 0xf0010080, 0xf0010084, 0xf0010094,
     0xf0010098, 0xf001009c, 0xf00100ac, 0xf00100e4, 1},  /* PE27 peri_B */
    {'E', 28, 'B', 0, "TIOB2", 0xf0010080, 0xf0010084, 0xf0010094,
     0xf0010098, 0xf001009c, 0xf00100ac, 0xf00100e4, 0},  /* PE28 peri_B */

    /* TCB1: */
    {'C', 0, 'B', 1, "TIOA3", 0xf8014000, 0xf8014004, 0xf8014014,
     0xf8014018, 0xf801401c, 0xf801402c, 0xf80140e4, 1},  /* PC0 peri_B */
    {'C', 1, 'B', 1, "TIOB3", 0xf8014000, 0xf8014004, 0xf8014014,
     0xf8014018, 0xf801401c, 0xf801402c, 0xf80140e4, 0},  /* PC1 peri_B */
    {'C', 3, 'B', 1, "TIOA4", 0xf8014040, 0xf8014044, 0xf8014054,
     0xf8014058, 0xf801405c, 0xf801406c, 0xf80140e4, 1},  /* PC3 peri_B */
    {'C', 4, 'B', 1, "TIOB4", 0xf8014040, 0xf8014044, 0xf8014054,
     0xf8014058, 0xf801405c, 0xf801406c, 0xf80140e4, 0},  /* PC4 peri_B */
    {'C', 6, 'B', 1, "TIOA5", 0xf8014080, 0xf8014084, 0xf8014094,
     0xf8014098, 0xf801409c, 0xf80140ac, 0xf80140e4, 1},  /* PC6 peri_B */
    {'C', 7, 'B', 1, "TIOB5", 0xf8014080, 0xf8014084, 0xf8014094,
     0xf8014098, 0xf801409c, 0xf80140ac, 0xf80140e4, 0},  /* PC7 peri_B */

    {0, 0, 0, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0},
};

static int tc_tclock1 = TIMER_CLOCK1;   /* may get divided by power of 2 */
static int cl_foreground = 1;

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "a5d3_tc_freq -b PIO_TIO [-d] [-D] [-e] [-f FN] [-h] "
            "[-i] [-I]\n"
            "                    [-m M,S] [-M] [-n] [-N] "
            "[-p F1,D1[,F2,D2...]] [-u]\n"
            "                    [-v] [-V] [-w WPEN]\n"
            "  where:\n"
            "    -b PIO_TIO    either PIO name (e.g. 'PC2') or TIO name "
            "(e.g. 'TIOB3')\n"
            "    -d           dummy mode: decode frequency,duration pairs, "
            "print\n"
            "                 them then exit; ignore PIO_TIO\n"
            "    -D           after initial checks, run as daemon which "
            "exits after\n"
            "                 frequency(s) is produced\n"
            "    -e           enumerate TC associated PIO names with "
            "corresponding\n"
            "                 TIO names then exit\n"
            "    -f FN        obtain input from file FN. A FN of '-' "
            "taken as\n"
            "                 read stdin. If '-f' not given then '-p' "
            "option expected\n"
            "    -h           print usage message\n"
            "    -i           initialize PIO_TIO for frequency output, "
            "(def: assume\n"
            "                 already set up). Line set low prior to "
            "frequency generation\n"
            "    -I           invert levels of mark and space\n"
            "    -m M,S       mark (M) space (S) ratio (def: 1,1), both "
            "M and S\n"
            "                 should be greater than zero\n"
            "    -M           show TC interrupt mask register then exit\n"
            "    -n           no realtime scheduling (def: set "
            "SCHED_FIFO)\n"
            "    -N           no PIO; do not touch PIO line settings\n"
            "    -p F1,D1[,F2,D2...]    one or more frequency duration "
            "pairs; frequency\n"
            "                           in Hz and the duration in "
            "milliseconds\n"
            "    -u           restore PIO_NAM to normal GPIO operation "
            "on exit\n"
            "                 (def: leave low for next frequency "
            "generation)\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n"
            "    -w WPEN      set or show write protect (WP) information "
            "for TCB.\n"
            "                 0 -> disable (def, no WP), 1 -> enable, "
            "-1 -> show\n"
            "                 WP en/disable state. Then in all cases "
            "exit\n\n"
            "Use the timer counter (TC) in the SAMA5D3 SoCs to "
            "generate frequencies.\n"
            "List of frequency (Hz when positive), duration (milliseconds "
            "when positive)\npairs can be given on the command line ('-p') "
            "or in a file ('-f'). Use a\nfrequency of 0 for a delay (line "
            "put at space level (usually low: 0\nvolts)). The first time "
            "this utility is called option '-i' probably should\nbe given "
            "to initialize the TC. Duration of -1 for continuous (exit and\n"
            "maintain), assumes '-i'. A negative frequency is treated as "
            "a period in\nmilliseconds, so -1 gives a period of 1 "
            "millisecond (i.e. 1000 Hz) while\n-2500, for example, gives a "
            "period of 2.5 seconds. The maximum period is\n131071.999 "
            "seconds, corresponding to -131071999 . The maximum "
            "frequency\ndepends on the master clock (MCK) and whether "
            "TCB dividers are set in the\nPMC; typically either 16.5 or 33 "
            "MHz.\n");
}



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


/* Returns > 0 on success, 0 on error */
unsigned int
get_num(const char * buf, int len)
{
    char b[68];
    int res;
    unsigned int unum;

    if (len > (int)(sizeof(b) - 4)) {
        fprintf(stderr, "get_num: len=%d too large for a number\n",
                len);
        return 0;       /* no good return, 0 is least harmful */
    }
    memcpy(b, buf, len);
    b[len] = '\0';
    res = sscanf(b, "%u", &unum);
    if (1 != res) {
        fprintf(stderr, "get_num: could not decode number\n");
        return 0;
    }
    return unum;
}

/* Read pairs of numbers from command line (comma (or (single) space)
 * separated list) or from stdin or file (one or two per line, comma
 * separated list or space separated list). Numbers assumed to be decimal.
 * Returns 0 if ok, or 1 if error. The 'arr' has an element with zero
 * fields (frequency and duration_ms) as a terminator */
static int
build_arr(FILE * fp, const char * inp, struct elem_t * arr, int max_arr_len)
{
    int in_len, k, j, m, fr, got_freq, neg;
    unsigned int u;
    const char * lcp;
    const char * allowp;

    if ((NULL == arr) || (max_arr_len < 1))
        return 1;
    allowp = "-0123456789 ,\t";
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
                fprintf(stderr, "build_arr: syntax error at "
                        "line %d, pos %d\n", j + 1, m + k + 1);
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
                u = get_num(lcp, strlen(lcp));
                if ((off + k) >= max_arr_len) {
                    fprintf(stderr, "build_arr: array length exceeded\n");
                    return 1;
                }
                if (neg) {
                    if ((u - 1) > INT_MAX) {
                        fprintf(stderr, "build_arr: number too small: "
                                "-%u\n", u);
                        return 1;
                    }
                    m = -((int)(u - 1));
                    --m;
                } else { /* non-negative */
                    if (u > INT_MAX) {
                        fprintf(stderr, "build_arr: number too large: "
                                "%u\n", u);
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
            fprintf(stderr, "build_arr: got frequency but missing "
                    "duration\n");
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
            fprintf(stderr, "build_arr: error at pos %d\n", k + 1);
            return 1;
        }
        for (k = 0, fr = 0; k < max_arr_len; ++k) {
            if ('-' == *lcp) {
                neg = 1;
                ++lcp;
            } else
                neg = 0;
            u = get_num(lcp, strlen(lcp));
            if (neg) {
                if ((u - 1) > INT_MAX) {
                    fprintf(stderr, "build_arr: number too small: "
                            "-%u\n", u);
                    return 1;
                }
                m = -((int)(u - 1));
                --m;
            } else { /* non-negative */
                if (u > INT_MAX) {
                    fprintf(stderr, "build_arr: number too large: "
                            "%u\n", u);
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
            fprintf(stderr, "build_arr: got frequency but missing "
                    "duration\n");
            return 1;
        }
        if (k == max_arr_len) {
            fprintf(stderr, "build_arr: array length exceeded\n");
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

    mask_addr = (wanted_addr & ~MAP_MASK);
    if ((0 == msp->mmap_ok) || (msp->prev_mask_addrp != mask_addr)) {
        if (msp->mmap_ok) {
            if (-1 == munmap(msp->mmap_ptr, MAP_SIZE)) {
                fprintf(stderr, "mmap_ptr=%p:\n", msp->mmap_ptr);
                perror("    munmap");
                return NULL;
            } else if (verbose > 2)
                fprintf(stderr, "munmap() ok, mmap_ptr=%p\n", msp->mmap_ptr);
        }
        msp->mmap_ptr = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                             mem_fd, mask_addr);
        if ((void *)-1 == msp->mmap_ptr) {
            msp->mmap_ok = 0;
            fprintf(stderr, "addr=0x%x, mask_addr=0x%lx :\n", wanted_addr,
                    mask_addr);
            perror("    mmap");
            return NULL;
        }
        msp->mmap_ok = 1;
        msp->prev_mask_addrp = mask_addr;
        if (verbose > 2)
            fprintf(stderr, "mmap() ok, mask_addr=0x%lx, mmap_ptr=%p\n",
                    mask_addr, msp->mmap_ptr);
    }
    return msp->mmap_ptr;
}

/* Looks for match of PIO name (e.g. PC2) or TIO name (e.g. TIOB3) in
 * table_arr. If found returns index (>= 0). If not found or error,
 * return -1 . */
static int
find_table_index(const char * cp)
{
    char b[64];
    int k, len, ch;
    int bn = -1;
    const struct table_io_t * tp;

    if ((! cp) || ((len = strlen(cp)) >= (int)sizeof(b))) {
        fprintf(stderr, "find_table_index: bad PIO_TIO, too long?\n");
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
        k = 0;
        if ('P' == b[k])
            ++k;
        ch = b[k];
        if ((ch < 'A') || (ch > 'E')) {
            fprintf(stderr, "find_table_index: badly formed PIO_TIO, "
                    "example PC4, got: %s\n", cp);
            return -1;
        }
        ++k;
        bn = -1;
        bn = atoi(b + k);
            if ((bn < 0) || (bn > 31)) {
                fprintf(stderr, "find_table_index: expect PIO_TIO "
                        "number in 0-31 range, got: %s\n", b + k);
                return -1;
        }
        for (tp = table_arr, k = 0; tp->tio_name; ++tp, ++k) {
            if ((ch == tp->bank) && (bn == tp->bn))
                return k;
        }
   }
   return -1;
}


int
main(int argc, char * argv[])
{
    int mem_fd, k, n, opt, tc_clk_ena, want_clk, mps, bank_ind, peri_id;
    unsigned int rc, rms, prev_rms, new_cmr, ui, r, pio_mask;
    unsigned int abcdsr1_val, abcdsr2_val;
    int got_div = 0;
    int have_continuous = 0;
    int div_2tothe = 0;
    int res = 1;
    int dummy = 0;
    int do_daemon = 0;
    int do_enum = 0;
    int do_init = 0;
    int show_imr = 0;
    int no_pio = 0;
    int no_sched = 0;
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
    void * mmap_ptr = (void *)-1;
    void * ap;
    struct timespec request;
    FILE * input_filep = NULL;
    struct sched_param spr;
    struct mmap_state mstat;

    mem_fd = -1;
    while ((opt = getopt(argc, argv, "b:dDef:hiIm:MnNp:uvVw:")) != -1) {
        switch (opt) {
            break;
        case 'b':
            if (isdigit(optarg[0])) {
                n = atoi(optarg);
                if ((n < 0) || (n > 31)) {
                    fprintf(stderr, "-b expects an argument between 0 and "
                            "31, better use 'PC7', for example\n");
                    return 1;
                }
                for (tp = table_arr, k = 0; tp->tio_name; ++tp, ++k) {
                    if (('C' == tp->bank) && (n == tp->bn))
                        break;
                }
                if (! tp->tio_name) {
                    fprintf(stderr, "PC%d not associated with TIO output\n",
                            n);
                    return 1;
                }
                fprintf(stderr, "Warning: PC%d assumed and matches %s, best "
                        "to use full pio name\n", n, tp->tio_name);
                t_ind = k;
            } else {
                t_ind = find_table_index(optarg);
                if (t_ind < 0) {
                    fprintf(stderr, "Unable to match given PIO_TIO of %s "
                            "with TC associated\nPIO name (e.g. PC3) or TIO "
                            "name (e.g. TIOA4)\n", optarg);
                    return -1;
                }
            }
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
            usage();
            return 0;
        case 'i':
            ++do_init;
            break;
        case 'I':
            ++ms_invert;
            break;
        case 'm':
            cp = strchr(optarg, ',');
            if ((NULL == cp) || (optarg == cp) ||
                ((cp - optarg) >= (int)sizeof(b))) {
                fprintf(stderr, "-m expects two numbers separated by a "
                        "comma\n");
                return 1;
            }
            memcpy(b, optarg, cp - optarg);
            b[cp - optarg] = '\0';
            mark = atoi(b);
            space = atoi(cp + 1);
            if ((mark < 1) || (space < 1)) {
                fprintf(stderr, "-m expects both numbers to be greater "
                        "than zero\n");
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
        case 'N':
            ++no_pio;
            break;
        case 'p':
            pstring = optarg;
            break;
        case 'u':
            ++do_uninit;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            fprintf(stderr, "version: %s\n", version_str);
            return 0;
        case 'w':
            if (0 == strcmp("-1", optarg))
                wpen = -1;
            else {
                wpen = atoi(optarg);
                if ((wpen < 0) || (wpen > 1)) {
                    fprintf(stderr, "expect argument to '-w' to be 0, 1 or "
                            "-1\n");
                    return 1;
                }
            }
            ++wpen_given;
            break;
        default:
            fprintf(stderr, "unrecognised option code 0x%x ??\n", opt);
            usage();
            return 1;
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            usage();
            return 1;
        }
    }

    if (do_enum) {
        for (tp = table_arr; tp->tio_name; ++tp)
            printf("    P%c%d  \t<->\t%s\t[peripheral: %c]\n", tp->bank,
                   (int)tp->bn, tp->tio_name, (int)tp->peri);
        return 0;
    }

    if ((verbose > 3) && ms_given)
        fprintf(stderr, "-m option decodes mark=%d and space=%d\n",
                mark, space);

    if (fname) {
        if ((1 == strlen(fname)) && ('-' == fname[0]))
            input_filep = stdin;
        else {
            input_filep = fopen(fname, "r");
            if (NULL == input_filep) {
                fprintf(stderr, "failed to open %s:  ", fname);
                perror("fopen()");
                return 1;
            }
        }
    }

    if (fname || pstring) {
        n = build_arr(input_filep, pstring, elem_arr, MAX_ELEMS - 1);
        if (n) {
            fprintf(stderr, "build_arr() failed\n");
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
        fprintf(stderr, "'-b PIO_TIO' option is required!\n");
        if ((0 == do_init) && (0 == do_uninit) && (0 == wpen_given) &&
            (0 == show_imr)) {
            fprintf(stderr, "\n");
            usage();
        } else
            fprintf(stderr, "Add '-h' for usage.\n");
        return 1;
    } else
        tp = table_arr + t_ind;
    if ((! tp) || (tp->bank < 'A') || (tp->bn > 31)) {
        fprintf(stderr, "Logic error, tp is NULL or bad\n");
        return 1;
    }
    bank_ind = tp->bank - 'A';
    pio_mask = no_pio ? 0 : (1 << tp->bn);
    peri_id = (0 == tp->tcb) ? SAMA5D3_PERI_ID_TCB0 : SAMA5D3_PERI_ID_TCB1;

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose)
        printf("open(" DEV_MEM ", O_RDWR | O_SYNC) okay\n");

    memset(&mstat, 0, sizeof(mstat));

    if (wpen_given) {
        if (-1 == wpen) {
            mmap_ptr = check_mmap(mem_fd, tp->tc_wpmr, &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (tp->tc_wpmr & MAP_MASK);
            r = *((unsigned long *)ap);
            printf("Write protect mode: %sabled\n",
                   ((r & 1) ? "EN" : "DIS"));
        } else if ((0 == wpen) || (1 == wpen)) {
            mmap_ptr = check_mmap(mem_fd, tp->tc_wpmr, &mstat);
            if (NULL == mmap_ptr)
                return 1;
            ap = (unsigned char *)mmap_ptr + (tp->tc_wpmr & MAP_MASK);
            *((unsigned int *)ap) = (A5D3_TCB_WPKEY << 8) | wpen;
        }
        res = 0;
        goto clean_up;
    }

    if (show_imr) {
        mmap_ptr = check_mmap(mem_fd, tp->tc_imr, &mstat);
        if (NULL == mmap_ptr)
            return 1;
        ap = (unsigned char *)mmap_ptr + (tp->tc_imr & MAP_MASK);
        r = *((unsigned long *)ap);
        printf("TC interrupt mask register=0x%x\n", r);
        res = 0;
        goto clean_up;
    }

    if (do_daemon)
        cl_daemonize("a5d3_tc_freq", 1, 1, verbose);

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
            fprintf(stderr, "initializing TC\n");
        if (NULL == ((mmap_ptr = check_mmap(mem_fd, tp->tc_ccr, &mstat))))
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (tp->tc_ccr & MAP_MASK);
        *((unsigned long *)ap) = 0x2;   /* CLKDIS=1 */
        if (verbose > 1)
            fprintf(stderr, "wrote: TC_CCR addr=0x%x, val=0x%x\n",
                    tp->tc_ccr, 0x2);

        if (0 == no_pio) {
            switch (tp->peri) {
            case 'A':
                abcdsr1_val = 0;
                abcdsr2_val = 0;
                break;
            case 'B':
                abcdsr1_val = 1;
                abcdsr2_val = 0;
                break;
            case 'C':
                abcdsr1_val = 0;
                abcdsr2_val = 1;
                break;
            default:
                fprintf(stderr, "Logic error, tp is bad\n");
                goto clean_up;
            }
            r = pio_abcdsr1[bank_ind];
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, r, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (r & MAP_MASK);
            ui = *((unsigned long *)ap);
            if (!!(ui & pio_mask) != abcdsr1_val) {
                if (ui & pio_mask)
                    ui &= ~pio_mask;
                else
                    ui |= pio_mask;
                *((unsigned long *)ap) = ui;
                if (verbose > 1)
                    fprintf(stderr, "wrote: PIO_ABCDSR1 addr=0x%x, val=0x%x\n",
                            r, ui);
            }
            r = pio_abcdsr2[bank_ind];
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, r, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (r & MAP_MASK);
            ui = *((unsigned long *)ap);
            if (!!(ui & pio_mask) != abcdsr2_val) {
                if (ui & pio_mask)
                    ui &= ~pio_mask;
                else
                    ui |= pio_mask;
                *((unsigned long *)ap) = ui;
                if (verbose > 1)
                    fprintf(stderr, "wrote:  PIO_ABCDSR2 addr=0x%x, "
                            "val=0x%x\n", r, ui);
            }

            r = pio_pdr[bank_ind];
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, r, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (r & MAP_MASK);
            *((unsigned long *)ap) = pio_mask;
            if (verbose > 1)
                fprintf(stderr, "wrote: PIO_PDR addr=0x%x, val=0x%x\n", r,
                        pio_mask);
        }

        if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCSR0, &mstat))))
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (PMC_PCSR0 & MAP_MASK);
        r = (1 << peri_id);     /* assume/know it is < 32 */
        if (0 == (*((unsigned long *)ap) & r)) {
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCER0, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_PCER0 & MAP_MASK);
            *((unsigned long *)ap) = r;
            if (verbose > 1)
                fprintf(stderr, "wrote: PMC_PCER0 addr=0x%x, val=0x%x\n",
                        PMC_PCER0, r);

            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCR, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_PCR & MAP_MASK);
            /* write a read cmd for given bn (in the PID field) */
            *((volatile unsigned long *)ap) = peri_id;
            /* now read back result, DIV field should be populated */
            r = *((volatile unsigned long *)ap);
            div_2tothe = ((r & 0x30000) >> 16);
            ++got_div;
            if (verbose)
                fprintf(stderr, "read PMC_PCR got 0x%x\n", r);
        }
    }

    for (k = 0, prev_rms = 0, tc_clk_ena = 0;
         (elem_arr[k].frequency || elem_arr[k].duration_ms); ++k) {
        ep = elem_arr + k;
        if (0 != ep->frequency) {
            if (ep->frequency < 0) {
                /* period = abs(ep->frequency) / 1000.0 seconds */
                if (ep->frequency <= -131072000) {
                    fprintf(stderr, "frequency[%d]=%d represent a "
                            "period of %u seconds which\nis too large "
                            "(131071.999 seconds is the limit)\n", k + 1,
                            ep->frequency, (-ep->frequency) / 1000);
                    goto clean_up;
                }
                else if ((-ep->frequency) > (INT_MAX / TIMER_CLOCK5))
                    rc = ((-ep->frequency) / 1000) * TIMER_CLOCK5;
                else
                    rc = (-ep->frequency) * TIMER_CLOCK5 / 1000 ;
                /* latter gives rc = 32768 for "freq" of -1000
                 * (i.e. a period of 1000 ms (1 second) */
                want_clk = 5;
            } else {    /* ep->frequency > 0 so it is an actual frequency */
                if (! got_div) {
                    if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCR,
                                                        &mstat))))
                        goto clean_up;
                    ap = (unsigned char *)mmap_ptr + (PMC_PCR & MAP_MASK);
                    /* write a read cmd for given bn (in the PID field) */
                    *((volatile unsigned long *)ap) = peri_id;
                    /* now read back result, DIV field should be populated */
                    r = *((volatile unsigned long *)ap);
                    div_2tothe = ((r & 0x30000) >> 16);
                    ++got_div;
                    if (verbose)
                        fprintf(stderr, "read PMC_PCR got 0x%x\n", r);
                }
                if (div_2tothe)
                    tc_tclock1 /= (1 << div_2tothe);

                /* since the SAMA5D3 has 32 bit counters just divide down the
                 * highest speed clock (master_ clock/2: 66 MHz) */
                rc = tc_tclock1 / ep->frequency;
                if (rc < 2) {
                    fprintf(stderr, "frequency[%d]=%d too high, limit: "
                            "%d Hz (CLK1)\n", k + 1, ep->frequency,
                            tc_tclock1 / 2);
                    goto clean_up;
                }
                want_clk = 1;       /* 1 Hz to 33 MHz */
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
                fprintf(stderr, "frequency[%d]=%d, bad want_clk=%d\n",
                        k + 1, ep->frequency, want_clk);
                goto clean_up;
            }
            new_cmr |= ((tp->is_tioa == ms_invert) ? TC_CMR_MS_INV_MASK :
                                                     TC_CMR_MS_MASK);
            mmap_ptr = check_mmap(mem_fd, tp->tc_cmr, &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (tp->tc_cmr & MAP_MASK);
            if (new_cmr != *((unsigned long *)ap)) {
                *((unsigned long *)ap) = new_cmr;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_CMR addr=0x%x, val=0x%lx\n",
                            tp->tc_cmr, *((unsigned long *)ap));
            }
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
                fprintf(stderr, "mark+space too large, please reduce\n");
                goto clean_up;
            }
            // rms = rc / 2;           // use 1:1 mark space ratio
            // rms = rc * 1 / 5;       // use 4:1 mark space ratio
            // rms = rc * 4 / 5;       // use 1:4 mark space ratio
            if (rc > prev_rms) {
                // set up RC prior to RA and RB
                mmap_ptr = check_mmap(mem_fd, tp->tc_rc, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_rc & MAP_MASK);
                *((unsigned long *)ap) = rc;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RC addr=0x%x, val=0x%x\n",
                            tp->tc_rc, rc);

                mmap_ptr = check_mmap(mem_fd, tp->tc_ra, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_ra & MAP_MASK);
                *((unsigned long *)ap) = rms;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RA addr=0x%x, val=0x%x\n",
                            tp->tc_ra, rms);
                mmap_ptr = check_mmap(mem_fd, tp->tc_rb, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_rb & MAP_MASK);
                *((unsigned long *)ap) = rc - rms;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RB addr=0x%x, val=0x%x\n",
                            tp->tc_rb, rc - rms);
            } else {
                // set up RA and RB prior to RC
                mmap_ptr = check_mmap(mem_fd, tp->tc_ra, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_ra & MAP_MASK);
                *((unsigned long *)ap) = rms;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RA addr=0x%x, val=0x%x\n",
                            tp->tc_ra, rms);
                mmap_ptr = check_mmap(mem_fd, tp->tc_rb, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_rb & MAP_MASK);
                *((unsigned long *)ap) = rc - rms;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RB addr=0x%x, val=0x%x\n",
                            tp->tc_rb, rc - rms);

                mmap_ptr = check_mmap(mem_fd, tp->tc_rc, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_rc & MAP_MASK);
                *((unsigned long *)ap) = rc;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_RC addr=0x%x, val=0x%x\n",
                            tp->tc_rc, rc);
            }
            prev_rms = (rms >= (rc - rms)) ? rms : (rc - rms);
            if (0 == tc_clk_ena) {
                // everything should be set up, start it ...
                mmap_ptr = check_mmap(mem_fd, tp->tc_ccr, &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr + (tp->tc_ccr & MAP_MASK);
                // enable clock (0x1) ORed with software trigger (0x4)
                *((unsigned long *)ap) = 0x5;
                if (verbose > 1)
                    fprintf(stderr, "wrote: TC_CCR addr=0x%x, val=0x%x\n",
                            tp->tc_ccr, 0x5);
                tc_clk_ena = 1;
            }
        } else if (tc_clk_ena) {
            // drive line low
            mmap_ptr = check_mmap(mem_fd, tp->tc_ccr, &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (tp->tc_ccr & MAP_MASK);
            // disable clock (0x1) ORed with software trigger (0x4)
            *((unsigned long *)ap) = 0x6;
            if (verbose > 1)
                fprintf(stderr, "wrote: TC_CCR addr=0x%x, val=0x%x\n",
                        tp->tc_ccr, 0x6);
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
                fprintf(stderr, "slept for %d milliseconds\n",
                        ep->duration_ms);
        }
    }

    if ((tc_clk_ena) && (0 == have_continuous)) {
        // drive line low
        mmap_ptr = check_mmap(mem_fd, tp->tc_ccr, &mstat);
        if (NULL == mmap_ptr)
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (tp->tc_ccr & MAP_MASK);
        // disable clock (0x1) ORed with software trigger (0x4)
        *((unsigned long *)ap) = 0x6;
        if (verbose > 1)
            fprintf(stderr, "wrote: TC_CCR addr=0x%x, val=0x%x\n",
                    tp->tc_ccr, 0x6);
        tc_clk_ena = 0;
    }

    if (do_uninit) {
        // disable clock within TC
        mmap_ptr = check_mmap(mem_fd, tp->tc_ccr, &mstat);
        if (NULL == mmap_ptr)
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (tp->tc_ccr & MAP_MASK);
        *((unsigned long *)ap) = 0x2;
        if (verbose > 1)
            fprintf(stderr, "wrote: TC_CCR addr=0x%x, val=0x%x\n",
                    tp->tc_ccr, 0x2);

        if (0 == no_pio) {
            // Enable normal, generic PIO action on PIO_TIO
            r = pio_per[bank_ind];
            mmap_ptr = check_mmap(mem_fd, r, &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (r & MAP_MASK);
            *((unsigned long *)ap) = pio_mask;
            if (verbose > 1)
                fprintf(stderr, "wrote: PIO_PER addr=0x%x, val=0x%x\n",
                        r, pio_mask);
        }
    }
    res = 0;

clean_up:
    if (mstat.mmap_ok) {
        if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
            fprintf(stderr, "mmap_ptr=%p:\n", mmap_ptr);
            perror("    munmap");
            res = 1;
        } else if (verbose > 2)
            fprintf(stderr, "trailing munmap() ok, mmap_ptr=%p\n", mmap_ptr);
    }
    if (mem_fd >= 0)
        close(mem_fd);
    return res;
}
