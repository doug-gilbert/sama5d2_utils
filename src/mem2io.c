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
 * mem2io.c
 *
 * Utility to read or write multiple 32 bit values from/to the given
 * addresses. Memory maps page size chunks (assumed to be 4 KB) containing
 * an address into this process's ram. If the next address is also in
 * this page then the same map is used, otherwise the previous map is
 * unmapped and a new chunk is mmapped. Time delays can replace an address
 * value pair in a write.
 *
 * Targets the AT91SAM9G20 microcontroller but should be useful on
 * any microcontroller that uses memory-mapped IO in a similar
 * fashion.
 *
 ****************************************************************/

/*******************************************************************
 * Example. Using a file called two_tones.txt containing the following
 * (apart from the " * " at the start of each line):
 *
 * # Set up PB0 (J7.9) to audio frequency suitable for buzzer
 * # using the AT91SAM9G20 Timer Counter (TC).
 *
 * fffdc000 2        # disable TC3 clock [TC_CCR0]
 * fffff604 1        # disable normal gpio on PB0 [PIO_PDR(B)]
 * fffff674 1        # select peripheral B on PB0 [PIO_BSR(B)]
 *
 * fffffc10 4000000  # turn on TC3 clock [PMC_PCER]
 * fffdc004 89c003   # wave=1, wavesel=2, T_CLK4 [TC_CMR0]
                   * # ASWTRG=2 ACPC=2 ACPA=1
 * fffdc01c 7b4      # RC max value (1031250/523 [TC_RC0]
 * fffdc014 3da      # RA for 50% down [TC_RA0]
 * #
 * fffdc000 1        # enable clock [TC_CCR0]
 * fffdc000 4        # software trigger [TC_CCR0]
 * t 200             # time delay of 200 milliseconds
 * fffdc014 292      # RA for 50% down [TC_RA0]
 * fffdc01c 523      # RC max value (1031250/784 [TC_RC0]
 * t 700             # time delay of 700 milliseconds
 * fffdc000 6        # disable clock + soft trig [TC_CCR0]
 * # fffff600 1      # enable normal gpio on PB0 [PIO_PER(B)]
 *
 *
 * then it can be invoked with 'mem2io -w -f two_tones.txt'
 *
 * For reference:
 * fffff400 w <mask>   # enable normal gpio on PA*
 * fffff404 w <mask>   # disable normal gpio on PA*
 * fffff408 r <mask>   # read normal gpio on PA*
 * fffff470 w <mask>   # select peripheral A on PA*
 * fffff474 w <mask>   # select peripheral B on PA*
 *
 * fffff600 w <mask>   # enable normal gpio on PB*
 * fffff800 w <mask>   # enable normal gpio on PC*
 *
 ****************************************************************/


#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

// #include <sys/ioctl.h>


static const char * version_str = "1.08 20160104";

#define MAJOR_TYP_READ 1
#define MAJOR_TYP_WRITE 2
#define MAX_ELEMS 256
#define ELEM_TYP_NULL 0
#define ELEM_TYP_READ MAJOR_TYP_READ
#define ELEM_TYP_WRITE MAJOR_TYP_WRITE
#define ELEM_TYP_WAIT_MS 3
#define DEF_MIN_ADDR 0xf0000000
#define DEV_MEM "/dev/mem"
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)

struct elem_t {
    int typ;
    unsigned int addr;
    unsigned int val;
};

static struct elem_t elem_arr[MAX_ELEMS];
static unsigned int min_addr = DEF_MIN_ADDR;
static int force_nm4 = 0;

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "mem2io [-d] [-f <file>] [-F] [-h] [-i X1[,X2...]] "
            "[-m <addr>]\n"
            "              [-M <mask>] [-q] [-r] [-s <shift_r>] [-v] [-V] "
            "[-w]\n"
            "  where:\n"
            "    -d           dummy mode: decode input, print it then "
            "exit, no memory IO\n"
            "    -f <file>    obtain input from <file>. <file> of '-' "
            "taken as\n"
            "                 read stdin. If '-f' not given then '-i' "
            "option expected\n"
            "    -F           force non-modulo 4 addresses to be accepted "
            "(def: require\n"
            "                 modulo 4 addresses)\n"
            "    -h           print usage message\n"
            "    -i X1[,X2...]    with '-r' a list of addresses in hex; "
            "with '-w'\n"
            "                     a list of address,value pairs in hex. "
            "A time\n"
            "                     delay can replace a pair with 't' or "
            "'T' in the\n"
            "                     first position followed by the delay in "
            "milliseconds\n"
            "    -m <addr>    minimum address (in hex) accepted (def: "
            "f000 0000)\n"
            "    -M <mask>    32 bit mask (in hex) and-ed to X1, result to "
            "stdout\n"
            "                 If result non-zero, exit status true(0), else "
            "false(1)\n"
            "    -q           quiet: suppress '-M <mask>' output to stdout\n"
            "    -r           Uses addresses to read from corresponding "
            "memory\n"
            "                 locations. Data read sent to stdout in hex, "
            "one line\n"
            "                 per address\n"
            "    -s <shift_r>    shift the output from the '-M <mask>' "
            "option\n"
            "                 <shift_r> bits to the right; can be 0 to 31 "
            "(def: 0)\n"
            "    -v           increase verbosity (multiple times for more)\n"
            "    -V           print version string then exit\n"
            "    -w           for each address,value pair writes value to "
            "corresponding\n"
            "                 address. If address is 't' or 'T', value is "
            "delay in ms\n\n"
            "Designed for systems with memory mapped IO. Requires superuser "
            "permissions.\n"
            "Read 32 bit words from given memory addresses; or write "
            "given 32 bit values\nto the given addresses. Mmaps /dev/mem "
            "to do this. Note all values are\nin hex apart from time delays "
            "which are in decimal (unit: milliseconds).\n");
}

static inline volatile unsigned int *
mmp_add(unsigned char * p, unsigned int v)
{
    return (volatile unsigned int *)(p + v);
}

/* If decodes hex number okay then returns it and if errp is non-NULL places
 * 0 at *errp. If cannot decode hex number returns 0 and if errp is non-NULL
 * places 1 at *errp. */
unsigned int
get_hnum(const char * buf, int len, int * errp)
{
    char b[68];
    int res;
    unsigned int unum;

    if (len > (int)(sizeof(b) - 4)) {
        fprintf(stderr, "get_hnum: len=%d too large for a hex number\n",
                len);
        if (errp)
            *errp = 1;
        return 0;       /* no good return, 0 is least harmful */
    }
    memcpy(b, buf, len);
    b[len] = '\0';
    if ('0' == b[0] && (('x' == b[1]) || ('X' == b[1])))
        res = sscanf(b + 2, "%x", &unum);
    else
        res = sscanf(b, "%x", &unum);
    if (1 != res) {
        fprintf(stderr, "get_hnum: could not decode hex number\n");
        if (errp)
            *errp = 1;
        return 0;
    }
    if (errp)
        *errp = 0;
    return unum;
}

/* Read hex numbers (up to 32 bits in size) from command line (comma (or
 * (single) space) separated list) or from stdin or file (one or two per
 * line, comma separated list or space separated list). If
 * MAJOR_TYP_WRITE==major_typ then address field may be 't' or 'T' in
 * which case value is decimal time delay in milliseconds.
 * Returns 0 if ok, or 1 if error. */
static int
build_arr(FILE * fp, const char * inp, int major_typ, struct elem_t * arr,
          int max_arr_len)
{
    int in_len, k, j, m, wr, ad, got_addr;
    int err = 0;
    unsigned int u, ms;
    const char * lcp;
    const char * allowp;

    if ((NULL == arr) || (max_arr_len < 1))
        return 1;
    wr = (MAJOR_TYP_WRITE == major_typ);
    allowp = wr ?  "0123456789aAbBcCdDeEfFtTxX ,\t" :
                   "0123456789aAbBcCdDeEfFxX ,\t";
    if (fp) {        /* read from file or stdin */
        char line[512];
        int off = 0;

        for (j = 0, ad = 0; j < 512; ++j) {
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
                if (wr && ('t' == tolower(*lcp))) {
                    lcp = strpbrk(lcp, " ,\t");
                    if (NULL == lcp) {
                        fprintf(stderr, "build_arr: want delay value after "
                                "'t' (on same line)\n");
                        return 1;
                    }
                    lcp += strspn(lcp, " ,\t");
                    if ('\0' == *lcp) {
                        fprintf(stderr, "build_arr: want delay value after "
                                "'t' (on same line) \n");
                        return 1;
                    }
                    if (isdigit(*lcp)) {
                        sscanf(lcp, "%u", &ms);
                        arr[off + k].typ = ELEM_TYP_WAIT_MS;
                        arr[off + k].val = ms;
                    }
                } else {
                    u = get_hnum(lcp, strlen(lcp), &err);
                    if (err) {
                        fprintf(stderr, "build_arr: could not decode '%s' "
                                "as a hex number\n", lcp);
                        return 1;
                    }
                    if ((off + k) >= max_arr_len) {
                        fprintf(stderr, "build_arr: array length exceeded\n");
                        return 1;
                    }
                    got_addr = 0;
                    if (wr) {
                        if (ad) {
                            ad = 0;
                            arr[off + k].val = u;
                        } else {
                            ad = 1;
                            arr[off + k].typ = ELEM_TYP_WRITE;
                            arr[off + k].addr = u;
                            ++got_addr;
                            --k;
                        }
                    } else {
                        arr[off + k].typ = ELEM_TYP_READ;
                        arr[off + k].addr = u;
                        ++got_addr;
                    }
                    if (got_addr) {
                        if (u < min_addr) {
                            fprintf(stderr, "build_arr: 0x%x less than "
                                    "minimum address, see '-m <addr>'\n", u);
                            return 1;
                        } else if ((! force_nm4) && (u & 0x3)) {
                            fprintf(stderr, "build_arr: 0x%x not module 4, "
                                    "see '-F' to override\n", u);
                            return 1;
                        }
                    }
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
        if (wr && ad) {
            fprintf(stderr, "build_arr: write address but missing value\n");
            return 1;
        }
        if (off < max_arr_len)
            arr[off].typ = ELEM_TYP_NULL;
    } else if (inp) {        /* list of numbers on command line */
        lcp = inp;
        in_len = strlen(inp);
        if (0 == in_len) {
            arr[0].typ = ELEM_TYP_NULL;
            return 0;
        }
        k = strspn(inp, allowp);
        if (in_len != k) {
            fprintf(stderr, "build_arr: error at pos %d\n", k + 1);
            return 1;
        }
        for (k = 0, ad = 0; k < max_arr_len; ++k) {
            if (wr && ('t' == tolower(*lcp))) {
                lcp = strpbrk(lcp, " ,\t");
                if (NULL == lcp) {
                    fprintf(stderr, "build_arr: want delay value after "
                            "'t'  \n");
                    return 1;
                }
                lcp += strspn(lcp, " ,\t");
                if ('\0' == *lcp) {
                    fprintf(stderr, "build_arr: want delay value after "
                            "'t'   \n");
                    return 1;
                }
                if (isdigit(*lcp)) {
                    sscanf(lcp, "%u", &ms);
                    arr[k].typ = ELEM_TYP_WAIT_MS;
                    arr[k].val = ms;
                }
            } else {
                u = get_hnum(lcp, strlen(lcp), &err);
                if (err) {
                    fprintf(stderr, "build_arr: could not decode '%s' "
                            "as a hex number\n", lcp);
                    return 1;
                }
                got_addr = 0;
                if (wr) {
                    if (ad) {
                        ad = 0;
                        arr[k].val = u;
                    } else {
                        ad = 1;
                        arr[k].typ = ELEM_TYP_WRITE;
                        arr[k].addr = u;
                        ++got_addr;
                        --k;
                    }
                } else {
                    arr[k].typ = ELEM_TYP_READ;
                    arr[k].addr = u;
                    ++got_addr;
                }
                if (got_addr) {
                    if (u < min_addr) {
                        fprintf(stderr, "build_arr: 0x%x less than minimum "
                                "address, see '-m <addr>'\n", u);
                        return 1;
                    } else if ((! force_nm4) && (u & 0x3)) {
                        fprintf(stderr, "build_arr: 0x%x not module 4, "
                                "see '-F' to override\n", u);
                        return 1;
                    }
                }
            }
            lcp = strpbrk(lcp, " ,\t");
            if (NULL == lcp)
                break;
            lcp += strspn(lcp, " ,\t");
            if ('\0' == *lcp)
                break;
        }
        if (wr && ad) {
            fprintf(stderr, "build_arr: write address but missing value\n");
            return 1;
        }
        if (k == max_arr_len) {
            fprintf(stderr, "build_arr: array length exceeded\n");
            return 1;
        }
        if (k < (max_arr_len - 1))
            arr[k + 1].typ = ELEM_TYP_NULL;
    }
    return 0;
}


int
main(int argc, char * argv[])
{
    int mem_fd, res, k, opt, mtyp;
    int dummy = 0;
    int user_mask_given = 0;
    int do_quiet = 0;
    int do_read = 0;
    int shift_r = 0;
    int do_write = 0;
    unsigned int user_mask_result;
    const char * fname = NULL;
    const char * istring = NULL;
    struct elem_t * ep;
    void * mmap_ptr;
    volatile unsigned int * mmp;
    struct timespec request;
    off_t mask_addr, prev_mask_addr;
    int have_prev_mask = 0;
    unsigned long ul;
    FILE * input_filep = NULL;

    mem_fd = -1;
    while ((opt = getopt(argc, argv, "df:hi:m:M:qrs:vVw")) != -1) {
        switch (opt) {
            break;
        case 'd':
            ++dummy;
            break;
        case 'f':
            fname = optarg;
            break;
        case 'h':
        case '?':
            usage();
            return 0;
        case 'i':
            istring = optarg;
            break;
        case 'm':
            if (1 != sscanf(optarg, "%x", &min_addr)) {
                fprintf(stderr, "'-m' unable to decode <addr>\n");
                return 1;
            }
            break;
        case 'M':
            if (1 != sscanf(optarg, "%x", &user_mask_result)) {
                fprintf(stderr, "'-M' unable to decode <mask>\n");
                return 1;
            }
            ++user_mask_given;
            break;
        case 'q':
            ++do_quiet;
            break;
        case 'r':
            ++do_read;
            break;
        case 's':
            if ((1 != sscanf(optarg, "%d", &shift_r)) || (shift_r < 0)) {
                fprintf(stderr, "'-s' unable to decode <shift_r> or bad "
                        "value\n");
                return 1;
            }
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            fprintf(stderr, "version: %s\n", version_str);
            return 0;
        case 'w':
            ++do_write;
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

    if (do_read && do_write) {
        fprintf(stderr, "can either read ('-r') or write ('w'), "
                "but not both\n\n");
        usage();
        return 1;
    }
    if ((0 == do_read) && (0 == do_write)) {
        fprintf(stderr, "nothing to read ('-r') or write ('w'), "
                "so exit\n\n");
        usage();
        return 1;
    }
    if (user_mask_given && do_write) {
        fprintf(stderr, "'-M <mask>' can only be used with '-r'\n\n");
        usage();
        return 1;
    }
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
    } else if (NULL == istring) {
        fprintf(stderr, "expecting either '-i' or '-f'  but got neither\n");
        usage();
        return 1;
    }

    mtyp = do_read ? MAJOR_TYP_READ : MAJOR_TYP_WRITE;
    res = build_arr(input_filep, istring, mtyp, elem_arr, MAX_ELEMS);
    if (res) {
        fprintf(stderr, "build_arr() failed\n");
        return 1;
    }

    if (dummy || (verbose > 1)) {
        printf("build_arr after command line input processing:\n");
        for (k = 0; elem_arr[k].typ > 0; ++k) {
            ep = elem_arr + k;
            if (ELEM_TYP_READ == ep->typ)
                printf("    R: 0x%x\n", ep->addr);
            else if (ELEM_TYP_WRITE == ep->typ)
                printf("    W: 0x%x 0x%x\n", ep->addr, ep->val);
            else if (ELEM_TYP_WAIT_MS == ep->typ)
                printf("    T: %d\n", ep->val);
            else
                printf("    U: \n");
        }
        if (dummy)
            return 0;
    }

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose)
        printf("open(" DEV_MEM ", O_RDWR | O_SYNC) okay\n");

    mmap_ptr = (void *)-1;
    prev_mask_addr = 0;
    have_prev_mask = 0;
    for (k = 0; elem_arr[k].typ > 0; ++k) {
        ep = elem_arr + k;
        if (ELEM_TYP_WAIT_MS == ep->typ) {
            if (ep->val < 1)
                continue;
            request.tv_sec = ep->val / 1000;
            request.tv_nsec = (ep->val % 1000) * 1000000;
            if (nanosleep(&request, NULL) < 0) {
                perror("nanosleep");
                res = 1;
                goto cleanup;
            }
            if (verbose > 1)
                fprintf(stderr, "slept for %d milliseconds\n", ep->val);
            continue;
        }
        mask_addr = (ep->addr & ~MAP_MASK);
        if (((void *)-1 == mmap_ptr) ||
            (have_prev_mask && (prev_mask_addr != mask_addr))) {
            if ((void *)-1 != mmap_ptr) {
                if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
                    fprintf(stderr, "mmap_ptr=%p:\n", mmap_ptr);
                    perror("    munmap");
                    res = 1;
                    goto cleanup;
                } else if (verbose > 2)
                    fprintf(stderr, "munmap() ok, mmap_ptr=%p\n", mmap_ptr);
            }
            mmap_ptr = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE,
                            MAP_SHARED, mem_fd, mask_addr);
            if ((void *)-1 == mmap_ptr) {
                fprintf(stderr, "addr=0x%x, mask_addr=0x%lx :\n", ep->addr,
                        mask_addr);
                perror("    mmap");
                res = 1;
                goto cleanup;
            } else if (verbose > 2)
                fprintf(stderr, "mmap() ok, mask_addr=0x%lx, mmap_ptr=%p\n",
                        mask_addr, mmap_ptr);
            have_prev_mask = 1;
            prev_mask_addr = mask_addr;
        }
        mmp = mmp_add(mmap_ptr, ep->addr & MAP_MASK);
        if (ELEM_TYP_WRITE == ep->typ) {
            *mmp = ep->val;
            if (1 == verbose)
                fprintf(stderr, "wrote: addr=0x%x, val=0x%x\n",
                        ep->addr, ep->val);
            else if (verbose > 1)
                fprintf(stderr, "wrote: addr=0x%x, val=0x%x "
                        "[mask_addr=0x%lx]\n", ep->addr, ep->val,
                        mask_addr);
        } else if (ELEM_TYP_READ == ep->typ) {
            ul = *mmp;
            ep->val = (unsigned int)ul;
            if (user_mask_given) {
                user_mask_result &= ep->val;
                break;
            }
            if (1 == verbose)
                fprintf(stderr, "read: addr=0x%x, val=0x%x\n",
                        ep->addr, ep->val);
            else if (verbose > 1)
                fprintf(stderr, "read: addr=0x%x, val=0x%x [mask_addr="
                        "0x%lx]\n", ep->addr, ep->val, mask_addr);
        }
    }

    if (user_mask_given) {
        if (! do_quiet)
            printf("%x\n", (shift_r ? (user_mask_result >> shift_r) :
                                      user_mask_result));
    } else {
        for (k = 0; elem_arr[k].typ > 0; ++k) {
            ep = elem_arr + k;
            if (ELEM_TYP_READ == ep->typ)
                printf("%x\n", ep->val);
        }
    }

    if ((void *)-1 != mmap_ptr) {
        if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
            fprintf(stderr, "mmap_ptr=%p:\n", mmap_ptr);
            perror("    munmap");
            res = 1;
            goto cleanup;
        } else if (verbose > 2)
            fprintf(stderr, "trailing munmap() ok, mmap_ptr=%p\n", mmap_ptr);
    }
    res = 0;

cleanup:
    if (mem_fd >= 0)
        close(mem_fd);
    if ((0 == res) && user_mask_given)
        return !user_mask_result;
    else
        return res;
}
