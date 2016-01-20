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

/************************************************************************
 * a5d2_pio_set.c
 *
 * Utility for setting SAMA5D2 SoC PIO line attributes.
 * The SAMA5D2 has 4 PIOs: PIOA, PIOB, PIOC and PIOD. Each contain
 * 32 lines (GPIOs), for example PIOA contains PA0 to PA31.
 * This utiity uses memory mapped IO.
 *
 ****************************************************/

#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>


static const char * version_str = "1.00 20160119";


#define PIO_BANKS_SAMA5D2 4  /* PA0-31, PB0-31, PC0-31 and PD0-32 */
#define LINES_PER_BANK 32
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)
#define DEV_MEM "/dev/mem"

#define GPIO_BANK_ORIGIN "/sys/class/gpio/gpiochip0"
/* Earlier kernels (G20+G25) offset GPIO numbers by 32 so
 * /sys/class/gpio/gpiochip32 (a directory) was the lowest
 * numbered bank and corresponded to PIOA. Now (in lk 3.7)
 * /sys/class/gpio/gpiochip0 exists and corresponds to PIOA.
 *
 */

#define SAMA5D2_PIO_WPKEY 0x50494f      /* "PIO" in ASCII */
#define SAMA5D2_PIO_FRZKEY 0x494F46     /* "IOF" in ASCII */

#define PIO_WPMR 0xfc0385e0     /* Write protection mode (rw) */
#define PIO_WPSR 0xfc0385e4     /* Write protection status (ro) */
#define S_PIO_SCDR 0xfc039500   /* Secure slow clock divider debouncing(rw) */
#define S_PIO_WPMR 0xfc0395e0   /* Secure write protection mode (rw) */
#define S_PIO_WPSR 0xfc0395e4   /* Secure write protection status (r0) */

#define FUNC_GPIO 0
#define PERI_A 1
#define PERI_B 2
#define PERI_C 3
#define PERI_D 4
#define PERI_E 5
#define PERI_F 6
#define PERI_G 7
#define MAX_PERIPH_NUM 7
#define DIR_IO 0    /* print as "[io]" */
#define DIR_I 1     /* print as "[i]" */
#define DIR_O 2     /* print as "[o]" */
#define DIR_OO 3    /* Atmel, what does this mean? Print as "[output]" */
#define DIR_UNK 4   /* ??, print as "[ ]" */
#define CFGR_FUNC_MSK 0x7
#define CFGR_DIR_MSK (1 << 8)   /* 0 -> pure input; 1 -> output */
#define CFGR_PUEN_MSK (1 << 9)  /* pull-up enable */
#define CFGR_PDEN_MSK (1 << 10) /* pull-down enable */
#define CFGR_IFEN_MSK (1 << 12) /* input filter enable */
#define CFGR_IFSCEN_MSK (1 << 13)       /* input filter slow clock enable */
#define CFGR_OPD_MSK (1 << 14)  /* open drain (like open collector) */
#define CFGR_SCHMITT_MSK (1 << 15)      /* Schmitt tripper (on input) */
                                        /* N.B. 0 -> enabled, 1 -> dis */
#define CFGR_DRVSTR_MSK 0x30000         /* 0, 1 -> LO; 2 -> ME; 3 -> HI */
#define CFGR_DRVSTR_SHIFT 16
#define CFGR_EVTSEL_MSK 0x7000000       /* 0 -> falling; 1 -> rising, ... */
#define CFGR_EVTSEL_SHIFT 24
#define CFGR_PCFS_MSK (1 << 29) /* physical configuration freezes status */
#define CFGR_ICFS_MSK (1 << 30) /* interrupt configuration freezes status */
#define IOFR_FINT_MSK (1 << 1)  /* freeze: ifen, ifscen, evtsel */
#define IOFR_FPHY_MSK (1 << 0)  /* freeze: func, dir puen, pden, opd, */
                                /* schmitt + drvstr */


struct opts_t {
    int dir;
    int do_func;
    int enumerate;
    int evtsel;
    int freeze_phy0int1b2;
    int en_if;
    int di_if;
    int en_interrupt;
    int di_interrupt;
    int en_opd;
    int di_opd;
    int en_pullup1dn2;
    int di_pullup1dn2;
    int out_data;
    int en_schmitt;
    int di_schmitt;
    int en_if_slow;
    int di_if_slow;
    int wpen;
    int scdr_div;
    int drvstr;
    int verbose;
    unsigned int msk;
    unsigned int dat;
    bool evtsel_given;
    bool freeze_given;
    bool wp_given;
    bool scdr_given;
    bool drvstr_given;
    bool wr_dat_given;
    bool dir_given;
};

struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};


static unsigned int pio_mskr[] = {0xfc038000, 0xfc038040, 0xfc038080,
                        0xfc0380c0};    /* Mask (rw) */
static unsigned int pio_cfgr[] = {0xfc038004, 0xfc038044, 0xfc038084,
                        0xfc0380c4};    /* Configuration (rw) */
#if 0
static unsigned int pio_pdsr[] = {0xfc038008, 0xfc038048, 0xfc038088,
                        0xfc0380c8};    /* Pin data status (ro) */
static unsigned int pio_locksr[] = {0xfc03800c, 0xfc03804c, 0xfc03808c,
                        0xfc0380cc};    /* Lock status (ro) */
#endif
static unsigned int pio_sodr[] = {0xfc038010, 0xfc038050, 0xfc038090,
                        0xfc0380d0};    /* Set output data (wo) */
static unsigned int pio_codr[] = {0xfc038014, 0xfc038054, 0xfc038094,
                        0xfc0380d4};    /* Clear output data (wo) */
static unsigned int pio_odsr[] = {0xfc038018, 0xfc038058, 0xfc038098,
                        0xfc0380d8};    /* Output data status (rw) */
static unsigned int pio_ier[] = {0xfc038020, 0xfc038060, 0xfc0380a0,
                        0xfc0380e0};    /* Interrupt enable (wo) */
static unsigned int pio_idr[] = {0xfc038024, 0xfc038064, 0xfc0380a4,
                        0xfc0380e4};    /* Interrupt disable (wo) */
#if 0
static unsigned int pio_imr[] = {0xfc038028, 0xfc038068, 0xfc0380a8,
                        0xfc0380e8};    /* Interrupt mask (ro) */
static unsigned int pio_isr[] = {0xfc03802c, 0xfc03806c, 0xfc0380ac,
                        0xfc0380ec};    /* Interrupt status (ro) */
#endif
static unsigned int pio_iofr[] = {0xfc03803c, 0xfc03807c, 0xfc0380bc,
                        0xfc0380fc};    /* I/O freeze (wo) */

#if 0
static unsigned int s_pio_sionr[] = {0xfc039030, 0xfc039070, 0xfc0390b0,
                        0xfc0390f0};    /* Secure I/O non-secure (wo) */
static unsigned int s_pio_siosr[] = {0xfc039034, 0xfc039074, 0xfc0390b4,
                        0xfc0390f4};    /* Secure I/O secure (wo) */
static unsigned int s_pio_iossr[] = {0xfc039038, 0xfc039078, 0xfc0390b8,
                        0xfc0390f8};    /* Secure I/O security status (ro) */
static unsigned int s_pio_iofr[] = {0xfc03903c, 0xfc03907c, 0xfc0390bc,
                        0xfc0390fc};    /* Secure I/O freeze (wo) */
#endif


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
usage(int help_val)
{
    if (1 == help_val)
        pr2serr("Usage: a5d2_pio_set [-b <bn>] [-d <div>] [-D <drvstr>] [-e] "
                "[-E <evt>]\n"
                "                    [-f <func>] [-F <phy0int1b2>] [-g|G] "
                "[-h] [-i|I]\n"
                "                    [-m|M] [-p <port>] [-r <dir>] "
                "[-s <func>] [-S <n>]\n"
                "                    [-t|T] [-u|U] [-uu|UU] [-v] [-V] "
                "[-w <wpen>]\n"
                "                    [-X <msk,dat>] [-z|Z]\n"
                "  where:\n"
                "    -b <bn>      bit number within port (0 to 31). Also "
                "accepts full\n"
                "                 GPIO name (e.g. '-b PC7' equivalent to "
                "'-p c -b 7')\n"
                "    -d <div>     slow clock divider [period=2*(<div>+1)"
                "*slow_clock_per]\n"
                "    -D <drvstr>     IO drive: 0->LO, 1->LO, 2->ME, 3->HI\n"
                "                    alternatively the letter L, M or H can "
                "be given\n"
                "    -e           enumerate pin names with corresponding "
                "kernel pin\n"
                "    -E <evt>     <evt> is input event: 0 -> falling edge, "
                "1 ->\n"
                "                 rising, 2 -> both edges, 3 -> low level, "
                "4 -> high\n"
                "    -f <func>    select line function: P->PIO, "
                "A->peri_A\n"
                "                 B->peri_B, C, D, E, F or G); "
                "alternatively\n"
                "                 <func> may be a number: 0->PIO(GPIO), "
                "1->peri_A,\n"
                "                 2->peri_B and so on until 7->peri_G\n"
                "    -F <phy0int1b2>    freeze config: 0 -> physical, "
                "1 -> interrupt\n"
                "                       2 -> physical+interrupt\n"
                "    -g|G         disable|enable (glitch) input filter\n"
                "    -h           print usage message; use twice for "
                "more help\n"
                "    -i|I         interrupt disable|enable\n"
                "    -m|M         disable|enable open drain (formerly "
                "multi-drive)\n"
                "    -p <port>    port bank ('A' to 'D') or gpio kernel "
                "line number\n"
                "    -r <dir>     direction: 0 -> pure input; 1 -> enabled "
                "for output\n"
                "                 also accepts 'I' for pure input and 'O' "
                "for output\n"
                "    -s <func>    same as '-f <func>'\n"
                "    -S <n>       set output data line to <n> (0 -> low, "
                "1 -> high)\n"
                "    -t|T         disable|enable Schmitt trigger on input\n"
                "    -u|U         disable|enable internal pull-up. Use twice "
                "to\n"
                "                 disable|enable internal pull-down. "
                "Example:\n"
                "                 switch PC12 from pull-up to pull-down:\n"
                "                     '-b PC12 -u -UU'\n"
                "    -v           increase verbosity (multiple times for "
                "more)\n"
                "    -V           print version string then exit\n"
                "    -w <wpen>    write protect mode (for whole PIO) set to "
                "<wpen>\n"
                "                 0->disabled (def, no write protection), "
                "1->enabled\n"
                "    -X <msk,dat>   write <dat> to <port> for those lines set "
                "in <msk>\n"
                "                   <msk> and <dat> are 32 bit hexadecimal "
                "values\n"
                "    -z|Z         disable|enable input filter slow clock\n"
                "Set SAMA5D2 SoCs PIO attributes. Uses memory mapped IO to "
                "access PIO\nregisters directly; bypasses kernel. Use '-hh' "
                "for more.\n"
               );
    else    /* -hh */
        pr2serr("Usage: a5d2_pio_set [-b <bn>] [-d <div>] [-D <drvstr>] [-e] "
                "[-E <evt>]\n"
                "                    [-f <func>] [-F <phy0int1b2>] [-g|G] "
                "[-h] [-i|I]\n"
                "                    [-m|M] [-p <port>] [-r <dir>] "
                "[-s <func>] [-S <n>]\n"
                "                    [-t|T] [-u|U] [-uu|UU] [-v] [-V] "
                "[-w <wpen>]\n"
                "                    [-X <msk,dat>] [-z|Z]\n\n"
                "Use '-h' to get an explanation of the above command "
                "line options.\n\n"
                "Setting the output data line (e.g. with '-S 1') only changes "
                "the\nexternal line when <func> is 0 (or 'P') . "
                "If the line is set high\n(i.e. '-S 1') and if 'open drain' "
                "is enabled ('-M') then an internal\nor external pull-up is "
                "needed to see a high level on the external line.\n\n"
                "A line's internal pull-up and pull-down (resistor) cannot "
                "be active\n(enabled) at the same time. The policy of the "
                "SAMA5D2 is to ignore\nthe pull-down when both are given."
                "\n\nOnce a GPIO line is frozen, only a hardware reset "
                "(e.g. a power\ncycle) will unfreeze that line.\n\n"
                "When multiple actions are requested, the order in which "
                "they are\napplied may be significant. If disable "
                "interrupts is requested,\nit is applied first, followed by "
                "disable write protection, followed\nby any requested "
                "change to <func>. The final three actions, if\nrequested, "
                "are to enable write protection, enable interrupts, then\n"
                "freeze physical or interrupts (or both) respectively.\n"
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

/* Since we map a whole page (MAP_SIZE) then that page may already be
 * mapped which means we can skip a munmap() and mmap() call. Returns
 * new or re-used pointer to mmapped page, or returns NULL if problem. */
static void *
check_mmap(int mem_fd, unsigned int wanted_addr, struct mmap_state * msp,
           const struct opts_t * op)
{
    off_t mask_addr;

    mask_addr = (wanted_addr & ~MAP_MASK);
    if ((0 == msp->mmap_ok) || (msp->prev_mask_addrp != mask_addr)) {
        if (msp->mmap_ok) {
            if (-1 == munmap(msp->mmap_ptr, MAP_SIZE)) {
                pr2serr("mmap_ptr=%p:\n", msp->mmap_ptr);
                perror("    munmap");
                return NULL;
            } else if (op->verbose > 2)
                pr2serr("munmap() ok, mmap_ptr=%p\n", msp->mmap_ptr);
        }
        msp->mmap_ptr = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                             mem_fd, mask_addr);
        if ((void *)-1 == msp->mmap_ptr) {
            msp->mmap_ok = 0;
            pr2serr("addr=0x%x, mask_addr=0x%lx :\n", wanted_addr, mask_addr);
            perror("    mmap");
            return NULL;
        }
        msp->mmap_ok = 1;
        msp->prev_mask_addrp = mask_addr;
        if (op->verbose > 2)
            pr2serr("mmap() ok, addr=0x%x, mask_addr=0x%lx, mmap_ptr=%p\n",
                    wanted_addr, mask_addr, msp->mmap_ptr);
    }
    return msp->mmap_ptr;
}

static volatile unsigned int *
get_mmp(int mem_fd, unsigned int wanted_addr, struct mmap_state * msp,
        const struct opts_t * op)
{
    void * mmap_ptr;

    mmap_ptr = check_mmap(mem_fd, wanted_addr, msp, op);
    if (NULL == mmap_ptr)
        return NULL;
    return mmp_add(mmap_ptr, wanted_addr & MAP_MASK);
}

static volatile unsigned int *
do_mask_get_cfgr(int mem_fd, int bit_num, int pioc_num,
                 const struct opts_t * op)
{
    unsigned int bit_mask;
    volatile unsigned int * mmp;
    struct mmap_state mstat;

    memset(&mstat, 0, sizeof(mstat));
    bit_mask = 1 << bit_num;
    if (NULL == ((mmp = get_mmp(mem_fd, pio_mskr[pioc_num], &mstat, op))))
        return NULL;
    if (bit_mask != *mmp)
        *mmp = bit_mask;
    return get_mmp(mem_fd, pio_cfgr[pioc_num], &mstat, op);
}

static int
pio_set(int mem_fd, int bit_num, int pioc_num, const struct opts_t * op)
{
    unsigned int addr, bit_mask;
    unsigned int cfgr = 0;
    volatile unsigned int * ommp;
    volatile unsigned int * cmmp = NULL;
    struct mmap_state mstat;

    memset(&mstat, 0, sizeof(mstat));
    bit_mask = 1 << bit_num;

    if (op->di_interrupt) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_idr[pioc_num], &mstat, op))))
            return 1;
        *ommp = bit_mask;
    }
    if (op->wp_given && (0 == op->wpen)) {
        if (NULL == ((ommp = get_mmp(mem_fd, PIO_WPMR, &mstat, op))))
            return 1;
        *ommp = (SAMA5D2_PIO_WPKEY << 8) | op->wpen;
    }
    if (op->do_func >= 0) {
        cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op);
        if (NULL == cmmp)
            return 1;
        cfgr = *cmmp;
        if ((unsigned int)op->do_func != (CFGR_FUNC_MSK & cfgr))
            *cmmp = ((~CFGR_FUNC_MSK & cfgr) | op->do_func);
    }
    if (op->dir_given) {
        cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op);
        if (NULL == cmmp)
            return 1;
        cfgr = *cmmp;
        if (op->dir != !!(CFGR_DIR_MSK & cfgr)) {
            if (op->dir)
                *cmmp = (CFGR_DIR_MSK | cfgr);
            else
                *cmmp = (~CFGR_DIR_MSK & cfgr);
        }
    }
    if (op->di_schmitt || op->en_schmitt) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_SCHMITT_MSK & cfgr) != !!op->di_schmitt) {
            if (op->di_schmitt)
                *cmmp = (cfgr | CFGR_SCHMITT_MSK);
            else
                *cmmp = (~CFGR_SCHMITT_MSK & cfgr);
        }
    }
    if (op->scdr_given) {
        if (NULL == ((ommp = get_mmp(mem_fd, S_PIO_SCDR, &mstat, op))))
            return 1;
        *ommp = op->scdr_div;
    }
    if (op->di_if_slow || op->en_if_slow) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_IFSCEN_MSK & cfgr) != !!op->en_if_slow) {
            if (op->en_if_slow)
                *cmmp = (cfgr | CFGR_IFSCEN_MSK);
            else
                *cmmp = (~CFGR_IFSCEN_MSK & cfgr);
        }
    }
    if (op->di_if || op->en_if) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_IFEN_MSK & cfgr) != !!op->en_if) {
            if (op->en_if)
                *cmmp = (cfgr | CFGR_IFEN_MSK);
            else
                *cmmp = (~CFGR_IFEN_MSK & cfgr);
        }
    }
    if (op->evtsel_given) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if ((unsigned int)op->evtsel !=
            ((CFGR_EVTSEL_MSK & cfgr) >> CFGR_EVTSEL_SHIFT))
            *cmmp = ((~CFGR_EVTSEL_MSK & cfgr) |
                    ((unsigned int)op->evtsel << CFGR_EVTSEL_SHIFT));
    }
    if (op->di_opd || op->en_opd) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_OPD_MSK & cfgr) != !!op->en_if) {
            if (op->en_opd)
                *cmmp = (cfgr | CFGR_OPD_MSK);
            else
                *cmmp = (~CFGR_OPD_MSK & cfgr);
        }
    }

    if ((op->di_pullup1dn2 > 0) || (op->en_pullup1dn2 > 0)) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        /* Apply disables, if any, first */
        if (op->di_pullup1dn2 > 0) {
            if (1 & op->di_pullup1dn2)
                *cmmp = (~CFGR_PUEN_MSK & cfgr);
            else
                *cmmp = (~CFGR_PDEN_MSK & cfgr);
        }
        if (op->en_pullup1dn2 > 0) {
            if (1 & op->en_pullup1dn2)
                *cmmp = (cfgr | CFGR_PUEN_MSK);
            else
                *cmmp = (cfgr | CFGR_PDEN_MSK);
        }
    }
    if (op->out_data >= 0) {
        addr = ((op->out_data > 0) ? pio_sodr[pioc_num] : pio_codr[pioc_num]);
        if (NULL == ((ommp = get_mmp(mem_fd, addr, &mstat, op))))
            return 1;
        *ommp = bit_mask;
    }
    if (op->drvstr_given) {
        if (! cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if ((unsigned int)op->drvstr !=
            ((CFGR_DRVSTR_MSK & cfgr) >> CFGR_DRVSTR_SHIFT))
            *cmmp = ((~CFGR_DRVSTR_MSK & cfgr) |
                     ((unsigned int)op->drvstr << CFGR_DRVSTR_SHIFT));
    }
    if (op->wr_dat_given) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_mskr[pioc_num], &mstat, op))))
            return 1;
        *ommp = op->msk;
        if (NULL == ((ommp = get_mmp(mem_fd, pio_odsr[pioc_num], &mstat, op))))
            return 1;
        *ommp = op->dat;
    }
    if (op->wp_given && op->wpen) {
        if (NULL == ((ommp = get_mmp(mem_fd, PIO_WPMR, &mstat, op))))
            return 1;
        *ommp = (SAMA5D2_PIO_WPKEY << 8) | op->wpen;
    }
    if (op->en_interrupt) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_ier[pioc_num], &mstat, op))))
            return 1;
        *ommp = bit_mask;
    }
    if (op->freeze_given) {
        if (! cmmp) {
            /* need pio_msk written to prior to write to pio_iofr */
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (NULL == ((ommp = get_mmp(mem_fd, pio_iofr[pioc_num], &mstat, op))))
            return 1;
        if (1 == op->freeze_phy0int1b2)
            *ommp = (SAMA5D2_PIO_FRZKEY << 8) | IOFR_FINT_MSK;
        else if (0 == op->freeze_phy0int1b2)
            *ommp = (SAMA5D2_PIO_FRZKEY << 8) | IOFR_FPHY_MSK;
        else
            *ommp = (SAMA5D2_PIO_FRZKEY << 8) | IOFR_FPHY_MSK | IOFR_FINT_MSK;
    }

    if (mstat.mmap_ok) {
        if (-1 == munmap(mstat.mmap_ptr, MAP_SIZE)) {
            pr2serr("mmap_ptr=%p:\n", mstat.mmap_ptr);
            perror("    munmap");
            return 1;
        } else if (op->verbose > 2)
            pr2serr("trailing munmap() ok, mmap_ptr=%p\n", mstat.mmap_ptr);
    }
    return 0;
}


int
main(int argc, char ** argv)
{
    int c, k, j, n, num, mem_fd, pioc_num, res;
    int do_help = 0;
    int help_exit = EXIT_SUCCESS;
    int knum = -1;
    int origin0 = 0;
    int bit_num = -1;
    const char * cp;
    const char * funcp = NULL;
    char ch;
    char bank = '\0';
    struct opts_t opts;
    struct opts_t * op;
    struct stat sb;

    op = &opts;
    memset(op, 0, sizeof(opts));
    op->out_data = -1;
    op->do_func = -1;
    while ((c = getopt(argc, argv,
                       "b:d:D:eE:f:F:gGhiImMp:r:s:S:tTuUvVw:X:zZ")) != -1) {
        switch (c) {
        case 'b':
            cp = optarg;
            if (isalpha(cp[0])) {
                if ('P' == toupper(*cp))
                    ++cp;
                ch = toupper(*cp);
                if ((ch >= 'A') && (ch <= 'E'))
                    bank = ch;
                else {
                    pr2serr("'-b' expects a letter ('A' to 'E')\n");
                    exit(EXIT_FAILURE);
                }
                ++cp;
            }
            k = atoi(cp);
            if ((k < 0) || (k > 31)) {
                pr2serr("'-b' expects a bit number from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            bit_num = k;
            break;
        case 'd':
            k = atoi(optarg);
            if ((k < 0) || (k > 16383)) {
                pr2serr("'-d' expects a bit number from 0 to 16383\n");
                return 1;
            }
            op->scdr_div = k;
            op->scdr_given = true;
            break;
        case 'D':
            if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 3)) {
                    pr2serr("'-D' expects a bit number from 0 to 3\n");
                    return 1;
                }
                op->drvstr = k;
            } else {
                switch (toupper(*optarg)) {
                case 'L':       /* lo, low or LOW should work */
                    op->drvstr = 0;
                    break;
                case 'M':       /* me, medium or MEDIUM should work */
                    op->drvstr = 2;
                    break;
                case 'H':       /* he, high or HIGH should work */
                    op->drvstr = 3;
                    break;
                default:
                    pr2serr("'-D' expects a word starting with 'L', 'M' or "
                            "'H'\n");
                    return 1;
                }
            }
            op->drvstr_given = true;
            break;
        case 'E':
            k = atoi(optarg);
            if ((k < 0) || (k > 4)) {
                pr2serr("'-E' expects a bit number from 0 to 4\n");
                return 1;
            }
            op->evtsel = k;
            op->evtsel_given = true;
            break;
        case 'e':
            ++op->enumerate;
            break;
        case 'f':
            funcp = optarg;
            break;
        case 'F':
            if (! isdigit(*optarg)) {
                pr2serr("'-F' expects a value of 0, 1 or 2\n");
                return 1;
            }
            k = atoi(optarg);
            if ((k < 0) || (k > 2)) {
                pr2serr("'-F' expects a value of 0, 1 or 2\n");
                return 1;
            }
            op->freeze_phy0int1b2 = k;
            op->freeze_given = true;
        case 'g':
            ++op->di_if;
            break;
        case 'G':
            ++op->en_if;
            break;
        case 'h':
            ++do_help;
            break;
        case 'i':
            ++op->di_interrupt;
            break;
        case 'I':
            ++op->en_interrupt;
            break;
        case 'm':
            ++op->di_opd;
            break;
        case 'M':
            ++op->en_opd;
            break;
        case 'p':
            if (isalpha(*optarg)) {
                ch = toupper(*optarg);
                if ((ch >= 'A') && (ch <= 'D'))
                    bank = ch;
                else {
                    pr2serr("'-p' expects a letter ('A' to 'D')\n");
                    return 1;
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 159)) {
                    pr2serr("'-p' expects a letter or a number from 0 to "
                            "159\n");
                    return 1;
                }
                knum = k;
            } else {
                pr2serr("'-p' expects a letter ('A' to 'D') or a number\n");
                return 1;
            }
            break;
        case 'r':
            if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 1)) {
                    pr2serr("'-r' expects 0 (pure input) or 1 (output "
                            "enabled)\n");
                    return 1;
                }
                op->dir = k;
            } else {
                switch (toupper(*optarg)) {
                case 'I':
                    op->dir = 0;
                    break;
                case 'O':
                    op->dir = 1;
                    break;
                default:
                    pr2serr("'-r' expects 'I' (pure input) or 'O' (output "
                            "enabled)\n");
                    return 1;
                }
            }
            op->dir_given = true;
            break;
        case 's':
            funcp = optarg;
            break;
        case 'S':
            if (! isdigit(*optarg)) {
                pr2serr("'-S' expects a value of 0 or 1\n");
                return 1;
            }
            k = atoi(optarg);
            if ((k < 0) || (k > 1)) {
                pr2serr("'-S' expects a value of 0 or 1\n");
                return 1;
            }
            op->out_data = k;
            break;
        case 't':
            ++op->di_schmitt;
            break;
        case 'T':
            ++op->en_schmitt;
            break;
        case 'u':
            ++op->di_pullup1dn2;
            break;
        case 'U':
            ++op->en_pullup1dn2;
            break;
        case 'v':
            ++op->verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            return 0;
        case 'w':
            k = atoi(optarg);
            if ((k < 0) || (k > 1)) {
                pr2serr("'-w' expects 0 (disabled) or 1 (enabled)\n");
                return 1;
            }
            op->wp_given = true;
            op->wpen = k;
            break;
        case 'X':
            k = sscanf(optarg, "%8x,%8x", &op->msk, &op->dat);
            if (2 != k) {
                pr2serr("'-X' expects msk,dat where both msk and dat are 32 "
                        "bit hexadecimal numbers\n");
                return 1;
            }
            op->wr_dat_given = true;
        case 'z':
            ++op->di_if_slow;
            break;
        case 'Z':
            ++op->en_if_slow;
            break;
        default: /* '?' */
            do_help = 1;
            help_exit = EXIT_FAILURE;
            break;
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                pr2serr("Unexpected extra argument: %s\n", argv[optind]);
            do_help = 1;
            help_exit = EXIT_FAILURE;
        }
    }
    if (do_help) {
        usage(do_help);
        exit(help_exit);
    }
    if (funcp) {
        switch (funcp[0]) {
        case 'a': case 'A':
            op->do_func = PERI_A;
            break;
        case 'b': case 'B':
            op->do_func = PERI_B;
            break;
        case 'c': case 'C':
            op->do_func = PERI_C;
            break;
        case 'd': case 'D':
            op->do_func = PERI_D;
            break;
        case 'e': case 'E':
            op->do_func = PERI_E;
            break;
        case 'f': case 'F':
            op->do_func = PERI_F;
            break;
        case 'g': case 'G':
            op->do_func = PERI_G;
            break;
        case 'p': case 'P':
            op->do_func = FUNC_GPIO;
            break;
        default:
            if (isdigit(funcp[0])) {
                k = atoi(funcp);
                if ((k >= 0) && (k <= 7)) {
                    op->do_func = k;
                    break;
                }
            }
            pr2serr("'-s' expects 'P', or 'A' to 'G'; or 0 to 7\n");
            return 1;
        }
    }
    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (op->verbose > 1)
            pr2serr("%s found so kernel pin numbers start at 0 (for PA0)\n",
                    GPIO_BANK_ORIGIN);
        ++origin0;
    } else if (op->verbose > 2)
        pr2serr("%s not found so kernel pin numbers start at 32 (for PA0)\n",
                GPIO_BANK_ORIGIN);

    if (op->enumerate) {
        if (op->enumerate) {
            printf("To see all lines decoded by function, use "
                   "'a5d2_pio_status -ee'\n");
            return 0;
        }
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

    if (op->wr_dat_given) {
        if ('\0' == bank) {
            pr2serr("With '-X <msk,dat>' require '-p <bank>' since it will "
                    "potentially\nwrite to all 32 lines in that bank\n");
            return 1;
        }
        if (bit_num >= 0) {
            pr2serr("With '-X <msk,dat>' require '-p bank' and '-b <bn>' "
                    "must not\nbe given\n");
            return 1;
        }
    }
    if (knum >= 0) {
        if (bit_num >= 0) {
            pr2serr("Give either '-p <knum>' or ('-b <bn>' and '-p <bank>') "
                    "but not both\n");
            return 1;
        }
        if ((! origin0) && (knum < 32)) {
            pr2serr("since %s not found assume kernel pin numbers start at "
                    "32\n(for PA0) so %d is too low\n", GPIO_BANK_ORIGIN,
                    knum);
            exit(EXIT_FAILURE);
        }
    } else if (bank) {
        if (bit_num < 0) {
            if (op->wp_given || op->scdr_given || op->wr_dat_given) {
                bit_num = 0;
                knum = ((! origin0) + bank - 'A') * 32;
            } else {
                pr2serr("If '-p <bank>' given then also need '-b <bn>'\n");
                return 1;
            }
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        pr2serr("Need to give gpio line with '-p <port>' and/or "
                "'-b <bn>'\n");
        goto help_exit;
    }
    if ((!!op->di_if + !!op->en_if) > 1) {
        pr2serr("Can only have one of '-g' and '-G'\n");
        goto help_exit;
    }
    if (((1 == op->di_interrupt) + (1 == op->en_interrupt)) > 1) {
        pr2serr("Can only have one of '-i' and '-I'\n");
        goto help_exit;
    }
    if (((op->di_interrupt > 1) + (op->en_interrupt > 1)) > 1) {
        pr2serr("Can only have one of '-ii' and '-II'\n");
        goto help_exit;
    }
    if ((!!op->di_opd + !!op->en_opd) > 1) {
        pr2serr("Can only have one of '-m' and '-M'\n");
        goto help_exit;
    }
    if (((1 == op->di_pullup1dn2) + (1 == op->en_pullup1dn2)) > 1) {
        pr2serr("Can only have one of '-u' and '-U'\n");
        goto help_exit;
    }
    if (((op->di_pullup1dn2 > 1) + (op->en_pullup1dn2 > 1)) > 1) {
        pr2serr("Can only have one of '-uu' and '-UU'\n");
        goto help_exit;
    }
    if ((!!op->di_schmitt + !!op->en_schmitt) > 1) {
        pr2serr("Can only have one of '-t' and '-T'\n");
        goto help_exit;
    }
    if ((!!op->di_if_slow + !!op->en_if_slow) > 1) {
        pr2serr("Can only have one of '-z' and '-Z'\n");
        goto help_exit;
    }

    pioc_num = bank ? (bank - 'A') : ((knum - (origin0 ? 0 : 32)) / 32);
    if (bit_num < 0)
        bit_num = knum % 32;
    if (op->verbose)
        printf("P%c%d:\n", 'A' + pioc_num, bit_num);
    if (op->verbose > 1)
        printf("  bit_mask=0x%08x\n", 1 << bit_num);

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (op->verbose > 2)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");

    res = pio_set(mem_fd, bit_num, pioc_num, op);

    if (mem_fd >= 0)
        close(mem_fd);
    return res;

help_exit:
    pr2serr(">>> Use '-h' for command line syntax, '-hh' for other help.\n");
    return EXIT_FAILURE;
}
