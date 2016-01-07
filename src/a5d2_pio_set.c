/*
 * Copyright (c) 2011-2016 Douglas Gilbert.
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
 * a5d3_pio_set.c
 *
 * Utility for setting SAMA5D3 SoC PIO line attributes.
 * The SAMA5D3 has 5 PIOs: PIOA, PIOB, PIOC, PIOD and PIOE. Each contain
 * 32 lines (GPIOs), for example PIOA contains PA0 to PA31.
 * This utiity uses memory mapped IO.
 *
 ****************************************************/

#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
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


static const char * version_str = "1.02 20160104";


#define PIO_BANKS_SAMA5D3 5  /* PA0-31, PB0-31, PC0-31, PD0-32 and PE0-31 */
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
 * SAMA5D3 base PIO addresses are 0xfffff200 (A), 0xfffff400 (B),
 * 0xfffff600 (C), 0xfffff800 (D) and 0xfffffa00 (E).
 */

#define SAMA5D3_PIO_WPKEY 0x50494f      /* "PIO" in ASCII */

struct opts_t {
    int do_pio;
    int peri_a;
    int peri_b;
    int peri_c;
    int peri_d;
    int enumerate;
    int en_fall_low;
    int en_ris_high;
    int en_output;
    int di_output;
    int en_glitch;
    int di_glitch;
    int en_interrupt;
    int di_interrupt;
    int en_edge;
    int en_level;
    int en_multi;
    int di_multi;
    int en_pullup;
    int di_pullup;
    int out_data;
    int en_schmitt;
    int di_schmitt;
    int en_if_slow_clk;
    int di_if_slow_clk;
    int wp_given;
    int wpen;
    int scdr_given;
    int scdr_div;
    int iod_given;
    int iod;
    int verbose;
};

struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};

static unsigned int pio_per[] = {0xfffff200, 0xfffff400, 0xfffff600,
                0xfffff800, 0xfffffa00};   /* PIO Enable Register */
static unsigned int pio_pdr[] = {0xfffff204, 0xfffff404, 0xfffff604,
                0xfffff804, 0xfffffa04};   /* PIO Disable Register */
static unsigned int pio_oer[] = {0xfffff210, 0xfffff410, 0xfffff610,
                0xfffff810, 0xfffffa10};   /* Output Enable Reg. */
static unsigned int pio_odr[] = {0xfffff214, 0xfffff414, 0xfffff614,
                0xfffff814, 0xfffffa14};   /* Output Disable Reg. */
static unsigned int pio_osr[] = {0xfffff218, 0xfffff418, 0xfffff618,
                0xfffff818, 0xfffffa18};   /* Output Status Reg. */
static unsigned int pio_ifer[] = {0xfffff220, 0xfffff420, 0xfffff620,
                0xfffff820, 0xfffffa20};  /* glitch Input Filter Ena */
static unsigned int pio_ifdr[] = {0xfffff224, 0xfffff424, 0xfffff624,
                0xfffff824, 0xfffffa24};  /* glitch Input Filter Dis */
static unsigned int pio_ier[] = {0xfffff240, 0xfffff440, 0xfffff640,
                0xfffff840, 0xfffffa40};   /* Interrupt Enable Reg. */
static unsigned int pio_idr[] = {0xfffff244, 0xfffff444, 0xfffff644,
                0xfffff844, 0xfffffa44};   /* Interrupt Disable Reg. */
static unsigned int pio_mder[] = {0xfffff250, 0xfffff450, 0xfffff650,
                0xfffff850, 0xfffffa50};  /* Multi-driver Enable */
static unsigned int pio_mddr[] = {0xfffff254, 0xfffff454, 0xfffff654,
                0xfffff854, 0xfffffa54};  /* Multi-driver Disable */
static unsigned int pio_puer[] = {0xfffff264, 0xfffff464, 0xfffff664,
                0xfffff864, 0xfffffa64};  /* Pull-Up Enable */
static unsigned int pio_pudr[] = {0xfffff260, 0xfffff460, 0xfffff660,
                0xfffff860, 0xfffffa60};  /* Pull-Up Disable */
static unsigned int pio_sodr[] = {0xfffff230, 0xfffff430, 0xfffff630,
                0xfffff830, 0xfffffa30};  /* Set Output Data Reg. */
static unsigned int pio_codr[] = {0xfffff234, 0xfffff434, 0xfffff634,
                0xfffff834, 0xfffffa34};  /* Clear Output Data Reg. */
static unsigned int pio_abcdsr1[] = {0xfffff270, 0xfffff470, 0xfffff670,
                0xfffff870, 0xfffffa70};/* Peripheral Select 1 */
static unsigned int pio_abcdsr2[] = {0xfffff274, 0xfffff474, 0xfffff674,
                0xfffff874, 0xfffffa74};/* Peripheral Select 2 */
static unsigned int pio_ppddr[] = {0xfffff290, 0xfffff490, 0xfffff690,
                0xfffff890, 0xfffffa90};  /* Pad Pull-Down Dis. */
static unsigned int pio_ppder[] = {0xfffff294, 0xfffff494, 0xfffff694,
                0xfffff894, 0xfffffa94};  /* Pad Pull-Down Ena. */
static unsigned int pio_aimer[] = {0xfffff2b0, 0xfffff4b0, 0xfffff6b0,
                0xfffff8b0, 0xfffffab0}; /* Additional Interrupt Modes Ena */
static unsigned int pio_aimdr[] = {0xfffff2b4, 0xfffff4b4, 0xfffff6b4,
                0xfffff8b4, 0xfffffab4}; /* Additional Interrupt Modes Dis */
static unsigned int pio_esr[] = {0xfffff2c0, 0xfffff4c0, 0xfffff6c0,
                0xfffff8c0, 0xfffffac0};   /* Edge Select Reg. */
static unsigned int pio_lsr[] = {0xfffff2c4, 0xfffff4c4, 0xfffff6c4,
                0xfffff8c4, 0xfffffac4};   /* Level Select Reg. */
static unsigned int pio_fellsr[] = {0xfffff2d0, 0xfffff4d0, 0xfffff6d0,
                0xfffff8d0, 0xfffffad0}; /* Falling Edge / Low Level Select */
static unsigned int pio_rehlsr[] = {0xfffff2d4, 0xfffff4d4, 0xfffff6d4,
                0xfffff8d4, 0xfffffad4}; /* Rising Edge / High Level Select */
static unsigned int pio_schmitt[] = {0xfffff300, 0xfffff500, 0xfffff700,
                0xfffff900, 0xfffffb00};
static unsigned int pio_ifscdr[] = {0xfffff280, 0xfffff480, 0xfffff680,
                0xfffff880, 0xfffffa80}; /* Input Filter Slow Clock Disable */
static unsigned int pio_ifscer[] = {0xfffff284, 0xfffff484, 0xfffff684,
                0xfffff884, 0xfffffa84}; /* Input Filter Slow Clock Enable */
static unsigned int pio_wpmr[] = {0xfffff2e4, 0xfffff4e4, 0xfffff6e4,
                0xfffff8e4, 0xfffffae4}; /* Write Protect Mode */
static unsigned int pio_scdr[] = {0xfffff28c, 0xfffff48c, 0xfffff68c,
                0xfffff88c, 0xfffffa8c}; /* Slow Clock Divider */
static unsigned int pio_driver1[] = {0xfffff314, 0xfffff514, 0xfffff714,
                        0xfffff914, 0xfffffb14}; /* IO drive 1 */
static unsigned int pio_driver2[] = {0xfffff318, 0xfffff518, 0xfffff718,
                        0xfffff918, 0xfffffb18}; /* IO drive 2 */


static void
usage(int help_val)
{
    if (1 == help_val)
        fprintf(stderr, "Usage: "
                "a5d3_pio_set [-b <bn>] [-d <div>] [-D <iod>] [-e] [-f|F] "
                "[-g|G]\n"
                "                    [-h] [-i|I] [-ii|II] [-l|L] [-m|M] "
                "[-o|O] [-p <port>]\n"
                "                    [-s <P_A_B_C_D>] [-S <n>] [-t|T] [-u|U] "
                "[-uu|UU]\n"
                "                    [-v] [-V] [-w <wpen>] [-z|Z]\n"
                "  where:\n"
                "    -b <bn>      bit number within port (0 to 31)."
                "Also\n"
                "                 accepts prefix like 'pb' or just 'b' for "
                "<port>.\n"
                "                 Example: '-b PC7' equivalent to '-p c "
                "-b 7'\n"
                "    -d <div>     slow clock divider [period=2*(<div>+1)"
                "*Tslow_clock]\n"
                "    -D <iod>     IO drive: 0->LO, 1->LO, 2->ME, 3->HI\n"
                "    -e           enumerate pin names with corresponding "
                "kernel pin\n"
                "    -f           set interrupt source as falling edge or "
                "low level event\n"
                "    -F           set interrupt source as rising edge or "
                "high level event\n"
                "    -g           disables glitch input filter\n"
                "    -G           enables glitch input filter\n"
                "    -h           print usage message; use twice for "
                "more help\n"
                "    -i           interrupt disable. Use twice to "
                "disable\n"
                "                 additional interrupt modes\n"
                "    -I           interrupt enable. Use twice to "
                "enable\n"
                "                 additional interrupt modes\n"
                "    -l           set interrupt source as edge event\n"
                "    -L           set interrupt source as level event\n"
                "    -m           disables multi-drive (open drain)\n"
                "    -M           enables multi-drive (open drain)\n"
                "    -o           output disable (make GPIO input only)\n"
                "    -O           output enable (in generic PIO mode only)\n"
                "    -p <port>    port bank ('A' to 'D') or gpio kernel "
                "line number\n"
                "    -s <P_A_B_C_D>    select line mode: P->PIO, "
                "A->peripheral_A\n"
                "                      B->peripheral_B, C or D)\n"
                "    -S <n>       set output data line to [<n> mod 2] (<n> "
                "is 0 to 3)\n"
                "                 if 0 or 1 then does an output enable if "
                "disabled\n"
                "    -t           disables Schmitt trigger on input\n"
                "    -T           enables Schmitt trigger on input\n"
                "    -u           disables internal pull-up. Use twice "
                "to disable\n"
                "                 internal pull-down\n"
                "    -U           enables internal pull-up. Use twice "
                "to enable\n"
                "                 internal pull-down. Example: switch PC12 "
                "from\n"
                "                 pull-up to pull-down: '-b PC12 -u -UU'\n"
                "    -v           increase verbosity (multiple times for "
                "more)\n"
                "    -V           print version string then exit\n"
                "    -w <wpen>    write protect mode (for whole PIO) set to "
                "<wpen>\n"
                "                 0->disabled (def, no write protection), "
                "1->enabled\n"
                "    -z           disables input filter slow clock\n"
                "    -Z           enables input filter slow clock\n\n"
                "Set SAMA5D3 SoCs PIO attributes. Uses memory mapped IO to "
                "access PIO\nregisters directly. Bypasses kernel.\n"
               );
    else    /* -hh */
        fprintf(stderr, "Usage: "
                "a5d3_pio_set [-b <bn>] [-d <div>] [-D <iod>] [-e] [-f|F] "
                "[-g|G]\n"
                "                    [-h] [-i|I] [-ii|II] [-l|L] [-m|M] "
                "[-o|O] "
                "[-p <port>]\n"
                "                    [-s <P_A_B_C_D>] [-S <n>] [-t|T] [-u|U] "
                "[-uu|UU]\n"
                "                    [-v] [-V] [-w <wpen>] [-z|Z]\n\n"
                "Use '-h' to get an explanation of the above command "
                "line options.\n\n"
                "Setting the output data line (e.g. with '-S 1') can fail "
                "to change the\nexternal line for several reasons. The line "
                "must be in GPIO/PIO mode\n('-s P'). Output must enabled "
                "('-O') but this will be set if '-S 0' or\n'-S 1' is given. "
                "If the line is set high (i.e. '-S 1') and if\nmulti-drive "
                "is enabled ('-M') then an internal or external pull-up is\n"
                "needed to see a high level on the external line.\n\n"
                "A line's internal pull-up and pull-down (resistor) cannot "
                "be active\n(enabled) at the same time. The policy of the "
                "SAMA5D3 is to discard any\nsubsequent conflicting enables. "
                "So the first to be enabled remains.\n"
               );
}

static inline volatile unsigned int *
mmp_add(unsigned char * p, unsigned int v)
{
    return (volatile unsigned int *)(p + v);
}

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
                fprintf(stderr, "mmap_ptr=%p:\n", msp->mmap_ptr);
                perror("    munmap");
                return NULL;
            } else if (op->verbose > 2)
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
        if (op->verbose > 2)
            fprintf(stderr, "mmap() ok, addr=0x%x, mask_addr=0x%lx, "
                    "mmap_ptr=%p\n", wanted_addr, mask_addr, msp->mmap_ptr);
    }
    return msp->mmap_ptr;
}

static int
pio_set(int mem_fd, int bit_num, int pioc_num, const struct opts_t * op)
{
    int sr1_val, sr2_val, res;
    unsigned int addr, val, bit_mask;
    volatile unsigned int * mmp;
    void * mmap_ptr = (void *)-1;
    struct mmap_state mstat;

    memset(&mstat, 0, sizeof(mstat));
    bit_mask = 1 << bit_num;

    if (op->peri_a || op->peri_b || op->peri_c || op->peri_d || op->do_pio) {
        if (op->do_pio) {
            mmap_ptr = check_mmap(mem_fd, pio_per[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_per[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        } else {
            if (op->peri_a) {
                sr1_val = 0;
                sr2_val = 0;
            } else if (op->peri_b) {
                sr1_val = 1;
                sr2_val = 0;
            } else if (op->peri_c) {
                sr1_val = 0;
                sr2_val = 1;
            } else {
                sr1_val = 1;
                sr2_val = 1;
            }
            res = 1;
            // Would like to take a lock here
            addr = pio_abcdsr1[pioc_num];
            mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
            if (NULL == mmap_ptr)
                goto err1_out;
            mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
            val = *mmp;
            if (!!(val & bit_mask) ^ sr1_val) {
                if (sr1_val)
                    val |= bit_mask;
                else
                    val &= ~bit_mask;
                *mmp = val;
            }
            addr = pio_abcdsr2[pioc_num];
            mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
            if (NULL == mmap_ptr)
                goto err1_out;
            mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
            val = *mmp;
            if (!!(val & bit_mask) ^ sr2_val) {
                if (sr2_val)
                    val |= bit_mask;
                else
                    val &= ~bit_mask;
                *mmp = val;
            }
            /* Now disable generic PIO action */
            mmap_ptr = check_mmap(mem_fd, pio_pdr[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                goto err1_out;
            mmp = mmp_add(mmap_ptr, pio_pdr[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
            res = 0;
err1_out:
            // Would like to free that lock here
            if (res)
                return res;
        }
    }
    if (op->di_schmitt || op->en_schmitt) {
        res = !!op->di_schmitt;
        addr = pio_schmitt[pioc_num];
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        val = *mmp;
        if (!!(val & bit_mask) ^ res) {
            if (res)
                val |= bit_mask;
            else
                val &= ~bit_mask;
            *mmp = val;
        }
    }
    if (op->scdr_given) {
        mmap_ptr = check_mmap(mem_fd, pio_scdr[pioc_num], &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_scdr[pioc_num] & MAP_MASK);
        *mmp = op->scdr_div;
    }
    if (op->di_if_slow_clk || op->en_if_slow_clk) {
        addr = (op->di_if_slow_clk ? pio_ifscdr[pioc_num] :
                                     pio_ifscer[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->di_glitch || op->en_glitch) {
        addr = (op->di_glitch ? pio_ifdr[pioc_num] : pio_ifer[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->di_interrupt) {
        if (1 & op->di_interrupt) {
            mmap_ptr = check_mmap(mem_fd, pio_idr[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_idr[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
        if (op->di_interrupt > 1) {
            mmap_ptr = check_mmap(mem_fd, pio_aimdr[pioc_num], &mstat,
                                  op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_aimdr[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
    }
    if (op->en_fall_low || op->en_ris_high) {
        addr = (op->en_fall_low ? pio_fellsr[pioc_num] :
                                  pio_rehlsr[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->en_edge || op->en_level) {
        addr = (op->en_edge ? pio_esr[pioc_num] : pio_lsr[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->en_interrupt) {
        if (1 & op->en_interrupt) {
            mmap_ptr = check_mmap(mem_fd, pio_ier[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_ier[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
        if (op->en_interrupt > 1) {
            mmap_ptr = check_mmap(mem_fd, pio_aimer[pioc_num], &mstat,
                                  op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_aimer[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
    }
    if (op->di_multi || op->en_multi) {
        addr = (op->di_multi ? pio_mddr[pioc_num] : pio_mder[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->di_output || op->en_output) {
        addr = (op->di_output ? pio_odr[pioc_num] : pio_oer[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
    }
    if (op->di_pullup) {
        if (1 & op->di_pullup) {
            mmap_ptr = check_mmap(mem_fd, pio_pudr[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_pudr[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
        if (op->di_pullup > 1) {
            mmap_ptr = check_mmap(mem_fd, pio_ppddr[pioc_num], &mstat,
                                  op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_ppddr[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
    }
    if (op->en_pullup) {
        if (1 & op->en_pullup) {
            mmap_ptr = check_mmap(mem_fd, pio_puer[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_puer[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
        if (op->en_pullup > 1) {
            mmap_ptr = check_mmap(mem_fd, pio_ppder[pioc_num], &mstat,
                                  op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_ppder[pioc_num] & MAP_MASK);
            *mmp = bit_mask;
        }
    }
    if (op->out_data >= 0) {
        addr = ((op->out_data % 2) ? pio_sodr[pioc_num] :
                                     pio_codr[pioc_num]);
        mmap_ptr = check_mmap(mem_fd, addr, &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, addr & MAP_MASK);
        *mmp = bit_mask;
        if (op->out_data < 2) {
            mmap_ptr = check_mmap(mem_fd, pio_osr[pioc_num], &mstat, op);
            if (NULL == mmap_ptr)
                return 1;
            mmp = mmp_add(mmap_ptr, pio_osr[pioc_num] & MAP_MASK);
            if (0 == (*mmp & bit_mask)) {
                mmap_ptr = check_mmap(mem_fd, pio_oer[pioc_num], &mstat, op);
                if (NULL == mmap_ptr)
                        return 1;
                mmp = mmp_add(mmap_ptr, pio_oer[pioc_num] & MAP_MASK);
                *mmp = bit_mask;
            }
        }
    }
    if (op->wp_given) {
        mmap_ptr = check_mmap(mem_fd, pio_wpmr[pioc_num], &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_wpmr[pioc_num] & MAP_MASK);
        *mmp = (SAMA5D3_PIO_WPKEY << 8) | op->wpen;
    }
    if (op->iod_given) {
        unsigned int * reg_arr;
        int bn;

        if (bit_num < 16) {
            reg_arr = pio_driver1;
            bn = bit_num;
        } else {
            reg_arr = pio_driver2;
            bn = bit_num & 0xf;
        }
        mmap_ptr = check_mmap(mem_fd, reg_arr[pioc_num], &mstat, op);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, reg_arr[pioc_num] & MAP_MASK);
        val = *mmp;
        if (op->verbose > 2)
            fprintf(stderr, "pio_driver%d read (before) val=0x%x\n",
                    (reg_arr == pio_driver2), val);
        val &= ~(0x3 << (2 * bn));
        if (op->iod > 0)
            val |= (op->iod << (2 * bn));
        *mmp = val;
        if (op->verbose > 2)
            fprintf(stderr, "pio_driver%d written (after) val=0x%x\n",
                    (reg_arr == pio_driver2), val);
    }

    if (mstat.mmap_ok) {
        if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
            fprintf(stderr, "mmap_ptr=%p:\n", mmap_ptr);
            perror("    munmap");
            return 1;
        } else if (op->verbose > 2)
            fprintf(stderr, "trailing munmap() ok, mmap_ptr=%p\n", mmap_ptr);
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
    char ch;
    char bank = '\0';
    struct opts_t opts;
    struct stat sb;

    memset(&opts, 0, sizeof(opts));
    opts.out_data = -1;
    while ((c = getopt(argc, argv,
                       "b:d:D:efFgGhiIlLmMoOp:Ps:S:tTuUvVw:zZ")) != -1) {
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
                    fprintf(stderr, "'-b' expects a letter ('A' to 'E')\n");
                    exit(EXIT_FAILURE);
                }
                ++cp;
            }
            k = atoi(cp);
            if ((k < 0) || (k > 31)) {
                fprintf(stderr, "'-b' expects a bit number from 0 to 31\n");
                exit(EXIT_FAILURE);
            }
            bit_num = k;
            break;
        case 'd':
            k = atoi(optarg);
            if ((k < 0) || (k > 16383)) {
                fprintf(stderr, "'-d' expects a bit number from 0 to "
                        "16383\n");
                return 1;
            }
            opts.scdr_div = k;
            ++opts.scdr_given;
            break;
        case 'D':
            k = atoi(optarg);
            if ((k < 0) || (k > 3)) {
                fprintf(stderr, "'-D' expects a bit number from 0 to "
                        "3\n");
                return 1;
            }
            opts.iod = k;
            ++opts.iod_given;
            break;
        case 'e':
            ++opts.enumerate;
            break;
        case 'f':
            ++opts.en_fall_low;
            break;
        case 'F':
            ++opts.en_ris_high;
            break;
        case 'g':
            ++opts.di_glitch;
            break;
        case 'G':
            ++opts.en_glitch;
            break;
        case 'h':
            ++do_help;
            break;
        case 'i':
            ++opts.di_interrupt;
            break;
        case 'I':
            ++opts.en_interrupt;
            break;
        case 'l':
            ++opts.en_edge;
            break;
        case 'L':
            ++opts.en_level;
            break;
        case 'm':
            ++opts.di_multi;
            break;
        case 'M':
            ++opts.en_multi;
            break;
        case 'o':
            ++opts.di_output;
            break;
        case 'O':
            ++opts.en_output;
            break;
        case 'p':
            if (isalpha(*optarg)) {
                ch = toupper(*optarg);
                if ((ch >= 'A') && (ch <= 'E'))
                    bank = ch;
                else {
                    fprintf(stderr, "'-p' expects a letter ('A' to 'E')\n");
                    return 1;
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 32) || (k > 191)) {
                    fprintf(stderr, "'-p' expects a letter or a number "
                            "32 or greater\n");
                    return 1;
                }
                knum = k;
            } else {
                fprintf(stderr, "'-p' expects a letter ('A' to 'D') or "
                        "a number\n");
                return 1;
            }
            break;
        case 'P':
            ++opts.do_pio;
            break;
        case 's':
            switch (optarg[0]) {
            case 'a': case 'A':
                ++opts.peri_a;
                break;
            case 'b': case 'B':
                ++opts.peri_b;
                break;
            case 'c': case 'C':
                ++opts.peri_c;
                break;
            case 'd': case 'D':
                ++opts.peri_d;
                break;
            case 'g': case 'G': case 'p': case 'P':
                ++opts.do_pio;
                break;
            default:
                fprintf(stderr, "'-s' expects 'P', 'A', 'B', 'C', or 'D'\n");
                return 1;
            }
            break;
        case 'S':
            if (! isdigit(*optarg)) {
                fprintf(stderr, "'-S' expects a value of 0 or 1\n");
                return 1;
            }
            k = atoi(optarg);
            if ((k < 0) || (k > 3)) {
                fprintf(stderr, "'-S' expects a value of 0, 1, 2 or 3\n");
                return 1;
            }
            opts.out_data = k;
            break;
        case 't':
            ++opts.di_schmitt;
            break;
        case 'T':
            ++opts.en_schmitt;
            break;
        case 'u':
            ++opts.di_pullup;
            break;
        case 'U':
            ++opts.en_pullup;
            break;
        case 'v':
            ++opts.verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            return 0;
        case 'w':
            k = atoi(optarg);
            if ((k < 0) || (k > 1)) {
                fprintf(stderr, "'-w' expects 0 (disabled) or 1 (enabled)\n");
                return 1;
            }
            ++opts.wp_given;
            opts.wpen = k;
            break;
        case 'z':
            ++opts.di_if_slow_clk;
            break;
        case 'Z':
            ++opts.en_if_slow_clk;
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
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            do_help = 1;
            help_exit = EXIT_FAILURE;
        }
    }
    if (do_help) {
        usage(do_help);
        exit(help_exit);
    }

    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (opts.verbose > 1)
            fprintf(stderr, "%s found so kernel pin numbers start at 0 "
                    "(for PA0)\n", GPIO_BANK_ORIGIN);
        ++origin0;
    } else if (opts.verbose > 2)
        fprintf(stderr, "%s not found so kernel pin numbers start at 32 "
                "(for PA0)\n", GPIO_BANK_ORIGIN);

    if (opts.enumerate) {
        num = PIO_BANKS_SAMA5D3;
        for (k = 0; k < LINES_PER_BANK; ++k) {
            for (j = 0; j < num; ++j) {
                n = ((j + (! origin0)) * 32) + k;
                printf("%sP%c%d: %d   ", (j ? "\t" : ""), 'A' + j, k, n);
            }
            printf("\n");
        }
        return 0;
    }

    if (knum >= 0) {
        if (bit_num >= 0) {
            fprintf(stderr, "Give either '-p <knum>' or ('-b <bn>' and "
                    "'-p <bank>') but not both\n");
            return 1;
        }
        if ((! origin0) && (knum < 32)) {
            fprintf(stderr, "since %s not found assume kernel pin numbers "
                    "start at 32\n(for PA0) so %d is too low\n",
                    GPIO_BANK_ORIGIN, knum);
            exit(EXIT_FAILURE);
        }
    } else if (bank) {
        if (bit_num < 0) {
            if (opts.wp_given || opts.scdr_given) {
                bit_num = 0;
                knum = ((! origin0) + bank - 'A') * 32;
            } else {
                fprintf(stderr, "If '-p <bank>' given then also need "
                        "'-b <bn>'\n");
                return 1;
            }
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        fprintf(stderr, "Need to give gpio line with '-p <port>' and/or "
                "'-b <bn>'\n");
        goto help_exit;
    }
    if ((!!opts.peri_a + !!opts.peri_b + !!opts.peri_c + !!opts.peri_d +
         !!opts.do_pio) > 1) {
        fprintf(stderr, "Can only have one of '-A', '-B', '-C', '-D', "
                "'-P' and '-s=<P_A_B_C_D>'\n");
        goto help_exit;
    }
    if ((!!opts.di_glitch + !!opts.en_glitch) > 1) {
        fprintf(stderr, "Can only have one of '-g' and '-G'\n");
        goto help_exit;
    }
    if (((1 == opts.di_interrupt) + (1 == opts.en_interrupt)) > 1) {
        fprintf(stderr, "Can only have one of '-i' and '-I'\n");
        goto help_exit;
    }
    if (((opts.di_interrupt > 1) + (opts.en_interrupt > 1)) > 1) {
        fprintf(stderr, "Can only have one of '-ii' and '-II'\n");
        goto help_exit;
    }
    if ((!!opts.di_multi + !!opts.en_multi) > 1) {
        fprintf(stderr, "Can only have one of '-m' and '-M'\n");
        goto help_exit;
    }
    if ((!!opts.di_output + !!opts.en_output) > 1) {
        fprintf(stderr, "Can only have one of '-o' and '-O'\n");
        goto help_exit;
    }
    if (((1 == opts.di_pullup) + (1 == opts.en_pullup)) > 1) {
        fprintf(stderr, "Can only have one of '-u' and '-U'\n");
        goto help_exit;
    }
    if (((opts.di_pullup > 1) + (opts.en_pullup > 1)) > 1) {
        fprintf(stderr, "Can only have one of '-uu' and '-UU'\n");
        goto help_exit;
    }
    if ((!!opts.en_fall_low + !!opts.en_ris_high) > 1) {
        fprintf(stderr, "Can only have one of '-f' and '-F'\n");
        goto help_exit;
    }
    if ((!!opts.en_edge + !!opts.en_level) > 1) {
        fprintf(stderr, "Can only have one of '-l' and '-L'\n");
        goto help_exit;
    }
    if ((!!opts.di_schmitt + !!opts.en_schmitt) > 1) {
        fprintf(stderr, "Can only have one of '-t' and '-T'\n");
        goto help_exit;
    }
    if ((!!opts.di_if_slow_clk + !!opts.en_if_slow_clk) > 1) {
        fprintf(stderr, "Can only have one of '-z' and '-Z'\n");
        goto help_exit;
    }

    pioc_num = bank ? (bank - 'A') : ((knum - (origin0 ? 0 : 32)) / 32);
    if (bit_num < 0)
        bit_num = knum % 32;
    if (opts.verbose)
        printf("P%c%d:\n", 'A' + pioc_num, bit_num);
    if (opts.verbose > 1)
        printf("  bit_mask=0x%08x\n", 1 << bit_num);

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (opts.verbose > 2)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");

    res = pio_set(mem_fd, bit_num, pioc_num, &opts);

    if (mem_fd >= 0)
        close(mem_fd);
    return res;

help_exit:
    fprintf(stderr, ">>> Use '-h' for command line syntax, '-hh' for "
            "other help.\n");
    return EXIT_FAILURE;
}
