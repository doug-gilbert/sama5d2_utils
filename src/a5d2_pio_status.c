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
 * a5d3_pio_status.c
 *
 * Utility for fetching SAMA5D3 family SoC PIO status values.
 * The SAMA5D3 family has 5 PIOs each with 32 gpio lines: PA0-PA31, PB0-PB31,
 * PC0-PC31, PD0-PD31 and PE0-PE31.
 * This utility uses memory mapped IO.
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


static const char * version_str = "1.01 20160104";


#define PIO_BANKS_SAMA5D3 5  /* PIOA, PIOB, PIOC, PIOD and PIOE */
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


struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};

static unsigned int pio_psr[] = {0xfffff208, 0xfffff408, 0xfffff608,
                        0xfffff808, 0xfffffa08}; /* PIO status */
static unsigned int pio_osr[] = {0xfffff218, 0xfffff418, 0xfffff618,
                        0xfffff818, 0xfffffa18}; /* Output status */
static unsigned int pio_ifsr[] = {0xfffff228, 0xfffff428, 0xfffff628,
                        0xfffff828, 0xfffffa28}; /* Input filter status */
                                /* a.k.a. Glitch input filter status */
static unsigned int pio_odsr[] = {0xfffff238, 0xfffff438, 0xfffff638,
                        0xfffff838, 0xfffffa38}; /* Output data status */
static unsigned int pio_pdsr[] = {0xfffff23c, 0xfffff43c, 0xfffff63c,
                        0xfffff83c, 0xfffffa3c}; /* Pin data status */
static unsigned int pio_imr[] = {0xfffff248, 0xfffff448, 0xfffff648,
                        0xfffff848, 0xfffffa48};  /* Interrupt mask */
static unsigned int pio_isr[] = {0xfffff24c, 0xfffff44c, 0xfffff64c,
                        0xfffff84c, 0xfffffa4c};  /* Interrupt status */
static unsigned int pio_mdsr[] = {0xfffff258, 0xfffff458, 0xfffff658,
                        0xfffff858, 0xfffffa58}; /* Multi drive status */
static unsigned int pio_pusr[] = {0xfffff268, 0xfffff468, 0xfffff668,
                        0xfffff868, 0xfffffa68}; /* Pull-up status */
                                /* a.k.a. Pad pull-up status */
static unsigned int pio_abcdsr1[] = {0xfffff270, 0xfffff470, 0xfffff670,
                        0xfffff870, 0xfffffa70}; /* Peripheral select 1 */
static unsigned int pio_abcdsr2[] = {0xfffff274, 0xfffff474, 0xfffff674,
                        0xfffff874, 0xfffffa74}; /* Peripheral select 2 */
static unsigned int pio_ifscsr[] = {0xfffff288, 0xfffff488, 0xfffff688,
                        0xfffff888, 0xfffffa88}; /* Input filter slow
                                                    clock status */
static unsigned int pio_scdr[] = {0xfffff28c, 0xfffff48c, 0xfffff68c,
                        0xfffff88c, 0xfffffa8c}; /* Slow clock divider
                                                    debouncing */
static unsigned int pio_ppdsr[] = {0xfffff298, 0xfffff498, 0xfffff698,
                        0xfffff898, 0xfffffa98}; /* Pad pull-down status */
static unsigned int pio_owsr[] = {0xfffff2a8, 0xfffff4a8, 0xfffff6a8,
                        0xfffff8a8, 0xfffffaa8}; /* Output write status */
static unsigned int pio_aimmr[] = {0xfffff2b8, 0xfffff4b8, 0xfffff6b8,
                        0xfffff8b8, 0xfffffab8}; /* Additional interrupt
                                                    modes mask */
static unsigned int pio_elsr[] = {0xfffff2c8, 0xfffff4c8, 0xfffff6c8,
                        0xfffff8c8, 0xfffffac8}; /* Edge/level status */
static unsigned int pio_frlhsr[] = {0xfffff2d8, 0xfffff4d8, 0xfffff6d8,
                        0xfffff8d8, 0xfffffad8}; /* Fall/rise - low/high
                                                    status */
static unsigned int pio_locksr[] = {0xfffff2e0, 0xfffff4e0, 0xfffff6e0,
                        0xfffff8e0, 0xfffffae0}; /* Lock status */
static unsigned int pio_wpmr[] = {0xfffff2e4, 0xfffff4e4, 0xfffff6e4,
                        0xfffff8e4, 0xfffffae4}; /* Write protect mode */
static unsigned int pio_wpsr[] = {0xfffff2e8, 0xfffff4e8, 0xfffff6e8,
                        0xfffff8e8, 0xfffffae8}; /* Write protect status */
static unsigned int pio_schmitt[] = {0xfffff300, 0xfffff500, 0xfffff700,
                        0xfffff900, 0xfffffb00}; /* Schmitt trigger */
static unsigned int pio_driver1[] = {0xfffff314, 0xfffff514, 0xfffff714,
                        0xfffff914, 0xfffffb14}; /* IO drive 1 */
static unsigned int pio_driver2[] = {0xfffff318, 0xfffff518, 0xfffff718,
                        0xfffff918, 0xfffffb18}; /* IO drive 2 */

static int verbose = 0;

static const char * driv_arr[] = {"LO_DRIVE", "LO_DRIVE", "ME_DRIVE",
                                  "HI_DRIVE"};


static const char * pioa_trans[3 * 32] = {
    /* triples: 0 */ "LCDDAT0", NULL, NULL,  "LCDDAT1", NULL, NULL,
    "LCDDAT2", NULL, NULL,  "LCDDAT3", NULL, NULL,
    "LCDDAT4", NULL, NULL,  "LCDDAT5", NULL, NULL,
    "LCDDAT6", NULL, NULL,  "LCDDAT7", NULL, NULL,
    /* triples: 8 */ "LCDDAT8", NULL, NULL,  "LCDDAT9", NULL, NULL,
    "LCDDAT10", NULL, NULL,  "LCDDAT11", NULL, NULL,
    "LCDDAT12", NULL, NULL,  "LCDDAT13", NULL, NULL,
    "LCDDAT14", NULL, NULL,  "LCDDAT15", NULL, NULL,
    /* triples: 16 */ "LCDDAT16", NULL, "ISI_D0",
    "LCDDAT17", NULL, "ISI_D1",  "LCDDAT18", "TWD2", "ISI_D2",
    "LCDDAT19", "TWCK2", "ISI_D3",  "LCDDAT20", "PWMH0", "ISI_D4",
    "LCDDAT21", "PWML0", "ISI_D5",  "LCDDAT22", "PWMH1", "ISI_D6",
    "LCDDAT23", "PWML1", "ISI_D7",
    /* triples: 24 */ "LCDPWM", NULL, NULL,  "LCDDISP", NULL, NULL,
    "LCDVSYNC", NULL, NULL,  "LCDHSYNC", NULL, NULL,  "LCDPCK", NULL, NULL,
    "LCDEN", NULL, NULL,  "TWD0", "URXD1", "ISI_VSYNC",
    "TWCK0", "UTXD1", "ISI_HSYNC", };

static const char * piob_trans[3 * 32] = {
    /* triples: 0 */ "GTX0", "PWMH0", NULL,  "GTX1", "PWML1", NULL,
    "GTX2", "TK1", NULL,  "GTX3", "TF1", NULL,
    "GRX0", "PWMH1", NULL,  "GRX1", "PWML1", NULL,  "GRX2", "TD1", NULL,
    "GRX3", "RK1", NULL,
    /* triples: 8 */ "GTXCK", "PWMH2", NULL,  "GTXEN", "PWML2", NULL,
    "GTXER", "RF1", NULL,  "GRXCK", "RD1", NULL,
    "GRXDV", "PWMH3", NULL,  "GRXER", "PWML3", NULL,
    "GCRS", "CANRX1", NULL,  "GCOL", "CANTX1", NULL,
    /* triples: 16 */ "GMDC", NULL, NULL,  "GMDIO", NULL, NULL,
    "G125CK", NULL, NULL,  "MCI1_CDA", "GTX4", NULL,
    "MCI1_DA0", "GTX5", NULL,  "MCI1_DA1", "GTX6", NULL,
    "MCI1_DA2", "GTX7", NULL,  "MCI1_DA3", "GRX4", NULL,
    /* triples: 24 */ "MCI1_CK", "GRX5", NULL,  "SCK1", "GRX6", NULL,
    "CTS1", "GRX7", NULL,  "RTS1", "G125CKO", NULL,  "RXD1", NULL, NULL,
    "TXD1", NULL, NULL,  "DRXD", NULL, NULL,
    "DTXD", NULL, NULL, };

static const char * pioc_trans[3 * 32] = {
    /* triples: 0 */ "ETX0", "TIOA3", NULL,  "ETX1", "TIOB3", NULL,
    "ERX0", "TCLK3", NULL,  "ERX1", "TIOA4", NULL,
    "ETXEN", "TIOB4", NULL,  "ECRSDV", "TCLK4", NULL,
    "ERXER", "TIOA5", NULL, "EREFCK", "TIOB5", NULL,
    /* triples: 8 */ "EMDC", "TCLK5", NULL,  "EMDIO", NULL, NULL,
    "MCI2_CDA", NULL, "LCDDAT20",  "MCI2_DA0", NULL, "LCDDAT19",
    "MCI2_DA1", "TIOA1", "LCDDAT18",  "MCI2_DA2", "TIOB1", "LCDDAT17",
    "MCI2_DA3", "TCLK1", "LCDDAT16",  "MCI2_CK", "PCK2", "LCDDAT21",
    /* triples: 16 */ "TK0", NULL, NULL,  "TF0", NULL, NULL,
    "TD0", NULL, NULL,  "RK0", NULL, NULL,  "RF0", NULL, NULL,
    "RD0", NULL, NULL,  "SPI1_MISO", NULL, NULL,
    "SPI1_MOSI", NULL, NULL,
    /* triples: 24 */ "SPI1_SPCK", NULL, NULL,  "SPI1_NPCS0", NULL, NULL,
    "SPI1_NPCS1", "TWD1", "ISI_D11",  "SPI1_NPCS2", "TWCK1", "ISI_D10",
    "SPI1_NPCS3", "PWMFI0", "ISI_D9",  "URXD0", "PWMFI2", "ISI_D8",
    "UTXD0", NULL, "ISI_PCK",  "FIQ", "PWMFI1", NULL, };

static const char * piod_trans[3 * 32] = {
    /* triples: 0 */ "MCI0_CDA", NULL, NULL,  "MCI0_D0", NULL, NULL,
    "MCI0_D1", NULL, NULL,  "MCI0_D2", NULL, NULL,
    "MCI0_D3", NULL, NULL,  "MCI0_D4", "TIOA0", "PWMH2",
    "MCI0_D5", "TIOB0", "PWML2",  "MCI0_D6", "TCLK0", "PWMH3",
    /* triples: 8 */ "MCI0_D7", NULL, "PWML3",  "MCI0_CK", NULL, NULL,
    "SPIO_MISO", NULL, NULL,  "SPI0_MOSI", NULL, NULL,
    "SPI0_SPCK", NULL, NULL,  "SPI0_NPCS0", NULL, NULL,
    "SCK0", "SPI0_NPCS1", "CANRX0",  "CTS0", "SPI0_NPCS2", "CANTX0",
    /* triples: 16 */ "RTS0", "SPI0_NPCS3", "PWMFI3",  "RXD0", NULL, NULL,
    "TXD0", NULL, NULL,  "ADTRG", NULL, NULL,  "AD0", NULL, NULL,
    "AD1", NULL, NULL,  "AD2", NULL, NULL,
    "AD3", NULL, NULL,
    /* triples: 24 */ "AD4", NULL, NULL,  "AD5", NULL, NULL,
    "AD6", NULL, NULL,  "AD7", NULL, NULL,  "AD8", NULL, NULL,
    "AD9", NULL, NULL,  "AD10", "PCK0", NULL,
    "AD11", "PCK1", "ETXEN", };

static const char * pioe_trans[3 * 32] = {
    /* triples: 0 */ "A0/NBS0", NULL, NULL,  "A1", NULL, NULL,
    "A2", NULL, NULL,  "A3", NULL, NULL,
    "A4", NULL, NULL,  "A5", NULL, NULL,  "A6", NULL, NULL,
    "A7", NULL, NULL,
    /* triples: 8 */ "A8", NULL, NULL,  "A9", NULL, NULL,
    "A10", NULL, NULL,  "A11", NULL, NULL,
    "A12", NULL, NULL,  "A13", NULL, NULL,
    "A14", NULL, NULL,  "A15", "SCK3", NULL,
    /* triples: 16 */ "A16", "CTS3", NULL,  "A17", "RTS3", NULL,
    "A18", "RXD3", NULL,  "A19", "TXD3", NULL,  "A20", "SCK2", NULL,
    "A21/NANDALE", NULL, NULL,  "A22/NANDCLE", NULL, NULL,
    "A23", "CTS2", NULL,
    /* triples: 24 */ "A24", "RTS2", NULL,  "A25", "RXD2", NULL,
    "NCS0", "TXD2", NULL,  "NCS1", "TIOA2", "LCDDAT22",
    "NCS2", "TIOB2", "LCDDAT23",  "NWR1/NBS1", "TCLK2", NULL,
    "NWAIT", NULL, NULL,  "IRQ", "PWML1", NULL, };



static void
usage(int hval)
{
    if (1 == hval) {
        fprintf(stderr, "Usage: "
                "a5d3_pio_status [-a] [-b <bn>] [-B] [-e] [-h] [-i]\n"
                "                       [-p <port>] [-s] [-S] [-t] [-v] "
                "[-V] [-w]\n"
                "  where:\n"
                "    -a           list all lines within a bank (def: "
                "'-p A')\n"
                "    -b <bn>      bit (line) number within port (0 to 31). "
                "Also\n"
                "                 accepts prefix like 'pb' or just 'b' for "
                "<port>.\n"
                "                 Example: '-b PC7' equivalent to '-p c "
                "-b 7'\n"
                "    -B           brief ouput (e.g. 'psr=1 pusr*=0 ...'). "
                "Use twice\n"
                "                 for single line output; thrice for name "
                "only\n"
                "    -e           enumerate pin names with corresponding "
                "kernel pin.\n"
                "                 Use twice to list peripheral names for "
                "each pin\n"
                "    -h           print usage message, use twice for "
                "abbreviations\n"
                "    -i           read interrupt status register which "
                "clears it\n"
                "    -p <port>    port bank ('A' to 'E') or gpio kernel "
                "line number\n"
                "                 0 -> PA0, 1 -> PA1 ... 159 -> PE31\n"
                "    -s           summarize all lines in a bank, equivalent "
                "to\n"
                "                 '-a -BB -t'. Example: 'a5d3_pio_status "
                "-s -p C'\n"
                "    -S           show all selected line names within all "
                "banks.\n"
                "                 Use twice for appended '^' indicating "
                "pull-up\n"
                "                 Use thrice to indicate multi-drive\n"
                "    -t           translate peripheral type to functional "
                "name\n"
                "                 (e.g. PD15 peri_a -> CTS0)\n"
                "    -v           increase verbosity (multiple times for "
                "more)\n"
                "    -V           print version string then exit\n");
        fprintf(stderr,
                "    -w           read write protect status register "
                "which clears it\n");
        fprintf(stderr,
                "\nSAMA5D3x SoC PIO fetch status program. "
                "Uses memory mapped\nIO to fetch PIO registers and shows "
                "settings for given line(s). Try\n'-hh' for more help.\n");
    } else {
        fprintf(stderr, ">> Abbreviation explanations\n"
                "abcdsr1:  peripheral select 1 [def: 0]\n"
                "abcdsr2:  peripheral select 2 [def: 0 -> peri_a (if "
                "abcdsr1=0)]\n"
                "aimmr:    additional interrupt mode mask [def: 0 -> "
                "disabled]]\n"
                "driver1:  IO drive 1 [def: 0]\n"
                "driver2:  IO drive 2 [def: 0 -> HI (if driver1=0)]\n"
                "elsr:     edge/level status [def: 0 -> edge]\n"
                "frlhsr:   fall/rise - low/high status [def: 0 -> "
                "falling or low]\n"
                "ifsr:     input (glitch) filter status [def: 0 -> "
                "disabled]\n"
                "imr:      interrupt mask [def: 0 -> disabled]\n"
                "isr:      interrupt status [def: 0 -> no change]\n"
                "locksr:   lock status [def: 0 -> unlocked]\n"
                "mdsr:     multi drive status [def: 0 -> disabled: "
                "driven high+low]\n"
                "odsr:     output data status [def: 0 -> level 0 to "
                "be driven]\n"
                "osr:      output status [def: 0 -> line is pure "
                "input]\n"
                "owsr:     output write status [def: 0 -> writing to "
                "odsr ignored]\n"
                "pdsr:     pin data status [0 -> line is at level 0; "
                "1 -> level 1]\n"
                "ppdsr*:   pad pull-down status [def: 1 -> disabled]\n"
                "psr:      PIO status [0 -> peripheral active; 1 -> "
                "generic GPIO pin]\n"
                "pusr*:    pad pull-up status [def: 0 -> enabled]\n"
                "scdr**:   slow clock divider (debouncing) [def: 0; per "
                "PIO]\n"
                "schmitt*: schmitt trigger [def: 0 -> enabled]\n"
                "wpmr**:  write protect mask [def: 0 -> PIO writeable]\n"
                "wpsr**:   write protect status [def: 0 -> no violation "
                "on PIO]\n"
               );
        fprintf(stderr, "\nAbbreviations with a trailing '*' have "
                "the corresponding function\nenabled when the value "
                "is 0 (i.e. negated logic). For example\n'pusr*=0' "
                "means the pull-up is enabled. The trailing '**' "
                "means\nthe register is per PIO bank rather than per "
                "GPIO line. An\nentry like 'isr=-1' means that isr "
                "(the interrupt status register)\nhas not been read.\n"
               );
    }
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

static char *
translate_peri(char * b, int max_blen, int pioc_num, int bit_num,
               int peri_num)
{
    const char * cp;

    if ((NULL == b) || (max_blen < 1))
        return b;
    b[max_blen - 1] = '\0';
    if ((bit_num < 0) || (bit_num > 31)) {
        snprintf(b, max_blen - 1, "bad bit_num=%d", bit_num);
        return b;
    }
    if ((peri_num < 0) || (peri_num > 3)) {
        snprintf(b, max_blen - 1, "bad peri_num=%d", peri_num);
        return b;
    }
    switch (pioc_num) {
    case 0:
        if ((peri_num < 3) &&
            (cp = pioa_trans[(3 * bit_num) + peri_num]))
            snprintf(b, max_blen - 1, "%s", cp);
        else
            strncpy(b, "", max_blen - 1);
        return b;
    case 1:
        if ((peri_num < 3) &&
            (cp = piob_trans[(3 * bit_num) + peri_num]))
            snprintf(b, max_blen - 1, "%s", cp);
        else
            strncpy(b, "", max_blen - 1);
        return b;
    case 2:
        if ((peri_num < 3) &&
            (cp = pioc_trans[(3 * bit_num) + peri_num]))
            snprintf(b, max_blen - 1, "%s", cp);
        else
            strncpy(b, "", max_blen - 1);
        return b;
    case 3:
        if ((peri_num < 3) &&
            (cp = piod_trans[(3 * bit_num) + peri_num]))
            snprintf(b, max_blen - 1, "%s", cp);
        else
            strncpy(b, "", max_blen - 1);
        return b;
    case 4:
        if ((peri_num < 3) &&
            (cp = pioe_trans[(3 * bit_num) + peri_num]))
            snprintf(b, max_blen - 1, "%s", cp);
        else
            strncpy(b, "", max_blen - 1);
        return b;
    default:
        snprintf(b, max_blen - 1, "bad pioc_num=%d", pioc_num);
        return b;
    }
}

static int
pio_status(int mem_fd, unsigned int bit_mask, int bit_num, int brief,
           int interrupt, int translate, int pioc_num, int write_prot)
{
    int psr, osr, ifsr, odsr, pdsr, imr, isr, mdsr, pusr, abcdsr1, abcdsr2;
    int owsr, ifscsr, scdr, ppdsr, aimmr, elsr, frlhsr, locksr, wpmr, wpsr;
    int schmitt, bn, driv, k;
    void * mmap_ptr = (void *)-1;
    struct mmap_state mstat;
    volatile unsigned int * mmp;
    unsigned int * reg_arr;
    const char * cp;
    char b[32];


    memset(&mstat, 0, sizeof(mstat));

    mmap_ptr = check_mmap(mem_fd, pio_psr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_psr[pioc_num] & MAP_MASK);
    psr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  PIO status: %d (%s)\n", psr,
               (psr ? "PIO ACTIVE, peripheral inactive" :
                      "peripheral ACTIVE"));

    mmap_ptr = check_mmap(mem_fd, pio_osr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_osr[pioc_num] & MAP_MASK);
    osr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  PIO output status: %d (%s)\n", osr,
               (osr ? "line enabled as output" : "line pure input"));

    mmap_ptr = check_mmap(mem_fd, pio_ifsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_ifsr[pioc_num] & MAP_MASK);
    ifsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  input filter status: %d (%s)\n", ifsr,
               (ifsr ? "glitch filter ENabled" : "glitch filter DISabled"));

    mmap_ptr = check_mmap(mem_fd, pio_odsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_odsr[pioc_num] & MAP_MASK);
    odsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  output data status: %d\n", odsr);

    mmap_ptr = check_mmap(mem_fd, pio_pdsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_pdsr[pioc_num] & MAP_MASK);
    pdsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  pin data status: %d\n", pdsr);

    mmap_ptr = check_mmap(mem_fd, pio_imr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_imr[pioc_num] & MAP_MASK);
    imr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  interrupt mask: %d (%s)\n", imr,
               (imr ? "ENabled" : "DISabled"));

    if (interrupt) {
        mmap_ptr = check_mmap(mem_fd, pio_isr[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_isr[pioc_num] & MAP_MASK);
        isr = !!(*mmp & bit_mask);
        if (0 == brief)
            printf("  interrupt status: %d (%s)\n", isr,
                   (isr ? "input CHANGE" : "NO input change"));
    } else
        isr = -1;

    mmap_ptr = check_mmap(mem_fd, pio_mdsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_mdsr[pioc_num] & MAP_MASK);
    mdsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  multi driver status: %d (%s)\n", mdsr,
               (mdsr ? "enabled, pin driven when low" :
               "pin driven high and low"));

    mmap_ptr = check_mmap(mem_fd, pio_pusr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_pusr[pioc_num] & MAP_MASK);
    pusr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  pad pull-up status: %d (%s)\n", pusr,
               (pusr ? "DISabled" : "ENabled"));

    if (psr) {
        abcdsr1 = -1;
        abcdsr2 = -1;
        mmap_ptr = check_mmap(mem_fd, pio_owsr[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_owsr[pioc_num] & MAP_MASK);
        owsr = !!(*mmp & bit_mask);
        if (0 == brief)
            printf("  output write status: %d (%s)\n", owsr,
                   (owsr ? "writing PIO_ODSR affects IO line" :
                           "writing PIO_ODSR ignored"));
    } else {
        /* peripheral mode */
        owsr = -1;
        mmap_ptr = check_mmap(mem_fd, pio_abcdsr1[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_abcdsr1[pioc_num] & MAP_MASK);
        abcdsr1 = !!(*mmp & bit_mask);
        mmap_ptr = check_mmap(mem_fd, pio_abcdsr2[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_abcdsr2[pioc_num] & MAP_MASK);
        abcdsr2 = !!(*mmp & bit_mask);
        if (0 == brief) {
            cp = NULL;
            k = (2 * abcdsr2) + abcdsr1;
            if (translate) {
                b[0] = '\0';
                translate_peri(b, sizeof(b), pioc_num, bit_num, k);
                if (strlen(b) > 0)
                    cp = b;
            }
            if (cp)
                printf("  %s [sr1:%d, sr2:%d]\n", cp, abcdsr1, abcdsr2);
            else
                printf("  peripheral %s function [sr1:%d, sr2:%d]\n",
                       (abcdsr2 ? (abcdsr1 ? "D" : "C") :
                                  (abcdsr1 ? "B" : "A")),
                       abcdsr1, abcdsr2);
        }
    }
    mmap_ptr = check_mmap(mem_fd, pio_ifscsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_ifscsr[pioc_num] & MAP_MASK);
    ifscsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  input filter slow clock status: %d (%s)\n", ifscsr,
               (ifscsr ? "slow" : "master-fast"));
    if (ifscsr) {
        mmap_ptr = check_mmap(mem_fd, pio_scdr[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_scdr[pioc_num] & MAP_MASK);
        scdr = *mmp & 0x3fff;
        if (0 == brief)
            printf("  slow clock divider debouncing register: %d [0x%x]\n",
                   scdr, scdr);
    } else
        scdr = -1;
    mmap_ptr = check_mmap(mem_fd, pio_ppdsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_ppdsr[pioc_num] & MAP_MASK);
    ppdsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  pad pull down status: %d (%s)\n", ppdsr,
               (ppdsr ? "DISabled" : "ENabled"));
    mmap_ptr = check_mmap(mem_fd, pio_aimmr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_aimmr[pioc_num] & MAP_MASK);
    aimmr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  additional interrupt modes mask: %d (%s)\n", aimmr,
               (aimmr ? "depends on ELSR+FRLHSR" : "Both edges"));
    mmap_ptr = check_mmap(mem_fd, pio_elsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_elsr[pioc_num] & MAP_MASK);
    elsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  edge/level status: %d (%s detection)\n", elsr,
               (elsr ? "level" : "edge"));
    mmap_ptr = check_mmap(mem_fd, pio_frlhsr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_frlhsr[pioc_num] & MAP_MASK);
    frlhsr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  fall/rise - low/high status: %d (%s detection)\n", elsr,
               (elsr ? "rising edge or high level" :
                       "falling edge or low level"));
    mmap_ptr = check_mmap(mem_fd, pio_locksr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_locksr[pioc_num] & MAP_MASK);
    locksr = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  locked status: %d (%slocked)\n", locksr,
               (locksr ? "" : "not "));
    mmap_ptr = check_mmap(mem_fd, pio_wpmr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_wpmr[pioc_num] & MAP_MASK);
    wpmr = *mmp;
    if (0 == brief)
        printf("  write protect mode: WPEN: %d (%s)\n",
               wpmr & 1, ((wpmr & 1) ? "ENabled" : "DISabled"));
    if (write_prot) {
        mmap_ptr = check_mmap(mem_fd, pio_wpsr[pioc_num], &mstat);
        if (NULL == mmap_ptr)
            return 1;
        mmp = mmp_add(mmap_ptr, pio_wpsr[pioc_num] & MAP_MASK);
        wpsr = *mmp & 0xffffff;
        if (0 == brief)
            printf("  write protect violation status: %d (%s), WPCSRC: "
                   "0x%x\n", (wpsr & 1), ((wpsr & 1) ? "VIOLATED" :
                   "NOT violated"), (wpsr >> 8) & 0xffff);
    } else
        wpsr = -1;
    mmap_ptr = check_mmap(mem_fd, pio_schmitt[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, pio_schmitt[pioc_num] & MAP_MASK);
    schmitt = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  schmitt trigger status: %d (%s)\n", schmitt,
               (schmitt ? "DISabled" : "ENabled "));
    if (bit_num < 16) {
        reg_arr = pio_driver1;
        bn = bit_num;
    } else {
        reg_arr = pio_driver2;
        bn = bit_num & 0xf;
    }
    mmap_ptr = check_mmap(mem_fd, reg_arr[pioc_num], &mstat);
    if (NULL == mmap_ptr)
        return 1;
    mmp = mmp_add(mmap_ptr, reg_arr[pioc_num] & MAP_MASK);
    driv = (*mmp >> (2 * bn)) & 0x3;
    if (0 == brief)
        printf("  IO drive: %d (%s)\n", driv, driv_arr[driv]);

    if (1 == brief) {
        if (psr) {
            printf("psr=%d pdsr=%d osr=%d odsr=%d mdsr=%d ifsr=%d imr=%d "
                   "isr=%d pusr*=%d ifscsr=%d\nscdr=%d ppdsr*=%d aimmr=%d "
                   "elsr=%d frlhsr=%d locksr=%d wpmr=0x%x", psr, pdsr, osr,
                   odsr, mdsr, ifsr, imr, isr, pusr, ifscsr, scdr, ppdsr,
                   aimmr, elsr, frlhsr, locksr, wpmr);
            if (write_prot)
                printf(" wpsr=0x%x", wpsr);
            printf("\nschmitt*=%d io_driv=%d\n", schmitt, driv);
        } else {
            printf("psr=%d pdsr=%d osr=%d odsr=%d mdsr=%d ifsr=%d imr=%d "
                   "isr=%d pusr*=%d abcdsr1=%d\nabcdsr2=%d owsr=%d "
                   "ifscsr=%d scdr=%d ppdsr*=%d aimmr=%d elsr=%d "
                   "frlhsr=%d\nlocksr=%d wpmr=0x%x", psr, pdsr, osr, odsr,
                   mdsr, ifsr, imr, isr, pusr, abcdsr1, abcdsr2, owsr,
                   ifscsr, scdr, ppdsr, aimmr, elsr, frlhsr, locksr, wpmr);
            if (write_prot)
                printf(" wpsr=0x%x", wpsr);
            printf(" schmitt*=%d io_driv=%d\n", schmitt, driv);
        }
    } else if (brief > 1) {
        if (psr) {
            if (translate && (1 == pioc_num) && (bit_num >= 6) &&
                (bit_num < 18)) {
                k = (bit_num + 1) % 12;
                snprintf(b, sizeof(b), "AD%d", k);
                cp = b;
            } else
                cp = "GPIO";
        } else {
            if (abcdsr1 & abcdsr2) {
                cp = "PERI_D";
                k = 3;
            } else if (abcdsr1) {
                cp = "PERI_B";
                k = 1;
            } else if (abcdsr2) {
                cp = "PERI_C";
                k = 2;
            } else {
                cp = "PERI_A";
                k = 0;
            }
            if (translate) {
                b[0] = '\0';
                translate_peri(b, sizeof(b), pioc_num, bit_num, k);
                if (strlen(b) > 0)
                    cp = b;
            }
        }
        if (2 == brief) {
            if (psr)
                printf(" %-2d: %s pdsr=%d osr=%d odsr=%d mdsr=%d ifsr=%d "
                       "pusr*=%d%s\n", bit_num, cp, pdsr, osr, odsr, mdsr,
                       ifsr, pusr, ((0 == ppdsr) ? " ppdsr*=0" : ""));
            else
                printf(" %-2d: %s pdsr=%d mdsr=%d ifsr=%d pusr*=%d%s\n",
                       bit_num, cp, pdsr, mdsr, ifsr, pusr,
                       ((0 == ppdsr) ? " ppdsr*=0" : ""));
        } else
            printf("\n");
    }

    if (mstat.mmap_ok) {
        if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
            fprintf(stderr, "mmap_ptr=%p:\n", mmap_ptr);
            perror("    munmap");
            return 1;
        } else if (verbose > 2)
            fprintf(stderr, "trailing munmap() ok, mmap_ptr=%p\n", mmap_ptr);
    }
    return 0;
}

static int
do_enumerate(int enum_val, int bank, int orig0)
{
    int k, j, n, num;
    char b[16];
    const char * cp;

    num = PIO_BANKS_SAMA5D3;
    if (1 == enum_val) {
        for (k = 0; k < LINES_PER_BANK; ++k) {
            for (j = 0; j < num; ++j) {
                n = ((j + (! orig0)) * 32) + k;
                printf("%sP%c%d: %d   ", (j ? "\t" : ""), 'A' + j,
                       k, n);
            }
            printf("\n");
        }
    } else { /* '-ee' */
        if ('\0' == bank)
            k = 0;
        else {
            k = bank - 'A';
            if (k < num)
                num = k + 1;
        }
        for ( ; k < num; ++k) {
            printf("SAMA5D3: PIO %c:\n", 'A' + k);
            for (j = 0; j < LINES_PER_BANK; ++j) {
                cp = translate_peri(b, sizeof(b), k, j, 0);
                printf("  P%c%d: %s, ", 'A' + k, j,
                       (strlen(cp) > 0) ? cp : "-");
                cp = translate_peri(b, sizeof(b), k, j, 1);
                printf("%s, ", (strlen(cp) > 0) ? cp : "-");
                cp = translate_peri(b, sizeof(b), k, j, 2);
                printf("%s\n", (strlen(cp) > 0) ? cp : "-");
            }
        }
    }
    return 0;
}

static int
do_show_all(int show_val)
{
    int k, j, n, num, psr, abcdsr1, abcdsr2, v;
    int res = 1;
    int mem_fd = -1;
    unsigned int bit_mask, addr;
    char b[32];
    size_t blen = sizeof(b) - 1;
    void * mmap_ptr = (void *)-1;
    struct mmap_state mstat;
    void * ap;

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose > 2)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");
    memset(&mstat, 0, sizeof(mstat));
    if (2 == show_val)
        printf("Appended '^' indicates internal pull-up active\n\n");
    else if (show_val > 2)
        printf("Appended '^' indicates multi-drive (open drain) "
               "mode\n\n");

    num = PIO_BANKS_SAMA5D3;
    printf("PIN   PIO_A        PIO_B        PIO_C        PIO_D        "
           "PIO_E\n");
    for (k = 0; k < LINES_PER_BANK; ++k) {
        if (k > 9)
            printf("%d:   ", k);
        else
            printf("%d:    ", k);
        bit_mask = 1 << k;
        for (j = 0; j < num; ++j) {
            mmap_ptr = check_mmap(mem_fd, pio_psr[j], &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (pio_psr[j] & MAP_MASK);
            psr = !!( *((unsigned int *)ap) & bit_mask);
            if (psr)
                snprintf(b, blen, "%s", "GPIO");
            else {
                mmap_ptr = check_mmap(mem_fd, pio_abcdsr1[j],
                                      &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr +
                     (pio_abcdsr1[j] & MAP_MASK);
                abcdsr1 = !!( *((unsigned int *)ap) & bit_mask);
                mmap_ptr = check_mmap(mem_fd, pio_abcdsr2[j],
                                      &mstat);
                if (NULL == mmap_ptr)
                    goto clean_up;
                ap = (unsigned char *)mmap_ptr +
                     (pio_abcdsr2[j] & MAP_MASK);
                abcdsr2 = !!( *((unsigned int *)ap) & bit_mask);
                if (abcdsr1 & abcdsr2)
                    n = 3;
                else if (abcdsr1)
                    n = 1;
                else if (abcdsr2) {
                    n = 2;
                } else
                    n = 0;
                translate_peri(b, blen, j, k, n);
                if (strlen(b) < 1)
                    snprintf(b, blen, "P%c%d: sel=%d", 'A' + j, k, n);
            }
            if (show_val > 1) {
                if (2 == show_val)
                    addr = pio_pusr[j];
                else
                    addr = pio_mdsr[j];
                mmap_ptr = check_mmap(mem_fd, addr, &mstat);
                if (NULL == mmap_ptr)
                    return 1;
                ap = (unsigned char *)mmap_ptr + (addr & MAP_MASK);
                v = !!( *((unsigned int *)ap) & bit_mask);
                if (2 == show_val)
                    v = ! v;    /* pusr==0, implies pull-up active */
                n = strlen(b);
                if (v && (n < (int)blen))
                    snprintf(b + n, blen - n, "%c", '^');
            }
            n = strlen(b);
            printf("%-13s", b);
        }
        printf("\n");
    }
    res = 0;

clean_up:
    if (mem_fd >= 0)
        close(mem_fd);
    return res;
}


int
main(int argc, char ** argv)
{
    struct stat sb;
    int pioc_num, opt, k, num;
    int res = 0;
    int mem_fd = -1;
    int do_all = 0;
    int brief = 0;
    int brief_given = 0;
    int enumerate = 0;
    int do_help = 0;
    int interrupt = 0;
    int origin0 = 0;
    int translate = 0;
    int show_all = 0;
    int write_prot = 0;
    int knum = -1;
    int bit_num = -1;
    int ret = 0;
    unsigned int bit_mask;
    const char * cp;
    char ch;
    char bank = '\0';

    while ((opt = getopt(argc, argv, "ab:Behip:sStvVw")) != -1) {
        switch (opt) {
        case 'a':
            ++do_all;
            break;
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
        case 'B':
            ++brief;
            ++brief_given;
            break;
        case 'e':
            ++enumerate;
            break;
        case 'h':
            ++do_help;
            break;
        case 'i':
            ++interrupt;
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
                if ((k < 0) || (k > 191)) {
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
        case 's':
            ++do_all;
            ++translate;
            brief += 2;
            break;
        case 'S':
            ++show_all;
            break;
        case 't':
            ++translate;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
        case 'w':
            ++write_prot;
            break;
        default: /* '?' */
            do_help = 1;
            ret = 1;
            break;
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            do_help = 1;
            ret = 1;
        }
    }
    if (do_help) {
        usage(do_help);
        exit(ret ? EXIT_FAILURE : EXIT_SUCCESS);
    }

    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (verbose > 1)
            fprintf(stderr, "%s found so kernel pin numbers start at 0 "
                    "(for PA0)\n", GPIO_BANK_ORIGIN);
        ++origin0;
    } else if (verbose > 2)
        fprintf(stderr, "%s not found so kernel pin numbers start at 32 "
                "(for PA0)\n", GPIO_BANK_ORIGIN);

    if (enumerate)
        return do_enumerate(enumerate, bank, origin0);
    if (show_all)
        return do_show_all(show_all);

    if (knum >= 0) {
        if (bit_num >= 0) {
            fprintf(stderr, "Give either '-p <knum>' or ('-b <bn>' and "
                    "'-p <bank>') but not both\n");
            exit(EXIT_FAILURE);
        }
        if ((! origin0) && (knum < 32)) {
            fprintf(stderr, "since %s not found assume kernel pin numbers "
                    "start at 32\n(for PA0) so %d is too low\n",
                    GPIO_BANK_ORIGIN, knum);
            exit(EXIT_FAILURE);
        }
    } else if (bank) {
        if (do_all)
            knum = origin0 ? 0 : 32;
        else if (bit_num < 0) {
            if (write_prot)
                knum = origin0 ? 0 : 32;
            else {
                fprintf(stderr, "If '-p <bank>' given then also need "
                        "'-b <bn>'\n");
                exit(EXIT_FAILURE);
            }
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        if (do_all) {
            printf(">>> Assuming bank A, use '-p <port>' to change\n");
            knum = origin0 ? 0 : 32;
        } else {
            fprintf(stderr, "Need to give gpio line with '-p <port>' and/or "
                    "'-b <bn>'\n");
            usage(1);
            exit(EXIT_FAILURE);
        }
    }
    pioc_num = bank ? (bank - 'A') : ((knum - (origin0 ? 0 : 32)) / 32);
    if (bit_num < 0)
        bit_num = knum % 32;
    bit_mask = 1 << bit_num;
    if (do_all) {
        if (brief_given && (brief > brief_given))
            brief = brief_given;
    } else {
        if (verbose)
            printf("P%c%d:\n", 'A' + pioc_num, bit_num);
        if (verbose > 1)
            printf("  bit_mask=0x%08x\n", bit_mask);
    }

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose > 2)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");

    if (do_all) {
        num = LINES_PER_BANK;
        res = 0;
        if (brief > 1)
            printf("AT91SAM9G25: PIO %c:\n", 'A' + pioc_num);
        for (bit_num = 0; bit_num < num; ++bit_num) {
            bit_mask = 1 << bit_num;
            if (brief < 2)
                printf("P%c%d:\n", 'A' + pioc_num, bit_num);
            res = pio_status(mem_fd, bit_mask, bit_num, brief, interrupt,
                             translate, pioc_num, write_prot);
            if (res)
                break;
        }
    } else
        res = pio_status(mem_fd, bit_mask, bit_num, brief, interrupt,
                         translate, pioc_num, write_prot);

    if (mem_fd >= 0)
        close(mem_fd);
    return res;
}
