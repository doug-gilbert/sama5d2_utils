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
 * a5d2_pmc.c
 *
 * Utility to access the Power Management Controller (PMC) on a
 * SAMA5D2 family of SoCs. The PMC controls the distribution
 * of system and peripheral clocks to various elements (e.g. PIO
 * controllers) within the microccontroller (SoC). All peripheral
 * clocks are off after reset and the kernel boot-up will turn
 * on a peripheral clock if the corresponding element is configured
 * in the kernel (and in the device-tree boot-up file). If a
 * peripheral is not being used then power can be saved by turning
 * off its peripheral clock.
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


static const char * version_str = "1.00 20160104";

#define MAX_ELEMS 256
#define DEV_MEM "/dev/mem"
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)

/* SAMA5D2* memory mapped registers for PMC unit */
#define PMC_SCER   0xf0014000   /* system clock enable */
#define PMC_SCDR   0xf0014004   /* system clock disable */
#define PMC_SCSR   0xf0014008   /* system clock status */
#define PMC_PCER0  0xf0014010   /* peripheral clock enable, reg 0 */
#define PMC_PCDR0  0xf0014014   /* peripheral clock disable, reg 0 */
#define PMC_PCSR0  0xf0014018   /* peripheral clock status, reg 0 */
#define PMC_WPMR   0xf00140e4   /* write protect mode */
#define PMC_WPSR   0xf00140e8   /* write protect status */
#define PMC_PCER1  0xf0014100   /* peripheral clock enable, reg 1 */
#define PMC_PCDR1  0xf0014104   /* peripheral clock disable, reg 1 */
#define PMC_PCSR1  0xf0014108   /* peripheral clock status, reg 1 */
#define PMC_PCR    0xf001410c   /* peripheral control */

#define A5D2_PMC_WPKEY 0x504d43  /* "PMC" in ASCII */
#define PMC_PCR_WR_CMD 0x1000
#define PMC_PCR_EN 0x10000000
#define PMC_PCR_GCKEN 0x20000000


struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};

struct bit_acron_desc {
    int bit_num;        /* for peripherals, also identifier (PID) */
    int div_apart_from_1;
    const char * acron;
    const char * desc;
};

/* Array of system clocks. bit_num values assumed to be in ascending order */
static struct bit_acron_desc sys_clock_arr[] = {
    {0, 0, "PCK", "Processor clock"},
    {2, 0, "DDRCK", "DDR clock"},
    {3, 0, "LCDCK", "LCD2x clock"},
    {6, 0, "UHP", "The UHP48M and UHP12M OHCI clocks"},
    {7, 0, "UDP", "USB device clock"},
    {8, 0, "PCK0", "Programmable clock 0"},
    {9, 0, "PCK1", "Programmable clock 1"},
    {10, 0, "PCK2", "Programmable clock 2"},
    {18, 0, "ISCCK", "Image sensor controller clock"},
    {-1, -1, NULL, NULL},
};

/* Array of peripheral/generic clock sources. bit_num values assumed to be
 * in ascending order */
static struct bit_acron_desc per_clock_src_arr[] = {
    {0, 0, "SLOW", "Slow clock (32768 Hz)"},
    {1, 0, "MAIN", "Main clock"},
    {2, 0, "PLLA", "PLLA clock (PLLACK)"},
    {3, 0, "UPLL", "UPLL clock"},
    {4, 0, "MCK", "master clock"},
    {5, 0, "AUDIO", "audio PLL clock"},
    {-1, -1, NULL, NULL},
};

/* Array of peripheral clocks. bit_num values assumed to be in ascending
 * order. */
static struct bit_acron_desc peri_clock_arr[] = {
    /* {0, 0, "FIQ", "FIQ Interrupt identifier"}, <not for PMC> */
    {2, 0, "ARM", "Performance monitor unit (PMU)"},
    /* {3, 0, "PIT", "Periodic interval timer"},  <not for PMC> */
    /* {4, 0, "WDT", "watchdog timer"},		<not for PMC> */
    {5, 1, "GMAC", "Ethernet MAC"},
    {6, 1, "XDMAC0", "DMA controller 0"},
    {7, 1, "XDMAC1", "DMA controller 1"},
    {8, 0, "ICM", "Integrity check monitor"},
    {9, 1, "AES", "Advanced encryption standard"},
    {10, 1, "AESB", "Advanced encryption standard brdige"},
    {11, 1, "TDES", "Triple data encryption standard"},
    {12, 1, "SHA", "SHA signature"},
    {13, 0, "MPDDRC", "MPDDR controller"},
    {14, 0, "MATRIX1", "H32MX 32 bit AHB matrix"},
    {15, 0, "MATRIX0", "H64MX 64 bit AHB matrix"},
    {16, 0, "SECUMOD", "Security module"},
    {17, 0, "HSMC", "Multi-bit ECC module"},
    {18, 1, "PIOA", "Parallel I/O controller A"},
    {19, 1, "FLEXCOM0", "FLEXCOM 0"},
    {20, 1, "FLEXCOM1", "FLEXCOM 1"},
    {21, 1, "FLEXCOM2", "FLEXCOM 2"},
    {22, 1, "FLEXCOM3", "FLEXCOM 3"},
    {23, 1, "FLEXCOM4", "FLEXCOM 4"},
    {24, 1, "UART0", "UART 0"},
    {25, 1, "UART1", "UART 1"},
    {26, 1, "UART2", "UART 2"},
    {27, 1, "UART3", "UART 3"},
    {28, 1, "UART4", "UART 4"},
    {29, 1, "TWIHS0", "Two wire interface (I2C) 0"},
    {30, 1, "TWIHS1", "Two wire interface 1"},
    {31, 0, "SDMMC0", "SD card controller 0"},
    {32, 0, "SDMMC1", "SD card controller 1"},
    {33, 1, "SPI0", "Serial peripheral interface 0"},
    {34, 1, "SPI1", "Serial peripheral interface 1"},
    {35, 1, "TC0", "Timer counter 0 (ch. 0, 1, 2)"},
    {36, 1, "TC1", "Timer counter 1 (ch. 4, 5, 6)"},
    {38, 1, "PWM", "Pulse width modulation controller 0 (ch. 0, 1, 2, 3)"},
    {40, 1, "ADC", "Touchscreen ADC controller"},
    {41, 0, "UHPHS", "USB host, high speed"},
    {42, 1, "UDPHS", "USB device, high speed"},
    {43, 1, "SSC0", "Synchronous serial controller 0"},
    {44, 1, "SSC1", "Synchronous serial controller 1"},
    {45, 0, "LCDC", "LCD controller"},
    {46, 0, "ISC", "Image sensor controller"},
    {47, 1, "TRNG", "True random number generator"},
    {48, 1, "PDMI", "Pulse density modulation interface controller"},
    /* {49, 0, "AIC", "IRQ interrupt ID"},          <not for PMC> */
    {50, 1, "SFC", "Fuse controller"},
    {51, 1, "SECURAM", "Secure RAM"},
    {52, 0, "QSPI0", "Quad SPI 0"},
    {53, 0, "QSPI1", "Quad SPI 1"},
    {54, 1, "I2SC0", "Inter-IC sound controller 0"},
    {55, 1, "I2SC1", "Inter-IC sound controller 1"},
    {56, 0, "CAN0", "MCAN 0 interrupt0"},
    {57, 0, "CAN1", "MCAN 1 interrupt0"},
    {59, 0, "CLASSD", "Audio class D amplifier"},
    {-1, -1, NULL, NULL},
};

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "a5d2_pmc [-a ACRON] [-d DIV] [-D] [-e] [-E] [-h] [-p] [-s]\n"
            "              [-v] [-V] [-w WPEN]\n"
            "  where:\n"
            "    -a ACRON    ACRON is a system or peripheral clock "
            "acronym\n"
	    "    -c PCS      PCS is peripheral/generic clock source\n"
            "    -d DIV      DIV is 0 to 255. Selected peripheral clock "
            "divided\n"
            "                by (DIV + 1). Default DIV value is 0\n"
            "    -D          disable ACRON system or peripheral clock\n"
            "    -e          enumerate system and peripheral clocks\n"
            "    -E          enable ACRON system or peripheral clock\n"
            "    -h          print usage message\n"
            "    -p          select peripheral clock. When no (other) "
            "options\n"
            "                given, shows all enabled peripheral clocks\n"
            "    -s          select system clock. When no other options "
            "given\n"
            "                shows all enabled system clocks\n"
            "    -v          increase verbosity (multiple times for more)\n"
            "    -V          print version string then exit\n"
            "    -w WPEN     set or show write protect (WP) information "
            "for PMC.\n"
            "                0 -> disable (def, no WP), 1 -> enable, "
            "-1 -> show\n"
            "                WP en/disable state and show WP status "
            "register\n\n"
            "Accesses the Power Management Controller (PMC) in a "
            "SAMA5D2* SoC.\nDisabling clocks for elements that are not "
            "being used may save power.\nNote that the kernel might only "
            "enable U(S)ART clocks when the\ncorresponding port is open. "
            "The master clock (MCK) is typically 166 MHz.\nWithout any "
            "options this utility will list active peripheral clocks.\n");
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


int
main(int argc, char * argv[])
{
    int mem_fd, k, opt, n, no_clock_preference, found;
    int bn = 0;
    int res = 1;
    unsigned int reg, ui, mask;
    const char * acronp = NULL;
    int divisor = 0;
    int do_disable = 0;
    int enumerate = 0;
    int do_enable = 0;
    int sel_peri_clks = 0;
    int sel_sys_clks = 0;
    int wpen = 0;
    int wpen_given = 0;
    struct bit_acron_desc * badp;
    int alternate_divisor = -1;
    void * mmap_ptr = (void *)-1;
    void * ap;
    struct mmap_state mstat;
    char b[16];
    const char * cp;

    mem_fd = -1;
    while ((opt = getopt(argc, argv, "a:d:DeEhpsvVw:")) != -1) {
        switch (opt) {
        case 'a':
            acronp = optarg;
            break;
        case 'd':
            divisor = atoi(optarg);
            if ((divisor < 0) || (divisor > 255)) {
                fprintf(stderr, "expect argument to '-d' to be 0 to 255 "
                        "inclusive\n");
                return 1;
            }
            break;
        case 'D':
            ++do_disable;
            break;
        case 'e':
            ++enumerate;
            break;
        case 'E':
            ++do_enable;
            break;
        case 'h':
        case '?':
            usage();
            return 0;
        case 'p':
            ++sel_peri_clks;
            break;
        case 's':
            ++sel_sys_clks;
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

    if (enumerate) {
        printf("System clocks:\n");
        printf("\tID\tAcronym\t\tDescription\n");
        printf("-------------------------------------------------\n");
        for (badp = sys_clock_arr; badp->bit_num >= 0; ++badp) {
            n = strlen(badp->acron);
            printf("\t%d\t%s%s\t%s\n", badp->bit_num, badp->acron,
                   ((n > 7) ? "" : "\t"), badp->desc);
        }
        printf("\nPeripheral clocks:\n");
        printf("\tID\tAcronym\t\tDescription\n");
        printf("-------------------------------------------------\n");
        for (badp = peri_clock_arr; badp->bit_num >= 0; ++badp) {
            n = strlen(badp->acron);
            printf("\t%d\t%s%s\t%s\n", badp->bit_num, badp->acron,
                   ((n > 7) ? "" : "\t"), badp->desc);
        }
        return 0;
    }

    if (do_disable && do_enable) {
        fprintf(stderr, "Cannot give both '-D' and '-E' options\n");
        usage();
        return 1;
    }
    if (divisor > 0) {
        if (NULL == acronp) {
            fprintf(stderr, "with '-d DIV' must also give '-a ACRON'\n");
            return 1;
        }
        if ((! do_disable) && (! do_enable)) {
            fprintf(stderr, "with '-d DIV' must give either '-D' or "
                    "'-E'\n");
            return 1;
        }
    }

    no_clock_preference = ((0 == sel_peri_clks) && (0 == sel_sys_clks));
    if (acronp) {
        if (isdigit(*acronp)) {
            if ((!! sel_peri_clks) == (!! sel_sys_clks)) {
                fprintf(stderr, "When ACRON is a number need either '-p' "
                        "or '-s' but not both\n");
                return 1;
            }
            bn = atoi(acronp);
            if ((bn < 0) || (bn > 63)) {
                fprintf(stderr, "When ACRON is a number that number needs "
                        "to be from 0 to 63\n");
                return 1;
            }
        } else { /* try to match (case insensitive) with the acron field */
            for (k = 0; k < (int)(sizeof(b) - 1); ++k)
                b[k] = toupper(acronp[k]);
            b[sizeof(b) - 1] = '\0';
            found = 0;
            if (no_clock_preference || sel_sys_clks) {
                for (badp = sys_clock_arr; badp->bit_num >= 0; ++badp) {
                    if (0 == strcmp(b, badp->acron))
                        break;
                }
                if (badp->bit_num >= 0) {
                    bn = badp->bit_num;
                    ++found;
                    sel_sys_clks = 1;
                    sel_peri_clks = 0;
                }
            }
            if (! found && ((no_clock_preference || sel_peri_clks))) {
                for (badp = peri_clock_arr; badp->bit_num >= 0; ++badp) {
                    if (0 == strcmp(b, badp->acron))
                        break;
                    if ((cp = strchr(badp->acron, '_')) &&
                        (0 == strncmp(b, badp->acron, cp - badp->acron)))
                        /* allow 'PIOA' to match 'PIOA_PIOB' */
                        break;
                }
                if (badp->bit_num >= 0) {
                    bn = badp->bit_num;
                    alternate_divisor = badp->div_apart_from_1;
                    ++found;
                    sel_peri_clks = 1;
                    sel_sys_clks = 0;
                }
            }
            if (! found) {
                fprintf(stderr, "Could not match ACRON: %s, use '-e' to "
                        "see what is available\n", acronp);
                return 1;
            }
        }
        if (sel_sys_clks && (bn > 31)) {
            fprintf(stderr, "For system clocks the ACRON value [%d] "
                    "cannot exceed 31\n", bn);
            return 1;
        }
    } else if (do_disable || do_enable) {
        fprintf(stderr, "Both '-D' and '-E' need the '-a ACRON' option "
                "to be given\n");
        usage();
        return 1;
    }
    if ((divisor > 0) && (0 == sel_peri_clks)) {
        fprintf(stderr, "'-d DIV' only applies to peripheral (not system) "
                "clocks\n");
        return 1;
    }

    if ((0 == sel_peri_clks) && (0 == sel_sys_clks))
        ++sel_peri_clks;

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        if (NULL == acronp)
            fprintf(stderr, "  Try '-h' to see usage.\n");
        return 1;
    } else if (verbose)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");

    memset(&mstat, 0, sizeof(mstat));

    if (wpen_given) {
        if (-1 == wpen) {
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_WPMR, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_WPMR & MAP_MASK);
            reg = *((unsigned long *)ap);
            printf("Write protect mode: %sabled\n",
                   ((reg & 1) ? "EN" : "DIS"));
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_WPSR, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_WPSR & MAP_MASK);
            reg = *((unsigned long *)ap) & 0xffffff;
            printf("Write protect violation status: %d (%s), WPCSRC: "
                   "0x%x\n", (reg & 1), ((reg & 1) ? "VIOLATED" :
                   "NOT violated"), (reg >> 8) & 0xffff);
        } else if ((0 == wpen) || (1 == wpen)) {
            mmap_ptr = check_mmap(mem_fd, PMC_WPMR, &mstat);
            if (NULL == mmap_ptr)
                return 1;
            ap = (unsigned char *)mmap_ptr + (PMC_WPMR & MAP_MASK);
            *((unsigned int *)ap) = (A5D2_PMC_WPKEY << 8) | wpen;
        }
        res = 0;
        goto clean_up;
    }
    if (do_enable || do_disable) {
        if (divisor > 0) {
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCR, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_PCR & MAP_MASK);
            n = divisor - 1;
            if (3 == n)
                n = 2;
            else if (7 == n)
                n = 3;
            reg = PMC_PCR_WR_CMD | bn | (n << 16);
            if (do_enable)
                reg |= PMC_PCR_EN;
            if (verbose > 1)
                printf("Writing 0x%x to %p [IO addr: 0x%x]\n", reg, ap,
                       PMC_PCR);
            *((unsigned long *)ap) = reg;
        } else {
            if (do_enable) {
                if (sel_sys_clks)
                    ui = PMC_SCER;
                else
                    ui = (bn > 31) ? PMC_PCER1 : PMC_PCER0;
            } else {
                if (sel_sys_clks)
                    ui = PMC_SCDR;
                else
                    ui = (bn > 31) ? PMC_PCDR1 : PMC_PCDR0;
            }
            mask = (bn > 31) ? (1 << (bn - 32)) : (1 << bn);
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, ui, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (ui & MAP_MASK);
            if (verbose > 1)
                printf("Writing 0x%x to %p [IO addr: 0x%x]\n", mask, ap, ui);
            *((unsigned long *)ap) = mask;
        }
        res = 0;
        goto clean_up;
    }

    if (sel_sys_clks) {
        if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_SCSR, &mstat))))
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (PMC_SCSR & MAP_MASK);
        reg = *((unsigned long *)ap);
        if (verbose)
            fprintf(stderr, "PMC_SCSR=0x%x\n", reg);
        badp = sys_clock_arr;

        if (acronp) {
            if (reg & (1 << bn))
                printf("%s system clock ENabled\n", acronp);
            else
                printf("%s system clock DISabled\n", acronp);
        } else {
            printf("System clocks enabled:\n");
            for (k = 0, mask = 1; k < 32; ++k, mask <<= 1) {
                if (reg & mask) {
                    for ( ; badp->bit_num >= 0; ++badp) {
                        if (k == badp->bit_num) {
                            printf("    %s:\t%s\n", badp->acron, badp->desc);
                            break;
                        } else if (k < badp->bit_num) {
                            printf("    PMC_SCSR bit_num=%d set\n", k);
                            break;
                        }
                    }
                    if (badp->bit_num < 0)
                        printf("    PMC_SCSR bit_num=%d set\n", k);
                }
            }
        }
    }

    if (sel_peri_clks) {
        if (sel_sys_clks)
            printf("\n");
        if (acronp) {
            if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCR, &mstat))))
                goto clean_up;
            ap = (unsigned char *)mmap_ptr + (PMC_PCR & MAP_MASK);
            /* write a read cmd for given bn (in the PID field) */
            *((volatile unsigned long *)ap) = bn;
            /* now read back result, DIV field should be populated */
            reg = *((volatile unsigned long *)ap);
            if (verbose)
                fprintf(stderr, "PMC_PCR=0x%x\n", reg);
            n = 1 << ((reg & 0x30000) >> 16);   /* should give 1, 2, 4 or 8 */
            if ((1 == n) && (alternate_divisor >= 0)) {
                if (0 == alternate_divisor)
                    printf("%s peripheral clock is MCK [no divisor "
                           "allowed]\n", acronp);
                else
                    printf("%s peripheral clock is MCK [divisor of 2, 4 or "
                           "8 may be selected]\n", acronp);
            } else
                printf("%s peripheral clock is MCK divided by %d\n", acronp,
                       n);
        }

        if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCSR0, &mstat))))
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (PMC_PCSR0 & MAP_MASK);
        reg = *((unsigned long *)ap);
        if (verbose)
            fprintf(stderr, "PMC_PCSR0=0x%x\n", reg);
        badp = peri_clock_arr;

        if (acronp) {
            if (bn < 32) {
                if (reg & (1 << bn))
                    printf("%s peripheral clock ENabled\n", acronp);
                else
                    printf("%s peripheral clock DISabled\n", acronp);
            }
        } else {
            printf("Peripheral clocks enabled:\n");
            for (k = 0, mask = 1; k < 32; ++k, mask <<= 1) {
                if (reg & mask) {
                    for ( ; badp->bit_num >= 0; ++badp) {
                        if (k == badp->bit_num) {
                            printf("    %s:\t%s\n", badp->acron, badp->desc);
                            break;
                        } else if (k < badp->bit_num) {
                            printf("    PMC_PCSR0 bit_num=%d set\n", k);
                            break;
                        }
                    }
                    if (badp->bit_num < 0)
                        printf("    PMC_PCSR0 bit_num=%d set\n", k);
                }
            }
        }

        if (NULL == ((mmap_ptr = check_mmap(mem_fd, PMC_PCSR1, &mstat))))
            goto clean_up;
        ap = (unsigned char *)mmap_ptr + (PMC_PCSR1 & MAP_MASK);
        reg = *((unsigned long *)ap);
        if (verbose)
            fprintf(stderr, "PMC_PCSR1=0x%x\n", reg);
        badp = peri_clock_arr;

        if (acronp) {
            if (bn > 31) {
                if (reg & (1 << (bn - 32)))
                    printf("%s peripheral clock ENabled\n", acronp);
                else
                    printf("%s peripheral clock DISabled\n", acronp);
            }
        } else {
            // printf("Peripheral clocks enabled:\n");
            for (k = 32, mask = 1; k < 64; ++k, mask <<= 1) {
                if (reg & mask) {
                    for ( ; badp->bit_num >= 0; ++badp) {
                        if (k == badp->bit_num) {
                            printf("    %s:\t%s\n", badp->acron, badp->desc);
                            break;
                        } else if (k < badp->bit_num) {
                            printf("    PMC_PCSR1 bit_num=%d set\n", k);
                            break;
                        }
                    }
                    if (badp->bit_num < 0)
                        printf("    PMC_PCSR1 bit_num=%d set\n", k);
                }
            }
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
