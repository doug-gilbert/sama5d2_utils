/*
 * Copyright (c) 2016-2018 Douglas Gilbert.
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


static const char * version_str = "1.01 20180730";


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
    int freeze_phy1int2b3;      /* -F [0123]  (freeze configuration) */
    int en_if;          /* -G   (glitch) */
    int di_if;          /* -g */
    int en_interrupt;   /* -I */
    int di_interrupt;   /* -i */
    int en_opd;         /* -M */
    int di_opd;         /* -m */
    int en_pullup1dn2;  /* -U, -UU */
    int di_pullup1dn2;  /* -u, -uu */
    int out_level;      /* -S LEV */
    int en_schmitt;     /* -T */
    int di_schmitt;     /* -t */
    int en_if_slow;     /* -Z */
    int di_if_slow;     /* -z */
    int wpen;           /* -w WPEN */
    int scdr_div;       /* -d DIV */
    int drvstr;         /* -D DRVSTR */
    int verbose;        /* -v */
    unsigned int msk;   /* -X MSK,DAT */
    unsigned int dat;
    bool evtsel_given;
    bool freeze_given;
    bool wp_given;
    bool scdr_given;
    bool drvstr_given;
    bool wr_dat_given;  /* -X MSK,DAT */
    bool dir_given;
};

struct mmap_state {
    void * mmap_ptr;
    off_t prev_mask_addrp;
    int mmap_ok;
};

struct periph_name {
    int pin;            /* 0 to 31 (PIO line number within bank) */
    int periph;         /* 1 for A, 2 for B, etc */
    const char * name;
    int dir;
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


static struct periph_name pioa_trans[] = {
    {0, PERI_A, "SDMMC0_CK", DIR_IO}, {0, PERI_B, "QSPI0_SCK", DIR_O},
    {0, PERI_F, "D0", DIR_IO},
    {1, PERI_A, "SDMMC0_CMD", DIR_IO}, {1, PERI_B, "QSPI0_CS", DIR_O},
    {1, PERI_F, "D1", DIR_IO},
    {2, PERI_A, "SDMMC0_DAT0", DIR_IO}, {2, PERI_B, "QSPI0_IO0", DIR_IO},
    {2, PERI_F, "D2", DIR_IO},
    {3, PERI_A, "SDMMC0_DAT1", DIR_IO}, {3, PERI_B, "QSPI0_IO1", DIR_IO},
    {3, PERI_F, "D3", DIR_IO},
    {4, PERI_A, "SDMMC0_DAT2", DIR_IO}, {4, PERI_B, "QSPI0_IO2", DIR_IO},
    {4, PERI_F, "D4", DIR_IO},
    {5, PERI_A, "SDMMC0_DAT3", DIR_IO}, {5, PERI_B, "QSPI0_IO3", DIR_IO},
    {5, PERI_F, "D5", DIR_IO},
    {6, PERI_A, "SDMMC0_DAT4", DIR_IO}, {6, PERI_B, "QSPI1_SCK", DIR_O},
    {6, PERI_D, "TIOA5", DIR_IO}, {6, PERI_E, "FLEXCOM2_IO0", DIR_IO},
    {6, PERI_F, "D6", DIR_IO},
    {7, PERI_A, "SDMMC0_DAT5", DIR_IO}, {7, PERI_B, "QSPI1_IO0", DIR_IO},
    {7, PERI_D, "TIOB5", DIR_IO}, {7, PERI_E, "FLEXCOM2_IO1", DIR_IO},
    {7, PERI_F, "D7", DIR_IO},
    {8, PERI_A, "SDMMC0_DAT6", DIR_IO}, {8, PERI_B, "QSPI1_IO1", DIR_IO},
    {8, PERI_D, "TCLK5", DIR_I}, {8, PERI_E, "FLEXCOM2_IO2", DIR_IO},
    {8, PERI_F, "NWE/NANDWE", DIR_OO},
    {9, PERI_A, "SDMMC0_DAT7", DIR_IO}, {9, PERI_B, "QSPI1_IO2", DIR_IO},
    {9, PERI_D, "TIOA4", DIR_IO}, {9, PERI_E, "FLEXCOM2_IO3", DIR_O},
    {9, PERI_F, "NCS3", DIR_O},
    {10, PERI_A, "SDMMC0_RSTN", DIR_O}, {10, PERI_B, "QSPI1_IO3", DIR_IO},
    {10, PERI_D, "TIOB4", DIR_IO}, {10, PERI_E, "FLEXCOM2_IO4", DIR_O},
    {10, PERI_F, "A21/NANDALE", DIR_OO},
    {11, PERI_A, "SDMMC0_VDDSEL", DIR_O}, {11, PERI_B, "QSPI1_CS", DIR_O},
    {11, PERI_D, "TCLK4", DIR_I}, {11, PERI_F, "A22/NANDCLE", DIR_OO},
    {12, PERI_A, "SDMMC0_WP", DIR_I}, {12, PERI_B, "IRQ", DIR_I},
    {12, PERI_F, "NRD/NANDOE", DIR_OO},
    {13, PERI_A, "SDMMC0_CD", DIR_I}, {13, PERI_E, "FLEXCOM3_IO1", DIR_IO},
    {13, PERI_F, "D8", DIR_IO},
    {14, PERI_A, "SPI0_SPCK", DIR_IO}, {14, PERI_B, "TK", DIR_IO},
    {14, PERI_C, "QSPI0_SCK", DIR_O}, {14, PERI_D, "I2SC1_MCK", DIR_O},
    {14, PERI_E, "FLEXCOM3_IO2", DIR_IO}, {14, PERI_F, "D9", DIR_IO},
    {15, PERI_A, "SPI0_MOSI", DIR_IO}, {15, PERI_B, "TF1", DIR_IO},
    {15, PERI_C, "QSPI0_CS", DIR_O}, {15, PERI_D, "I2SC1_CK", DIR_IO},
    {15, PERI_E, "FLEXCOM3_IO0", DIR_IO}, {15, PERI_F, "D10", DIR_IO},
    {16, PERI_A, "SPI0_MISO", DIR_IO}, {16, PERI_B, "TD1", DIR_IO},
    {16, PERI_C, "QSPI0_IO0", DIR_IO}, {16, PERI_D, "I2SC1_WS", DIR_IO},
    {16, PERI_E, "FLEXCOM3_IO3", DIR_O}, {16, PERI_F, "D11", DIR_IO},
    {17, PERI_A, "SPI0_NPCS0", DIR_IO}, {17, PERI_B, "RD1", DIR_I},
    {17, PERI_C, "QSPI0_IO1", DIR_IO}, {17, PERI_D, "I2SC1_DI0", DIR_I},
    {17, PERI_E, "FLEXCOM3_IO4", DIR_O}, {17, PERI_F, "D12", DIR_IO},
    {18, PERI_A, "SPI0_NPCS1", DIR_O}, {18, PERI_B, "RK1", DIR_IO},
    {18, PERI_C, "QSPI0_IO2", DIR_IO}, {18, PERI_D, "I2SC1_DO0", DIR_O},
    {18, PERI_E, "SDMMC1_DAT0", DIR_IO}, {18, PERI_F, "D13", DIR_IO},
    {19, PERI_A, "SPI0_NPCS2", DIR_O}, {19, PERI_B, "RF1", DIR_IO},
    {19, PERI_C, "QSPI0_IO3", DIR_IO}, {19, PERI_D, "TIOA0", DIR_IO},
    {19, PERI_E, "SDMMC1_DAT1", DIR_IO}, {19, PERI_F, "D14", DIR_IO},
    {20, PERI_A, "SPI0_NPCS3", DIR_O}, {20, PERI_D, "TIOB0", DIR_IO},
    {20, PERI_E, "SDMMC1_DAT2", DIR_IO}, {20, PERI_F, "D15", DIR_IO},
    {21, PERI_A, "IRQ", DIR_I}, {21, PERI_B, "PCK2", DIR_O},
    {21, PERI_D, "TCLK0", DIR_I}, {21, PERI_E, "SDMMC1_DAT3", DIR_IO},
    {21, PERI_F, "NANDRDY", DIR_I},
    {22, PERI_A, "FLEXCOM1_IO2", DIR_IO}, {22, PERI_B, "D0", DIR_IO},
    {22, PERI_C, "TCK", DIR_I}, {22, PERI_D, "SPI1_SPCK", DIR_IO},
    {22, PERI_E, "SDMMC1_CK", DIR_IO}, {22, PERI_F, "QSPI0_SCK", DIR_O},
    {23, PERI_A, "FLEXCOM1_IO1", DIR_IO}, {23, PERI_B, "D1", DIR_IO},
    {23, PERI_C, "TDI", DIR_I}, {23, PERI_D, "SPI1_MOSI", DIR_IO},
    {23, PERI_F, "QSPI0_CS", DIR_O},
    {24, PERI_A, "FLEXCOM1_IO0", DIR_IO}, {24, PERI_B, "D2", DIR_IO},
    {24, PERI_C, "TDO", DIR_O}, {24, PERI_D, "SPI1_MISO", DIR_IO},
    {24, PERI_F, "QSPI0_IO0", DIR_IO},
    {25, PERI_A, "FLEXCOM1_IO3", DIR_O}, {25, PERI_B, "D3", DIR_IO},
    {25, PERI_C, "TMS", DIR_I}, {25, PERI_D, "SPI1_NPCS0", DIR_IO},
    {25, PERI_F, "QSPI0_IO1", DIR_IO},
    {26, PERI_A, "FLEXCOM1_IO4", DIR_O}, {26, PERI_B, "D4", DIR_IO},
    {26, PERI_C, "NTRST", DIR_I}, {26, PERI_D, "SPI1_NPCS1", DIR_O},
    {26, PERI_F, "QSPI0_IO2", DIR_IO},
    {27, PERI_A, "TIOA1", DIR_IO}, {27, PERI_B, "D5", DIR_IO},
    {27, PERI_C, "SPI0_NPCS2", DIR_O}, {27, PERI_D, "SPI1_NPCS2", DIR_O},
    {27, PERI_E, "SDMMC1_RSTN", DIR_O}, {27, PERI_F, "QSPI0_IO3", DIR_IO},
    {28, PERI_A, "TIOB1", DIR_IO}, {28, PERI_B, "D6", DIR_IO},
    {28, PERI_C, "SPI0_NPCS3", DIR_O}, {28, PERI_D, "SPI1_NPCS3", DIR_O},
    {28, PERI_E, "SDMMC1_CMD", DIR_IO}, {28, PERI_F, "CLASS_L0", DIR_O},
    {29, PERI_A, "TCLK1", DIR_I}, {29, PERI_B, "D7", DIR_IO},
    {29, PERI_C, "SPI0_NPCS1", DIR_O}, {29, PERI_E, "SDMMC1_WP", DIR_I},
    {29, PERI_F, "CLASS_L1", DIR_O},
    {30, PERI_B, "NWE/NANDWE", DIR_OO}, {30, PERI_C, "SPI0_NPCS0", DIR_IO},
    {30, PERI_D, "PWMH0", DIR_O}, {30, PERI_E, "SDMMC1_CD", DIR_I},
    {30, PERI_F, "CLASS_L2", DIR_O},
    {31, PERI_B, "NCS3", DIR_O}, {31, PERI_C, "SPI0_MISO", DIR_IO},
    {31, PERI_D, "PWML0", DIR_O}, {31, PERI_F, "CLASS_L3", DIR_O},
    {-1, -1, NULL, -1},         /* sentinel */
};

static struct periph_name piob_trans[] = {
    {0, PERI_B, "A21/NANDALE", DIR_OO}, {0, PERI_C, "SPI0_MOSI", DIR_O},
    {0, PERI_D, "PWMH1", DIR_O},
    {1, PERI_B, "A22/NANDCLE", DIR_OO}, {1, PERI_C, "SPI0_SPCK", DIR_IO},
    {1, PERI_D, "PWML1", DIR_O}, {1, PERI_F, "CLASSD_R0", DIR_O},
    {2, PERI_B, "NRD/NANDOE", DIR_OO}, {2, PERI_D, "PWMFI0", DIR_I},
    {2, PERI_F, "CLASSD_R1", DIR_O},
    {3, PERI_A, "URXD4", DIR_I}, {3, PERI_B, "D8", DIR_IO},
    {3, PERI_C, "IRQ", DIR_I}, {3, PERI_D, "PWMEXTRG0", DIR_I},
    {3, PERI_F, "CLASSD_R2", DIR_O},
    {4, PERI_A, "UTXD4", DIR_O}, {4, PERI_B, "D9", DIR_IO},
    {4, PERI_C, "FIQ", DIR_I}, {4, PERI_F, "CLASSD_R3", DIR_O},
    {5, PERI_A, "TCLK2", DIR_I}, {5, PERI_B, "D10", DIR_IO},
    {5, PERI_C, "PWMH2", DIR_O}, {5, PERI_D, "QSPI1_SCK", DIR_I},
    {5, PERI_F, "GTSUCOMP", DIR_O},
    {6, PERI_A, "TIOA2", DIR_IO}, {6, PERI_B, "D11", DIR_IO},
    {6, PERI_C, "PWML2", DIR_O}, {6, PERI_D, "QSPI1_CS", DIR_O},
    {6, PERI_F, "GTSUCOMP", DIR_O},
    {7, PERI_A, "TIOB2", DIR_IO}, {7, PERI_B, "D12", DIR_IO},
    {7, PERI_C, "PWMH3", DIR_O}, {7, PERI_D, "QSPI1_IO0", DIR_IO},
    {7, PERI_F, "GRXCK", DIR_I},
    {8, PERI_A, "TCLK3", DIR_I}, {8, PERI_B, "D13", DIR_IO},
    {8, PERI_C, "PWML3", DIR_O}, {8, PERI_D, "QSPI1_IO1", DIR_IO},
    {8, PERI_F, "GCRS", DIR_I},
    {9, PERI_A, "TIOA3", DIR_IO}, {9, PERI_B, "D14", DIR_IO},
    {9, PERI_C, "PWMFI1", DIR_I}, {9, PERI_D, "QSPI1_IO2", DIR_IO},
    {9, PERI_F, "GCOL", DIR_I},
    {10, PERI_A, "TIOB3", DIR_IO}, {10, PERI_B, "D15", DIR_IO},
    {10, PERI_C, "PWMEXTRG1", DIR_I}, {10, PERI_D, "QSPI1_IO3", DIR_IO},
    {10, PERI_F, "GRX2", DIR_I},
    {11, PERI_A, "LCDDAT0", DIR_O}, {11, PERI_B, "A0/NBS0", DIR_OO},
    {11, PERI_C, "URXD3", DIR_I}, {11, PERI_D, "PDMIC_DAT", DIR_UNK},
    {11, PERI_F, "GRX3", DIR_I},
    {12, PERI_A, "LCDDAT1", DIR_O}, {12, PERI_B, "A1", DIR_O},
    {12, PERI_C, "UTXD3", DIR_O}, {12, PERI_D, "PDMIC_CLK", DIR_UNK},
    {12, PERI_F, "GTX2", DIR_O},
    {13, PERI_A, "LCDDAT2", DIR_O}, {13, PERI_B, "A2", DIR_O},
    {13, PERI_C, "PCK1", DIR_O}, {13, PERI_F, "GTX3", DIR_O},
    {14, PERI_A, "LCDDAT3", DIR_O}, {14, PERI_B, "A3", DIR_O},
    {14, PERI_C, "TK1", DIR_IO}, {14, PERI_D, "I2SC1_MCK", DIR_O},
    {14, PERI_E, "QSPI1_SCK", DIR_O}, {14, PERI_F, "GTX3", DIR_O},
    {15, PERI_A, "LCDDAT4", DIR_O}, {15, PERI_B, "A4", DIR_O},
    {15, PERI_C, "TF1", DIR_IO}, {15, PERI_D, "I2SC1_CK", DIR_IO},
    {15, PERI_E, "QSPI1_CS", DIR_O}, {15, PERI_F, "GTXEN", DIR_O},
    {16, PERI_A, "LCDDAT5", DIR_O}, {16, PERI_B, "A5", DIR_O},
    {16, PERI_C, "TD1", DIR_O}, {16, PERI_D, "I2SC1_WS", DIR_IO},
    {16, PERI_E, "QSPI1_IO0", DIR_IO}, {16, PERI_F, "GRXDV", DIR_I},
    {17, PERI_A, "LCDDAT6", DIR_O}, {17, PERI_B, "A6", DIR_O},
    {17, PERI_C, "RD1", DIR_I}, {17, PERI_D, "I2SC1_DI0", DIR_I},
    {17, PERI_E, "QSPI1_IO1", DIR_IO}, {17, PERI_F, "GRXER", DIR_I},
    {18, PERI_A, "LCDDAT7", DIR_O}, {18, PERI_B, "A7", DIR_O},
    {18, PERI_C, "RK1", DIR_IO}, {18, PERI_D, "I2SC1_DO0", DIR_O},
    {18, PERI_E, "QSPI1_IO2", DIR_IO}, {18, PERI_F, "GRX0", DIR_I},
    {19, PERI_A, "LCDDAT8", DIR_O}, {19, PERI_B, "A8", DIR_O},
    {19, PERI_C, "RF1", DIR_IO}, {19, PERI_D, "TIOA3", DIR_IO},
    {19, PERI_E, "QSPI1_IO3", DIR_IO}, {19, PERI_F, "GRX1", DIR_I},
    {20, PERI_A, "LCDDAT9", DIR_O}, {20, PERI_B, "A9", DIR_O},
    {20, PERI_C, "TK0", DIR_IO}, {20, PERI_D, "TIOB3", DIR_IO},
    {20, PERI_E, "PCK1", DIR_O}, {20, PERI_F, "GTX0", DIR_O},
    {21, PERI_A, "LCDDAT10", DIR_O}, {21, PERI_B, "A10", DIR_O},
    {21, PERI_C, "TF0", DIR_IO}, {21, PERI_D, "TCLK3", DIR_I},
    {21, PERI_E, "FLEXCOM3_IO2", DIR_IO}, {21, PERI_F, "GTX1", DIR_O},
    {22, PERI_A, "LCDDAT11", DIR_O}, {22, PERI_B, "A11", DIR_O},
    {22, PERI_C, "TD0", DIR_O}, {22, PERI_D, "TIOA2", DIR_IO},
    {22, PERI_E, "FLEXCOM3_IO1", DIR_IO}, {22, PERI_F, "GMDC", DIR_O},
    {23, PERI_A, "LCDDAT12", DIR_O}, {23, PERI_B, "A12", DIR_O},
    {23, PERI_C, "RD0", DIR_I}, {23, PERI_D, "TIOB2", DIR_IO},
    {23, PERI_E, "FLEXCOM3_IO0", DIR_IO}, {23, PERI_F, "GMDIO", DIR_IO},
    {24, PERI_A, "LCDDAT13", DIR_O}, {24, PERI_B, "A13", DIR_O},
    {24, PERI_C, "RK0", DIR_IO}, {24, PERI_D, "TIOB2", DIR_IO},
    {24, PERI_E, "FLEXCOM3_IO3", DIR_O}, {24, PERI_F, "ISC_D10", DIR_I},
    {25, PERI_A, "LCDDAT14", DIR_O}, {25, PERI_B, "A14", DIR_O},
    {25, PERI_C, "RF0", DIR_IO}, {25, PERI_E, "FLEXCOM3_IO4", DIR_O},
    {25, PERI_F, "ISC_D11", DIR_I},
    {26, PERI_A, "LCDDAT15", DIR_O}, {26, PERI_B, "A15", DIR_O},
    {26, PERI_C, "URXD0", DIR_I}, {26, PERI_D, "PDMIC_DAT", DIR_UNK},
    {26, PERI_F, "ISC_D0", DIR_I},
    {27, PERI_A, "LCDDAT16", DIR_O}, {27, PERI_B, "A16", DIR_O},
    {27, PERI_C, "UTXD0", DIR_O}, {27, PERI_D, "PDMIC_CLK", DIR_UNK},
    {27, PERI_F, "ISC_D1", DIR_I},
    {28, PERI_A, "LCDDAT17", DIR_O}, {28, PERI_B, "A17", DIR_O},
    {28, PERI_C, "FLEXCOM0_IO0", DIR_IO}, {28, PERI_D, "TIOA5", DIR_IO},
    {28, PERI_F, "ISC_D2", DIR_I},
    {29, PERI_A, "LCDDAT18", DIR_O}, {29, PERI_B, "A18", DIR_O},
    {29, PERI_C, "FLEXCOM0_IO1", DIR_IO}, {29, PERI_D, "TIOB5", DIR_IO},
    {29, PERI_F, "ISC_D3", DIR_I},
    {30, PERI_A, "LCDDAT19", DIR_O}, {30, PERI_B, "A19", DIR_O},
    {30, PERI_C, "FLEXCOM0_IO2", DIR_IO}, {30, PERI_D, "TCLK5", DIR_I},
    {30, PERI_F, "ISC_D4", DIR_I},
    {31, PERI_A, "LCDDAT20", DIR_O}, {31, PERI_B, "A20", DIR_O},
    {31, PERI_C, "FLEXCOM0_IO3", DIR_O}, {31, PERI_D, "TWD0", DIR_IO},
    {31, PERI_F, "ISC_D5", DIR_I},
    {-1, -1, NULL, -1},         /* sentinel */
};

static struct periph_name pioc_trans[] = {
    {0, PERI_A, "LCDDAT21", DIR_O}, {0, PERI_B, "A23", DIR_O},
    {0, PERI_C, "FLEXCOM0_IO4", DIR_O}, {0, PERI_D, "TWCK0", DIR_I},
    {0, PERI_F, "ISC_D6", DIR_I},
    {1, PERI_A, "LCDDAT22", DIR_O}, {1, PERI_B, "A24", DIR_O},
    {1, PERI_C, "CANTX0", DIR_O}, {1, PERI_D, "SPI1_SPCK", DIR_IO},
    {1, PERI_E, "I2SC0_CK", DIR_IO}, {1, PERI_F, "ISC_D7", DIR_I},
    {2, PERI_A, "LCDDAT23", DIR_O}, {2, PERI_B, "A25", DIR_O},
    {2, PERI_C, "CANRX0", DIR_I}, {2, PERI_D, "SPI1_MOSI", DIR_IO},
    {2, PERI_E, "I2SC0_MCK", DIR_O}, {2, PERI_F, "ISC_D8", DIR_I},
    {3, PERI_A, "LCDPWM", DIR_O}, {3, PERI_B, "NWAIT", DIR_I},
    {3, PERI_C, "TIOA1", DIR_IO}, {3, PERI_D, "SPI1_MISO", DIR_IO},
    {3, PERI_E, "I2SC0_WS", DIR_IO}, {3, PERI_F, "ISC_D9", DIR_I},
    {4, PERI_A, "LCDDISP", DIR_O}, {4, PERI_B, "NWR1/NBS1", DIR_OO},
    {4, PERI_C, "TIOB1", DIR_IO}, {4, PERI_D, "SPI1_NPCS0", DIR_IO},
    {4, PERI_E, "I2SC0_DI0", DIR_I}, {4, PERI_F, "ISC_PCK", DIR_I},
    {5, PERI_A, "LCDVSYNC", DIR_O}, {5, PERI_B, "NCS0", DIR_O},
    {5, PERI_C, "TCLK1", DIR_I}, {5, PERI_D, "SPI1_NPCS1", DIR_O},
    {5, PERI_E, "I2SC0_DO0", DIR_O}, {5, PERI_F, "ISC_VSYNC", DIR_I},
    {6, PERI_A, "LCDHSYNC", DIR_O}, {6, PERI_B, "NCS1", DIR_O},
    {6, PERI_C, "TWD1", DIR_IO}, {6, PERI_D, "SPI1_NPCS2", DIR_O},
    {6, PERI_F, "ISC_HSYNC", DIR_I},
    {7, PERI_A, "LCDPCK", DIR_O}, {7, PERI_B, "NCS2", DIR_O},
    {7, PERI_C, "TWCK1", DIR_IO}, {7, PERI_D, "SPI1_NPCS3", DIR_O},
    {7, PERI_F, "URXD1", DIR_I}, {7, PERI_F, "ISC_MCK", DIR_O},
    {8, PERI_A, "LCDDEN", DIR_O}, {8, PERI_B, "NANDRDY", DIR_I},
    {8, PERI_C, "FIQ", DIR_I}, {8, PERI_D, "PCK0", DIR_O},
    {8, PERI_F, "UTXD1", DIR_O}, {8, PERI_F, "ISC_FIELD", DIR_I},
    {9, PERI_A, "FIQ", DIR_I}, {9, PERI_B, "GTSUCOMP", DIR_O},
    {9, PERI_C, "ISC_D0", DIR_I}, {9, PERI_D, "TIOA4", DIR_IO},
    {10, PERI_A, "LCDDAT2", DIR_O}, {10, PERI_B, "GTXCK", DIR_IO},
    {10, PERI_C, "ISC_D1", DIR_I}, {10, PERI_D, "TIOB4", DIR_IO},
    {10, PERI_E, "CANTX0", DIR_O},
    {11, PERI_A, "LCDDAT3", DIR_O}, {11, PERI_B, "GTXEN", DIR_O},
    {11, PERI_C, "ISC_D2", DIR_I}, {11, PERI_D, "TCLK4", DIR_I},
    {11, PERI_E, "CANRX0", DIR_I}, {11, PERI_F, "A0/NBS0", DIR_OO},
    {12, PERI_A, "LCDDAT4", DIR_O}, {12, PERI_B, "GTXDV", DIR_I},
    {12, PERI_C, "ISC_D3", DIR_I}, {12, PERI_D, "URXD3", DIR_I},
    {12, PERI_E, "TK0", DIR_IO}, {12, PERI_F, "A1", DIR_O},
    {13, PERI_A, "LCDDAT5", DIR_O}, {13, PERI_B, "GTXER", DIR_I},
    {13, PERI_C, "ISC_D4", DIR_I}, {13, PERI_D, "UTXD3", DIR_O},
    {13, PERI_E, "TF0", DIR_IO}, {13, PERI_F, "A2", DIR_O},
    {14, PERI_A, "LCDDAT6", DIR_O}, {14, PERI_B, "GRX0", DIR_I},
    {14, PERI_C, "ISC_D5", DIR_I}, {14, PERI_E, "TD0", DIR_O},
    {14, PERI_F, "A3", DIR_O},
    {15, PERI_A, "LCDDAT7", DIR_O}, {15, PERI_B, "GRX1", DIR_I},
    {15, PERI_C, "ISC_D6", DIR_I}, {15, PERI_E, "RD0", DIR_I},
    {15, PERI_F, "A4", DIR_O},
    {16, PERI_A, "LCDDAT10", DIR_O}, {16, PERI_B, "GTX0", DIR_O},
    {16, PERI_C, "ISC_D7", DIR_I}, {16, PERI_E, "RK0", DIR_IO},
    {16, PERI_F, "A5", DIR_O},
    {17, PERI_A, "LCDDAT11", DIR_O}, {17, PERI_B, "GTX1", DIR_O},
    {17, PERI_C, "ISC_D8", DIR_I}, {17, PERI_E, "RF0", DIR_IO},
    {17, PERI_F, "A6", DIR_O},
    {18, PERI_A, "LCDDAT12", DIR_O}, {18, PERI_B, "GMDC", DIR_O},
    {18, PERI_C, "ISC_D9", DIR_I}, {18, PERI_E, "FLEXCOM3_IO2", DIR_IO},
    {18, PERI_F, "A7", DIR_O},
    {19, PERI_A, "LCDDAT13", DIR_O}, {19, PERI_B, "GMDIO", DIR_IO},
    {19, PERI_C, "ISC_D10", DIR_I}, {19, PERI_E, "FLEXCOM3_IO1", DIR_IO},
    {19, PERI_F, "A8", DIR_O},
    {20, PERI_A, "LCDDAT14", DIR_O}, {20, PERI_B, "GRXCK", DIR_I},
    {20, PERI_C, "ISC_D11", DIR_I}, {20, PERI_E, "FLEXCOM3_IO0", DIR_IO},
    {20, PERI_F, "A9", DIR_O},
    {21, PERI_A, "LCDDAT15", DIR_O}, {21, PERI_B, "GTXER", DIR_O},
    {21, PERI_C, "ISC_PCK", DIR_I}, {21, PERI_E, "FLEXCOM3_IO3", DIR_O},
    {21, PERI_F, "A10", DIR_O},
    {22, PERI_A, "LCDDAT18", DIR_O}, {22, PERI_B, "GCRS", DIR_I},
    {22, PERI_C, "ISC_VSYNC", DIR_I}, {22, PERI_E, "FLEXCOM3_IO4", DIR_O},
    {22, PERI_F, "A11", DIR_O},
    {23, PERI_A, "LCDDAT19", DIR_O}, {23, PERI_B, "GCOL", DIR_I},
    {23, PERI_C, "ISC_HSYNC", DIR_I}, {23, PERI_F, "A12", DIR_O},
    {24, PERI_A, "LCDDAT20", DIR_O}, {24, PERI_B, "GRX2", DIR_I},
    {24, PERI_C, "ISC_MCK", DIR_O}, {24, PERI_F, "A13", DIR_O},
    {25, PERI_A, "LCDDAT21", DIR_O}, {25, PERI_B, "GRX3", DIR_I},
    {25, PERI_C, "ISC_FIELD", DIR_I}, {25, PERI_F, "A14", DIR_O},
    {26, PERI_A, "LCDDAT22", DIR_O}, {26, PERI_B, "GTX2", DIR_O},
    {26, PERI_D, "CANTX1", DIR_O}, {26, PERI_F, "A15", DIR_O},
    {27, PERI_A, "LCDDAT23", DIR_O}, {27, PERI_B, "GTX3", DIR_O},
    {27, PERI_C, "PCK1", DIR_O}, {27, PERI_D, "CANRX1", DIR_I},
    {27, PERI_E, "TWD0", DIR_IO}, {27, PERI_F, "A16", DIR_O},
    {28, PERI_A, "LCDPWM", DIR_O}, {28, PERI_B, "FLEXCOM4_IO0", DIR_IO},
    {28, PERI_C, "PCK2", DIR_O}, {28, PERI_E, "TWCK0", DIR_IO},
    {28, PERI_F, "A17", DIR_O},
    {29, PERI_A, "LCDDISP", DIR_O}, {29, PERI_B, "FLEXCOM4_IO1", DIR_IO},
    {29, PERI_F, "A18", DIR_O},
    {30, PERI_A, "LCDVSYNC", DIR_O}, {30, PERI_B, "FLEXCOM4_IO2", DIR_IO},
    {30, PERI_F, "A19", DIR_O},
    {31, PERI_A, "LCDHSYNC", DIR_O}, {31, PERI_B, "FLEXCOM4_IO3", DIR_O},
    {31, PERI_C, "URXD3", DIR_I}, {31, PERI_F, "A20", DIR_O},
    {-1, -1, NULL, -1},         /* sentinel */
};

static struct periph_name piod_trans[] = {
    {0, PERI_A, "LCDPCK", DIR_O}, {0, PERI_B, "FLEXCOM4_IO4", DIR_O},
    {0, PERI_C, "UTXD3", DIR_O}, {0, PERI_D, "GTSUCOMP", DIR_O},
    {0, PERI_F, "A23", DIR_O},
    {1, PERI_A, "LCDDEN", DIR_O}, {1, PERI_D, "GRXCK", DIR_I},
    {1, PERI_F, "A24", DIR_O},
    {2, PERI_A, "URXD1", DIR_I}, {2, PERI_D, "GTXER", DIR_O},
    {2, PERI_E, "ISC_MCK", DIR_O}, {2, PERI_F, "A25", DIR_O},
    {3, PERI_A, "UTXD1", DIR_O}, {3, PERI_B, "FIQ", DIR_I},
    {3, PERI_D, "GCRS", DIR_I}, {3, PERI_E, "ISC_D11", DIR_I},
    {3, PERI_F, "NWAIT", DIR_I},
    {4, PERI_A, "TWD1", DIR_IO}, {4, PERI_B, "URXD2", DIR_I},
    {4, PERI_D, "GCOL", DIR_I}, {4, PERI_E, "ISC_D10", DIR_I},
    {4, PERI_F, "NCS0", DIR_O},
    {5, PERI_A, "TWCK1", DIR_IO}, {5, PERI_B, "UTXD2", DIR_O},
    {5, PERI_D, "GRX2", DIR_I}, {5, PERI_E, "ISC_D9", DIR_I},
    {5, PERI_F, "NCS1", DIR_O},
    {6, PERI_A, "TCK", DIR_I}, {6, PERI_B, "PCK1", DIR_O},
    {6, PERI_D, "GRX3", DIR_I}, {6, PERI_E, "ISC_D8", DIR_I},
    {6, PERI_F, "NCS2", DIR_O},
    {7, PERI_A, "TDI", DIR_I}, {7, PERI_C, "UTMI_RXVAL", DIR_O},
    {7, PERI_D, "GTX2", DIR_O}, {7, PERI_E, "ISC_D0", DIR_I},
    {7, PERI_F, "NWR1/NBS1", DIR_OO},
    {8, PERI_A, "TDO", DIR_O}, {8, PERI_C, "UTMI_RXERR", DIR_O},
    {8, PERI_D, "GTX3", DIR_O}, {8, PERI_E, "ISC_D1", DIR_I},
    {8, PERI_F, "NANDRDY", DIR_I}, {9, PERI_A, "TMS", DIR_I},
    {9, PERI_C, "UTMI_RXACT", DIR_O}, {9, PERI_D, "GTXCK", DIR_IO},
    {9, PERI_E, "ISC_D2", DIR_I},
    {10, PERI_A, "NTRST", DIR_I}, {10, PERI_C, "UTMI_HDIS", DIR_O},
    {10, PERI_D, "GTXEN", DIR_O}, {10, PERI_E, "ISC_D3", DIR_I},
    {11, PERI_A, "TIOA1", DIR_IO}, {11, PERI_B, "PCK2", DIR_O},
    {11, PERI_C, "UTMI_LS0", DIR_O}, {11, PERI_D, "GRXDV", DIR_I},
    {11, PERI_E, "ISC_D4", DIR_I}, {11, PERI_F, "ISC_MCK", DIR_O},
    {12, PERI_A, "TIOB1", DIR_IO}, {12, PERI_B, "FLEXCOM4_IO0", DIR_IO},
    {12, PERI_C, "UTMI_LS1", DIR_O}, {12, PERI_D, "GRXER", DIR_I},
    {12, PERI_E, "ISC_D5", DIR_I}, {12, PERI_F, "ISC_D4", DIR_I},
    {13, PERI_A, "TCLK1", DIR_I}, {13, PERI_B, "FLEXCOM4_IO1", DIR_IO},
    {13, PERI_C, "UTMI_CDRCPSEL0", DIR_I}, {13, PERI_D, "GRX0", DIR_I},
    {13, PERI_E, "ISC_D6", DIR_I}, {13, PERI_F, "ISC_D5", DIR_I},
    {14, PERI_A, "TCK", DIR_I}, {14, PERI_B, "FLEXCOM4_IO2", DIR_IO},
    {14, PERI_C, "UTMI_CDRCPSEL1", DIR_I}, {14, PERI_D, "GRX1", DIR_I},
    {14, PERI_E, "ISC_D7", DIR_I}, {14, PERI_F, "ISC_D6", DIR_I},
    {15, PERI_A, "TDI", DIR_I}, {15, PERI_B, "FLEXCOM4_IO3", DIR_O},
    {15, PERI_C, "UTMI_CDRCPDIVEN", DIR_I}, {15, PERI_D, "GTX0", DIR_O},
    {15, PERI_E, "ISC_PCK", DIR_I}, {15, PERI_F, "ISC_D7", DIR_I},
    {16, PERI_A, "TDO", DIR_O}, {16, PERI_B, "FLEXCOM4_IO4", DIR_O},
    {16, PERI_C, "UTMI_CDRBISTEN", DIR_I}, {16, PERI_D, "GTX1", DIR_O},
    {16, PERI_E, "ISC_VSYNC", DIR_I}, {16, PERI_F, "ISC_D8", DIR_I},
    {17, PERI_A, "TMS", DIR_I}, {17, PERI_C, "UTMI_CDRCPSELDIV", DIR_O},
    {17, PERI_D, "GMDC", DIR_O}, {17, PERI_E, "ISC_HSYNC", DIR_I},
    {17, PERI_F, "ISC_D9", DIR_I},
    {18, PERI_A, "NTRST", DIR_I}, {18, PERI_D, "GMDIO", DIR_IO},
    {18, PERI_E, "ISC_FIELD", DIR_I}, {18, PERI_F, "ISC_D10", DIR_I},
    {19, PERI_A, "PCK0", DIR_O}, {19, PERI_B, "TWD1", DIR_IO},
    {19, PERI_C, "URXD2", DIR_I}, {19, PERI_E, "I2SC0_CK", DIR_IO},
    {19, PERI_F, "ISC_D11", DIR_I},
    {20, PERI_A, "TIOA2", DIR_IO}, {20, PERI_B, "TWCK1", DIR_IO},
    {20, PERI_C, "UTXD2", DIR_O}, {20, PERI_E, "I2SC0_MCK", DIR_O},
    {20, PERI_F, "ISC_PCK", DIR_I},
    {21, PERI_A, "TIOB2", DIR_IO}, {21, PERI_B, "TWD0", DIR_IO},
    {21, PERI_C, "FLEXCOM4_IO0", DIR_IO}, {21, PERI_E, "I2SC0_WS", DIR_IO},
    {21, PERI_F, "ISC_VSYNC", DIR_I},
    {22, PERI_A, "TCLK2", DIR_I}, {22, PERI_B, "TWCK0", DIR_IO},
    {22, PERI_C, "FLEXCOM4_IO1", DIR_IO}, {22, PERI_E, "I2SC0_DI0", DIR_I},
    {22, PERI_F, "ISC_HSYNC", DIR_I},
    {23, PERI_A, "URXD2", DIR_I}, {23, PERI_C, "FLEXCOM4_IO2", DIR_IO},
    {23, PERI_E, "I2SC0_DO0", DIR_O}, {23, PERI_F, "ISC_FIELD", DIR_I},
    {24, PERI_A, "UTXD2", DIR_O}, {24, PERI_C, "FLEXCOM4_IO3", DIR_O},
    {25, PERI_A, "SPI1_SPCK", DIR_IO}, {25, PERI_C, "FLEXCOM4_IO3", DIR_O},
    {26, PERI_A, "SPI1_MOSI", DIR_IO},
    {26, PERI_C, "FLEXCOM2_IO0", DIR_IO},
    {27, PERI_A, "SPI1_MISO", DIR_IO}, {27, PERI_B, "TCK", DIR_I},
    {27, PERI_C, "FLEXCOM2_IO1", DIR_IO},
    {28, PERI_A, "SPI1_NPCS0", DIR_IO}, {28, PERI_B, "TDI", DIR_I},
    {28, PERI_C, "FLEXCOM2_IO2", DIR_IO},
    {29, PERI_A, "SPI1_NPCS1", DIR_O}, {29, PERI_B, "TDO", DIR_O},
    {29, PERI_C, "FLEXCOM2_IO3", DIR_O}, {29, PERI_D, "TIOA3", DIR_IO},
    {29, PERI_E, "TWD0", DIR_IO},
    {30, PERI_A, "SPI1_NPCS2", DIR_O}, {30, PERI_B, "TMS", DIR_I},
    {30, PERI_C, "FLEXCOM2_IO4", DIR_O}, {30, PERI_D, "TIOB3", DIR_IO},
    {30, PERI_E, "TWCK0", DIR_IO},
    {31, PERI_A, "ADTRG", DIR_I}, {31, PERI_B, "NTRST", DIR_I},
    {31, PERI_C, "IRQ", DIR_I}, {31, PERI_D, "TCLK3", DIR_I},
    {31, PERI_E, "PCK0", DIR_O},
    {-1, -1, NULL, -1},         /* sentinel */
};

static struct periph_name * bank_pn_arr[] = {
    pioa_trans,
    piob_trans,
    pioc_trans,
    piod_trans,
    NULL,       /* sentinel */
};

static const char * bank_str_arr[] = {
    "PA",
    "PB",
    "PC",
    "PD",
    NULL,       /* sentinel */
};

static const char * dir_arr[] = {"io", "i", "o", "output", " "};


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
        pr2serr("Usage: a5d2_pio_set [-b BN] [-d DIV] [-D DRVSTR] [-e] "
                "[-E EVT]\n"
                "                    [-f FUNC] [-F PHY1INT2B3] [-g|G] "
                "[-h] [-i|I]\n"
                "                    [-m|M] [-p PORT] [-r DIR] "
                "[-s FUNC] [-S LEV]\n"
                "                    [-t|T] [-u|U] [-uu|UU] [-v] [-V] "
                "[-w WPEN]\n"
                "                    [-X MSK,DAT] [-z|Z]\n"
                "  where the main options are:\n"
                "    -b BN        bit number within port (0 to 31). Also "
                "accepts full\n"
                "                 GPIO name (e.g. '-b PC7' equivalent to "
                "'-p c -b 7')\n"
                "    -D DRVSTR    IO drive: 0->LO, 1->LO, 2->ME, 3->HI; "
                "alternatively\n"
                "                 the letter L, M or H can be given\n"
                "    -e           enumerate pin names with corresponding "
                "kernel pin;\n"
                "                 use twice to to list peripheral names for "
                "each pin,\n"
                "                 use thrice to add direction indication to "
                "those names\n"
                "    -f FUNC      select line function: P->PIO, "
                "A->peri_A\n"
                "                 B->peri_B, C, D, E, F or G); "
                "alternatively\n"
                "                 FUNC may be a number: 0->PIO(GPIO), "
                "1->peri_A,\n"
                "                 2->peri_B and so on until 7->peri_G\n"
                "    -h           print usage message; use twice for "
                "more help\n"
                "    -i|I         interrupt disable|enable\n"
                "    -m|M         disable|enable open drain (formerly "
                "multi-drive)\n"
                "    -p PORT      port bank ('A' to 'D') or gpio kernel "
                "line number\n"
                "    -r DIR       direction: 0 -> pure input; 1 -> enabled "
                "for output\n"
                "                 also accepts 'I' for pure input and 'O' "
                "for output\n"
                "    -s FUNC      same as '-f FUNC'\n"
                "    -S LEV       set output data line to LEV (0 -> low, "
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
                "    -V           print version string then exit\n\n"
                "Set SAMA5D2 SoCs PIO attributes. Uses memory mapped IO to "
                "access PIO\nregisters directly; bypasses kernel. Use '-hh' "
                "for more.\n"
               );
    else    /* -hh */
        pr2serr("Usage: a5d2_pio_set [-b BN] [-d DIV] [-D DRVSTR] [-e] "
                "[-E EVT]\n"
                "                    [-f FUNC] [-F PHY1INT2B3] [-g|G] "
                "[-h] [-i|I]\n"
                "                    [-m|M] [-p PORT] [-r DIR] "
                "[-s FUNC] [-S LEV]\n"
                "                    [-t|T] [-u|U] [-uu|UU] [-v] [-V] "
                "[-w WPEN]\n"
                "                    [-X MSK,DAT] [-z|Z]\n\n"
                "  where the remaining options are:\n"
                "    -d DIV       slow clock divider [period=2*(DIV+1)"
                "*slow_clock_per]\n"
                "    -E EVT       EVT is input event: 0 -> falling edge, "
                "1 ->\n"
                "                 rising, 2 -> both edges, 3 -> low level, "
                "4 -> high\n"
                "    -F PHY1INT2B3    freeze config: 1 -> physical, "
                "2 -> interrupt\n"
                "                       3 -> physical+interrupt\n"
                "    -g|G         disable|enable (glitch) input filter\n"
                "    -w WPEN      write protect mode (for whole PIO) set to "
                "WPEN\n"
                "                 0->disabled (def, no write protection), "
                "1->enabled\n"
                "    -X MSK,DAT   write DAT to PORT for those lines set "
                "in MSK\n"
                "                 MSK and DAT are 32 bit hexadecimal "
                "values\n"
                "    -z|Z         disable|enable input filter slow clock\n\n"
                "Setting the output data line (e.g. with '-S 1') only changes "
                "the\nexternal line when FUNC is 0 (or 'P') . "
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
                "change to FUNC. The final three actions, if\nrequested, "
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

static char *
translate_peri(char * b, int max_blen, int pioc_num, int bit_num,
               int peri_num, bool show_dir)
{
    const struct periph_name * bpnp;
    const struct periph_name * pnp;

    if ((NULL == b) || (max_blen < 1))
        return b;
    b[max_blen - 1] = '\0';
    if ((bit_num < 0) || (bit_num > 31)) {
        snprintf(b, max_blen - 1, "bad bit_num=%d", bit_num);
        return b;
    }
    if ((peri_num < 0) || (peri_num > MAX_PERIPH_NUM)) {
        snprintf(b, max_blen - 1, "bad peri_num=%d", peri_num);
        return b;
    }
    if ((pioc_num < 0) || (pioc_num >= PIO_BANKS_SAMA5D2)) {
        snprintf(b, max_blen - 1, "bad poic_num=%d", pioc_num);
        return b;
    }
    if (0 == peri_num) {
        snprintf(b, max_blen - 1, "%s%d", bank_str_arr[pioc_num], bit_num);
        return b;
    }
    bpnp = bank_pn_arr[pioc_num];
    for (pnp = bpnp; pnp->pin >= 0; ++pnp) {
        if (pnp->pin > bit_num)
            break;
        else if (pnp->pin < bit_num)
            continue;
        for ( ; ((pnp->pin == bit_num) && (pnp->periph <= peri_num)); ++pnp) {
            if (pnp->periph < peri_num)
                continue;
            if (show_dir)
                snprintf(b, max_blen - 1, "%s [%s]", pnp->name,
                         dir_arr[pnp->dir]);
            else
                snprintf(b, max_blen - 1, "%s", pnp->name);
            return b;
        }
        b[0] = '\0';
        return b;       /* empty string denotes no match */
    }
    b[0] = '\0';
    return b;   /* empty string denotes no match */
}

static int
do_enumerate(int enum_val, int bank, int orig0, int do_dir)
{
    int k, j, m, n, num;
    char b[24];
    const char * cp;

    num = PIO_BANKS_SAMA5D2;
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
            printf("SAMA5D2: PIO %c:\n", 'A' + k);
            for (j = 0; j < LINES_PER_BANK; ++j) {
                cp = translate_peri(b, sizeof(b), k, j, 1, do_dir);
                printf("  P%c%d: %s", 'A' + k, j,
                       (strlen(cp) > 0) ? cp : "-");
                for (m = 2; m < 7; ++m) {
                    cp = translate_peri(b, sizeof(b), k, j, m, do_dir);
                    printf(", %s", (strlen(cp) > 0) ? cp : "-");
                }
                printf("\n");
            }
        }
    }
    return 0;
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
    if (bit_mask != *mmp) {
        *mmp = bit_mask;
        if (op->verbose > 1)
            pr2serr("  assert 0x%u in PIO_MSKR%d\n", bit_mask, pioc_num);
    }
    mmp = get_mmp(mem_fd, pio_cfgr[pioc_num], &mstat, op);
    if (op->verbose > 1) {
        unsigned int ui;

        ui = *mmp;
        pr2serr("  current PIO_CFGR%d value=0x%x\n", pioc_num, ui);
    }
    return mmp;
}

static int
do_set(int mem_fd, int bit_num, int pioc_num, const struct opts_t * op)
{
    unsigned int addr, bit_mask, ui;
    unsigned int cfgr;
    bool equal, cfgr_changed;
    volatile unsigned int * ommp;
    volatile unsigned int * cmmp = NULL;
    struct mmap_state mstat;

    memset(&mstat, 0, sizeof(mstat));
    bit_mask = 1 << bit_num;
    cfgr_changed = false;
    cfgr = 0;

    if (op->di_interrupt) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_idr[pioc_num], &mstat, op))))
            return 1;
        *ommp = bit_mask;
        if (op->verbose > 1)
            pr2serr("  disable interrupt: 0x%x in PIO_IDR%d\n", bit_mask,
                    pioc_num);
    }
    if (op->wp_given && (0 == op->wpen)) {
        if (NULL == ((ommp = get_mmp(mem_fd, PIO_WPMR, &mstat, op))))
            return 1;
        *ommp = (SAMA5D2_PIO_WPKEY << 8) | op->wpen;
        if (op->verbose > 1)
            pr2serr("  disable WPEN\n");
    }
    if (op->do_func >= 0) {
        if (NULL == cmmp) {
            cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op);
            if (NULL == cmmp)
                return 1;
            cfgr = *cmmp;
        }
        if ((unsigned int)op->do_func != (CFGR_FUNC_MSK & cfgr)) {
            cfgr = ((~CFGR_FUNC_MSK & cfgr) | op->do_func);
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert function=%d\n", op->do_func);
        }
    }
    if (op->dir_given) {
        if (NULL == cmmp) {
            cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op);
            if (NULL == cmmp)
                return 1;
            cfgr = *cmmp;
        }
        if (op->dir != !!(CFGR_DIR_MSK & cfgr)) {
            if (op->dir)
                cfgr |= CFGR_DIR_MSK;
            else
                cfgr &= ~CFGR_DIR_MSK;
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert direction=%d\n", op->dir);
        }
    }
    if (op->di_schmitt || op->en_schmitt) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_SCHMITT_MSK & cfgr) != !!op->di_schmitt) {
            if (op->di_schmitt)
                cfgr |= CFGR_SCHMITT_MSK;
            else
                cfgr &= ~CFGR_SCHMITT_MSK;
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert schmitt=%d\n", op->en_schmitt);
        }
    }
    if (op->scdr_given) {
        if (NULL == ((ommp = get_mmp(mem_fd, S_PIO_SCDR, &mstat, op))))
            return 1;
        *ommp = op->scdr_div;
        if (op->verbose > 1)
            pr2serr("  assert scdiv=%d in S_PIO_SCDR\n", op->scdr_div);
    }
    if (op->di_if_slow || op->en_if_slow) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_IFSCEN_MSK & cfgr) != !!op->en_if_slow) {
            if (op->en_if_slow)
                cfgr |= CFGR_IFSCEN_MSK;
            else
                cfgr &= ~CFGR_IFSCEN_MSK;
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert IFSCEN=%d\n", op->en_if_slow);
        }
    }
    if (op->di_if || op->en_if) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_IFEN_MSK & cfgr) != !!op->en_if) {
            if (op->en_if)
                cfgr |= CFGR_IFEN_MSK;
            else
                cfgr &= ~CFGR_IFEN_MSK;
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert IFEN=%d\n", op->en_if);
        }
    }
    if (op->evtsel_given) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if ((unsigned int)op->evtsel !=
            ((CFGR_EVTSEL_MSK & cfgr) >> CFGR_EVTSEL_SHIFT)) {
            cfgr = ((~CFGR_EVTSEL_MSK & cfgr) |
                    ((unsigned int)op->evtsel << CFGR_EVTSEL_SHIFT));
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert EVTSEL=%d\n", op->evtsel);
        }
    }
    if (op->di_opd || op->en_opd) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if (!!(CFGR_OPD_MSK & cfgr) != !!op->en_if) {
            if (op->en_opd)
                cfgr |= CFGR_OPD_MSK;
            else
                cfgr &= ~CFGR_OPD_MSK;
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert OPD=%d\n", op->en_opd);
        }
    }

    if ((op->di_pullup1dn2 > 0) || (op->en_pullup1dn2 > 0)) {
        bool pullup_en, pulldown_en, changed;

        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        pullup_en = !!(cfgr & ~CFGR_PUEN_MSK);
        pulldown_en = !!(cfgr & ~CFGR_PDEN_MSK);
        changed = false;

        /* Apply disables, if any, first */
        if (op->di_pullup1dn2 > 0) {
            if ((1 & op->di_pullup1dn2) && pullup_en) {
                cfgr &= ~CFGR_PUEN_MSK;
                changed = true;
            }
            if ((2 & op->di_pullup1dn2) && pulldown_en) {
                cfgr &= ~CFGR_PDEN_MSK;
                changed = true;
            }
            if (changed) {
                cfgr_changed = true;
                if (op->verbose > 1)
                    pr2serr("  P%c disable\n",
                            ((1 & op->di_pullup1dn2) ? 'U' : 'D'));
            }
        }
        if (op->en_pullup1dn2 > 0) {
            if ((1 & op->en_pullup1dn2) && (! pullup_en)) {
                cfgr |= CFGR_PUEN_MSK;
                changed = true;
            }
            if ((2 & op->en_pullup1dn2) && (! pulldown_en)) {
                cfgr |= CFGR_PDEN_MSK;
                changed = true;
            }
            if (changed) {
                cfgr_changed = true;
                if (op->verbose > 1)
                    pr2serr("  P%c enable\n",
                            ((1 & op->en_pullup1dn2) ? 'U' : 'D'));
            }
        }
    }
    if (op->out_level >= 0) {
        addr = ((op->out_level > 0) ? pio_sodr[pioc_num] : pio_codr[pioc_num]);
        if (NULL == ((ommp = get_mmp(mem_fd, addr, &mstat, op))))
            return 1;
        *ommp = bit_mask;
        if (op->verbose > 1)
            pr2serr("  %s output\n", (op->out_level ? "Set" : "Clear"));
    }
    if (op->drvstr_given) {
        if (NULL == cmmp) {
            if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num, op)))
                return 1;
            cfgr = *cmmp;
        }
        if ((unsigned int)op->drvstr !=
            ((CFGR_DRVSTR_MSK & cfgr) >> CFGR_DRVSTR_SHIFT)) {
            cfgr = ((~CFGR_DRVSTR_MSK & cfgr) |
                    ((unsigned int)op->drvstr << CFGR_DRVSTR_SHIFT));
            cfgr_changed = true;
            if (op->verbose > 1)
                pr2serr("  assert drvstr=%d\n", op->drvstr);
        }
    }
    if (cfgr_changed) {
        if (cmmp) {
            *cmmp = cfgr;
            if (op->verbose > 1)
                pr2serr("  cfgr changed so new PIO_CFGR%d=0x%x\n", pioc_num,
                        cfgr);
        } else
            pr2serr(">>> bad logic: cfgr changed but NULL cmmp\n");
    } else if (op->verbose > 2)
        pr2serr("  no change to PIO_CFGR%d\n", pioc_num);
    if (op->wr_dat_given) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_mskr[pioc_num], &mstat, op))))
            return 1;
        *ommp = op->msk;
        if (NULL == ((ommp = get_mmp(mem_fd, pio_odsr[pioc_num], &mstat, op))))
            return 1;
        ui = *ommp;
        equal = false;
        if ((ui & op->msk) != op->dat)
            *ommp = op->dat;
        else
            equal = true;
        if (op->verbose > 1)
            pr2serr("  prior PIO_ODSR%d=0x%x, msk=0x%x, dat=0x%x%s\n",
                    pioc_num, ui, op->msk, op->dat,
                    equal ? ", same so ignore" : "");
    }
    if (op->wp_given && op->wpen) {
        if (NULL == ((ommp = get_mmp(mem_fd, PIO_WPMR, &mstat, op))))
            return 1;
        *ommp = (SAMA5D2_PIO_WPKEY << 8) | op->wpen;
        if (op->verbose > 1)
            pr2serr("  disable WPEN\n");
    }
    if (op->en_interrupt) {
        if (NULL == ((ommp = get_mmp(mem_fd, pio_ier[pioc_num], &mstat, op))))
            return 1;
        *ommp = bit_mask;
        if (op->verbose > 1)
            pr2serr("  enable interrupt: 0x%x in PIO_IER%d\n", bit_mask,
                    pioc_num);
    }
    if (op->freeze_given) {
        if (0 == op->freeze_phy1int2b3) {
            if (op->verbose)
                pr2serr("  freeze ignored because PHY1INT2B3 is 0\n");
        } else {
            if (NULL == cmmp) {
                /* need pio_msk written to prior to write to pio_iofr */
                if (! (cmmp = do_mask_get_cfgr(mem_fd, bit_num, pioc_num,
                                               op)))
                    return 1;
            }
            if (NULL == ((ommp = get_mmp(mem_fd, pio_iofr[pioc_num], &mstat,
                                         op))))
                return 1;
            ui = 0;
            if (1 & op->freeze_phy1int2b3)
                ui |= IOFR_FPHY_MSK;
            if (2 & op->freeze_phy1int2b3)
                ui |= IOFR_FINT_MSK;
            *ommp = (SAMA5D2_PIO_FRZKEY << 8) | ui;
            if (op->verbose > 1) {
                switch (op->freeze_phy1int2b3) {
                case 1:
                    pr2serr("  set IOFR_FPHY\n");
                    break;
                case 2:
                    pr2serr("  set IOFR_FINT\n");
                    break;
                case 3:
                    pr2serr("  set IOFR_FPHY+IOFR_FINT\n");
                    break;
                default:
                    pr2serr("  >> unexpected freeze_phy1int2b3 value %d\n",
                            op->freeze_phy1int2b3);
                    break;
                }
            }
        }
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
    int c, k, mem_fd, pioc_num, res;
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
    op->out_level = -1;
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
                pr2serr("'-d' expects a value from 0 to 16383\n");
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
            if ((k < 0) || (k > 3)) {
                pr2serr("'-F' expects a value of 0, 1, 2 or 3\n");
                return 1;
            }
            op->freeze_phy1int2b3 = k;
            op->freeze_given = true;
            break;
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
            if (('-' == optarg[0]) && ('1' == optarg[1]) &&
                ('\0' == optarg[2])) {
                op->out_level = -1;
                break;
            }
            if (! isdigit(*optarg)) {
                pr2serr("'-S' expects LEV to be 0 or 1\n");
                return 1;
            }
            k = atoi(optarg);
            if ((k < 0) || (k > 1)) {
                pr2serr("'-S' expects LEV to be 0 or 1\n");
                return 1;
            }
            op->out_level = k;
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
            break;
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

    if (op->enumerate)
        return do_enumerate(op->enumerate, bank, origin0, op->enumerate > 2);

    if (op->wr_dat_given) {
        if ('\0' == bank) {
            pr2serr("With '-X MSK,DAT' require '-p PORT' since it will "
                    "potentially\nwrite to all 32 lines in that bank\n");
            return 1;
        }
    }
    if (knum >= 0) {
        if (bit_num >= 0) {
            pr2serr("Give either '-p PORT' or ('-b BN' and '-p PORT') "
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
                pr2serr("If '-p PORT' given then also need '-b BN'\n");
                return 1;
            }
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        pr2serr("Need to give gpio line with '-p PORT' and/or "
                "'-b BN'\n");
        goto help_exit;
    }
    if ((!!op->di_if + !!op->en_if) > 1) {
        pr2serr("Can only have one of '-g' and '-G'\n");
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

    res = do_set(mem_fd, bit_num, pioc_num, op);

    if (mem_fd >= 0)
        close(mem_fd);
    return res;

help_exit:
    pr2serr(">>> Use '-h' for command line syntax, '-hh' for other help.\n");
    return EXIT_FAILURE;
}
