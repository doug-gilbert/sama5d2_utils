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
 * a5d2_pio_status.c
 *
 * Utility for fetching SAMA5D2 family SoC PIO status values.
 * The SAMA5D2 family has 4 PIOs each with 32 gpio lines: PA0-PA31, PB0-PB31,
 * PC0-PC31 and PD0-PD31.
 * This utility uses memory mapped IO.
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


static const char * version_str = "1.01 20160123";


#define PIO_BANKS_SAMA5D2 4  /* PIOA, PIOB, PIOC and PIOD */
#define LINES_PER_BANK 32
#define MAP_SIZE 4096   /* assume to be power of 2 */
#define MAP_MASK (MAP_SIZE - 1)
#define DEV_MEM "/dev/mem"

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
#define CFGR_DRVSTR_MSK 0x30000         /* 0, 1 -> LO; 2 -> ME; 3 -> HI */
#define CFGR_DRVSTR_SHIFT 16
#define CFGR_EVTSEL_MSK 0x7000000       /* 0 -> falling; 1 -> rising, ... */
#define CFGR_EVTSEL_SHIFT 24
#define CFGR_PCFS_MSK (1 << 29) /* physical configuration freezes status */
#define CFGR_ICFS_MSK (1 << 30) /* interrupt configuration freezes status */

#define GPIO_BANK_ORIGIN "/sys/class/gpio/gpiochip0"
/* Earlier kernels (G20+G25) offset GPIO numbers by 32 so
 * /sys/class/gpio/gpiochip32 (a directory) was the lowest numbered
 * bank and corresponded to PIOA. Now (in lk 4.4)
 * /sys/class/gpio/gpiochip0 exists and corresponds to PIOA.
 *
 * SAMA5D2 base PIO addresses are 0xfc038000 (A), 0xfc038040 (B),
 * 0xfc038080 (C) and 0xfc0380c0 (D). The secure banks are 0xfc039000 (A),
 * 0xfc039040 (B), 0xfc039080 (C) and 0xfc0390c0 (D).
 */


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
static unsigned int pio_pdsr[] = {0xfc038008, 0xfc038048, 0xfc038088,
                        0xfc0380c8};    /* Pin data status (ro) */
static unsigned int pio_locksr[] = {0xfc03800c, 0xfc03804c, 0xfc03808c,
                        0xfc0380cc};    /* Lock status (ro) */
#if 0
static unsigned int pio_sodr[] = {0xfc038010, 0xfc038050, 0xfc038090,
                        0xfc0380d0};    /* Set output data (wo) */
static unsigned int pio_codr[] = {0xfc038014, 0xfc038054, 0xfc038094,
                        0xfc0380d4};    /* Clear output data (wo) */
#endif
static unsigned int pio_odsr[] = {0xfc038018, 0xfc038058, 0xfc038098,
                        0xfc0380d8};    /* Output data status (rw) */
#if 0
static unsigned int pio_ier[] = {0xfc038020, 0xfc038060, 0xfc0380a0,
                        0xfc0380e0};    /* Interrupt enable (wo) */
static unsigned int pio_idr[] = {0xfc038024, 0xfc038064, 0xfc0380a4,
                        0xfc0380e4};    /* Interrupt disable (wo) */
#endif
static unsigned int pio_imr[] = {0xfc038028, 0xfc038068, 0xfc0380a8,
                        0xfc0380e8};    /* Interrupt mask (ro) */
static unsigned int pio_isr[] = {0xfc03802c, 0xfc03806c, 0xfc0380ac,
                        0xfc0380ec};    /* Interrupt status (ro) */
#if 0
static unsigned int pio_iofr[] = {0xfc03803c, 0xfc03807c, 0xfc0380bc,
                        0xfc0380fc};    /* I/O freeze (wo) */

static unsigned int s_pio_sionr[] = {0xfc039030, 0xfc039070, 0xfc0390b0,
                        0xfc0390f0};    /* Secure I/O non-secure (wo) */
static unsigned int s_pio_siosr[] = {0xfc039034, 0xfc039074, 0xfc0390b4,
                        0xfc0390f4};    /* Secure I/O secure (wo) */
static unsigned int s_pio_iossr[] = {0xfc039038, 0xfc039078, 0xfc0390b8,
                        0xfc0390f8};    /* Secure I/O security status (ro) */
static unsigned int s_pio_iofr[] = {0xfc03903c, 0xfc03907c, 0xfc0390bc,
                        0xfc0390fc};    /* Secure I/O freeze (wo) */
#endif

static int verbose = 0;

static const char * driv_arr[] = {"LO_DRIVE", "LO_DRIVE", "ME_DRIVE",
                                  "HI_DRIVE"};
static const char * dir_arr[] = {"io", "i", "o", "output", " "};
static const char * evtsel_arr[] = {"falling edge", "rising edge",
                                    "both edges", "low level", "high level",
                                    "reserved [5]", "reserved [6]",
                                    "reserved [7]"};

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

struct periph_name * bank_pn_arr[] = {
    pioa_trans,
    piob_trans,
    pioc_trans,
    piod_trans,
    NULL,       /* sentinel */
};

const char * bank_str_arr[] = {
    "PA",
    "PB",
    "PC",
    "PD",
    NULL,       /* sentinel */
};


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
usage(int hval)
{
    if (1 == hval) {
        pr2serr("Usage: a5d2_pio_status [-a] [-b BN] [-B] [-d] [-e] "
                "[-f STR] [-h]\n"
                "                       [-i] [-p PORT] [-s] [-S] [-t] [-v] "
                "[-V] [-w]\n"
                "  where:\n"
                "    -a           list all lines within a bank (def: "
                "'-p A')\n"
                "    -b BN        bit (line) number within port (0 to 31). "
                "Also\n"
                "                 accepts prefix like 'pb' or just 'b' for "
                "PORT.\n"
                "                 Example: '-b PC7' equivalent to '-p c "
                "-b 7'\n"
                "    -B           brief ouput (e.g. 'func=1 puen=1 ...'). "
                "Use twice\n"
                "                 for single line output; thrice for name "
                "only\n"
                "    -d           show direction on peripheral translates\n"
                "    -e           enumerate pin names with corresponding "
                "kernel pin.\n"
                "                 Use twice to list peripheral names for "
                "each pin\n"
                "    -f STR       find peripheral names containing STR "
                "(list\n"
                "                 then exit). If STR is a line name (e.g. "
                "PC3)\n"
                "                 then list peripherals for that line\n"
                "    -h           print usage message, use twice for "
                "abbreviations\n"
                "    -i           read interrupt status register which "
                "then clears it\n"
                "    -p PORT      port bank ('A' to 'D') or gpio kernel "
                "line number\n"
                "                 0 -> PA0, 1 -> PA1 ... 127 -> PD31\n"
                "    -s           summarize all lines in a bank, equivalent "
                "to\n"
                "                 '-a -BB -t'. Example: 'a5d2_pio_status "
                "-s -p C'\n"
                "    -S           show all selected line names within all "
                "banks.\n"
                "                 Use twice to append direction "
                "annotations\n"
                "    -t           translate peripheral type to functional "
                "name\n"
                "                 (e.g. PD15 peri_b -> FLEXCOM4_IO3)\n"
                "    -v           increase verbosity (multiple times for "
                "more)\n"
                "    -V           print version string then exit\n");
        pr2serr("    -w           read write protect status register "
                "which clears it\n");
        pr2serr("\nSAMA5D2x SoC PIO fetch status program. "
                "Uses memory mapped\nIO to fetch PIO registers and shows "
                "settings for given line(s). Try\n'-hh' for more help.\n");
    } else {
        pr2serr(">> Abbreviation explanations\n"
                "driv:     line drive strength [def: 0 -> lo; 1 -> lo; "
                "2 -> me;\n"
                "          3 -> hi]. For 3.3 volts: 2, 2, 20, 32 mA "
                "respectively\n"
                "evtsel:   event selection on input (def: 0 -> falling]\n"
                "func:     function of pin (0 -> GPIO; 1 -> peri_a; etc)\n"
                "icfs:     interrupt configuation freeze status [def: 0 -> "
                "none]\n"
                "ifen:     input filter enabled [def: 0 -> "
                "disabled]\n"
                "im:       interrupt mask [def: 0 -> disabled]\n"
                "is:       interrupt status [def: 0 -> no change]\n"
                "locks:    lock status [def: 0 -> unlocked]\n"
                "ods:      output data status [def: 0 -> level 0 to "
                "be driven]\n"
                "opd:      open drain status [def: 0 -> disabled: "
                "driven high+low]\n"
                "pcfs:     physical configuation freeze status [def: 0 -> "
                "none]\n"
                "pden:     pull-down status [def: 0 -> disabled]\n"
                "pds:      pin data status [0 -> line is at level 0; "
                "1 -> level 1]\n"
                "puen:     pull-up status [def: 0 -> disabled]\n"
                "scd**:    slow clock divider (debouncing) [def: 0; per "
                "PIO]\n"
                "schmitt*: schmitt trigger [def: 0 -> enabled]\n"
                "wpm**:    write protect mask [def: 0 -> PIO writeable]\n"
                "wps**:    write protect status [def: 0 -> no violation "
                "on PIO]\n"
               );
        pr2serr("\nAbbreviations with a trailing '*' have "
                "the corresponding function\nenabled when the value "
                "is 0 (i.e. negated logic). For example\n'schmitt*=1' "
                "means the schmitt trigger is disabled. The trailing '**' "
                "\nmeans the register is for all PIOs rather than per "
                "GPIO line. An\nentry like 'is=-1' means that is "
                "(the interrupt status register)\nhas not been read.\n"
               );
    }
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
check_mmap(int mem_fd, unsigned int wanted_addr, struct mmap_state * msp)
{
    off_t mask_addr;

    mask_addr = (wanted_addr & ~MAP_MASK);
    if ((0 == msp->mmap_ok) || (msp->prev_mask_addrp != mask_addr)) {
        if (msp->mmap_ok) {
            if (-1 == munmap(msp->mmap_ptr, MAP_SIZE)) {
                pr2serr("mmap_ptr=%p:\n", msp->mmap_ptr);
                perror("    munmap");
                return NULL;
            } else if (verbose > 2)
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
        if (verbose > 2)
            pr2serr("mmap() ok, mask_addr=0x%lx, mmap_ptr=%p\n", mask_addr,
                    msp->mmap_ptr);
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
pio_status(int mem_fd, unsigned int bit_mask, int bit_num, int brief,
           int interrupt, int translate, int pioc_num, int write_prot,
           int do_dir)
{
    int ifen, ifscen, ods, pds, im, is, opd, puen, scd, pden, evtsel;
    int wpm, wps, schmitt, driv, func, pcfs, icfs;
    unsigned int cfgr, locks;
    struct mmap_state mstat;
    volatile unsigned int * mmp;
    const char * cp;
    char b[32];
    char e[32];


    memset(&mstat, 0, sizeof(mstat));

    if (NULL == ((mmp = get_mmp(mem_fd, pio_mskr[pioc_num], &mstat))))
        return 1;
    if (*mmp != bit_mask)
        *mmp = bit_mask;
    if (NULL == ((mmp = get_mmp(mem_fd, pio_cfgr[pioc_num], &mstat))))
        return 1;
    cfgr = *mmp;
    func = (cfgr & CFGR_FUNC_MSK);

    ifen = !!(CFGR_IFEN_MSK & cfgr);
    ifscen = !!(CFGR_IFSCEN_MSK & cfgr);
    if (0 == brief) {
        if (0 == func)
            printf("  function: GPIO ACTIVE [0]\n");
        else {
            printf("  peripheral function: %c ", 'A' + func - 1);
            if (translate) {
                translate_peri(b, sizeof(b), pioc_num, bit_num, func, do_dir);
                if (0 == strlen(b))
                    printf("[-]\n");
                else
                    printf("[%s]\n", b);
            } else
                printf("[%d]\n", func);
        }
        printf("  direction: %s\n", (CFGR_DIR_MSK & cfgr) ?
                         "line enabled as output" : "line pure input");
        if (0 == ifen)
            printf("  input filter disabled\n");
        else
            printf("  input filter %senabled\n",
                   (ifscen ? "slow clock " : ""));
    }
    if (NULL == ((mmp = get_mmp(mem_fd, pio_odsr[pioc_num], &mstat))))
        return 1;
    ods = !!(*mmp & bit_mask);
    if (0 == brief) {
        if (func || (0 == (CFGR_DIR_MSK & cfgr)))
            printf("  [output data status: %d]\n", ods);
        else
            printf("  output data status: %d\n", ods);
    }
    if (NULL == ((mmp = get_mmp(mem_fd, pio_pdsr[pioc_num], &mstat))))
        return 1;
    pds = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  pin data status: %d\n", pds);
    if (NULL == ((mmp = get_mmp(mem_fd, pio_imr[pioc_num], &mstat))))
        return 1;
    im = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  interrupt mask: %d (%s)\n", im,
               (im ? "ENabled" : "DISabled"));

    if (interrupt) {
        if (NULL == ((mmp = get_mmp(mem_fd, pio_isr[pioc_num], &mstat))))
            return 1;
        is = !!(*mmp & bit_mask);
        if (0 == brief)
            printf("  interrupt status: %d (%s)\n", is,
                   (is ? "input CHANGE" : "NO input change"));
        if ((verbose > 1) && is)
            printf("  >>> N.B. that interrupt has now been cleared\n");
    } else
        is = -1;

    opd = !!(CFGR_OPD_MSK & cfgr);
    if (0 == brief)
        printf("  open drain: %d (%s)\n", opd,
               (opd ? "enabled, pin driven when low" :
               "pin driven high and low"));

    puen = !!(CFGR_PUEN_MSK & cfgr);
    if (0 == brief)
        printf("  pull-up status: %d (%s)\n", puen,
               (puen ? "ENabled" : "DISabled"));

    if (CFGR_IFSCEN_MSK & cfgr) {
        if (NULL == ((mmp = get_mmp(mem_fd, S_PIO_SCDR, &mstat))))
            return 1;
        scd = *mmp & 0x3fff;
        if (0 == brief)
            printf("  (secure) slow clock divider debouncing register: %d "
                   "[0x%x]\n", scd, scd);
    } else
        scd = -1;
    pden = !!(CFGR_PDEN_MSK & cfgr);
    if (0 == brief)
        printf("  pull down status: %d (%s)\n", pden,
               (pden ? "ENabled" : "DISabled"));

    evtsel = (CFGR_EVTSEL_MSK & cfgr) >> CFGR_EVTSEL_SHIFT;
    if (0 == brief) {
        if (im)
            printf("  input event: %s\n", evtsel_arr[evtsel]);
        else
            printf("  [input event: %s]\n", evtsel_arr[evtsel]);
    }
    if (NULL == ((mmp = get_mmp(mem_fd, pio_locksr[pioc_num], &mstat))))
        return 1;
    locks = !!(*mmp & bit_mask);
    if (0 == brief)
        printf("  locked status: %d (%slocked)\n", locks,
               (locks ? "" : "not "));

    if (NULL == ((mmp = get_mmp(mem_fd, PIO_WPMR, &mstat))))
        return 1;
    wpm = *mmp;
    if (0 == brief)
        printf("  write protect mode: WPEN: %d (%s)\n",
               wpm & 1, ((wpm & 1) ? "ENabled" : "DISabled"));
    if (write_prot) {
        if (NULL == ((mmp = get_mmp(mem_fd, PIO_WPSR, &mstat))))
            return 1;
        wps = *mmp & 0xffffff;
        if (0 == brief)
            printf("  write protect violation status: %d (%s), WPCSRC: "
                   "0x%x\n", (wps & 1), ((wps & 1) ? "VIOLATED" :
                   "NOT violated"), (wps >> 8) & 0xffff);
    } else
        wps = -1;
    schmitt = !!(CFGR_SCHMITT_MSK & cfgr);
    if (0 == brief)
        printf("  schmitt trigger: %d (%s)\n", schmitt,
               (schmitt ? "DISabled" : "ENabled "));
    driv = (CFGR_DRVSTR_MSK & cfgr) >> CFGR_DRVSTR_SHIFT;
    if (0 == brief)
        printf("  IO drive: %d (%s)\n", driv, driv_arr[driv]);

    pcfs = !!(CFGR_PCFS_MSK & cfgr);
    icfs = !!(CFGR_ICFS_MSK & cfgr);
    if (0 == brief) {
        printf("  physical configuration freeze status: %d (%s)\n", pcfs,
               (pcfs ? "FROZEN" : "not frozen"));
        printf("  interrupt configuration freeze status: %d (%s)\n", icfs,
               (icfs ? "FROZEN" : "not frozen"));
    }

    if (1 == brief) {
        if (0 == func) {
            if (translate)
                printf("GPIO ");
            else
                printf("func=0 ");
            printf("pds=%d ods=%d opd=%d ifen=%d im=%d is=%d puen=%d\n"
                   "ifscen=%d scd=%d pden=%d evtsel=%d locks=%d wpm=0x%x",
                   pds, ods, opd, ifen, im, is, puen, ifscen, scd, pden,
                   evtsel, locks, wpm);
            if (write_prot)
                printf(" wps=0x%x", wps);
            if (pcfs)
                printf(" pcfs=1");
            if (icfs)
                printf(" icfs=1");
            printf("\nschmitt*=%d io_driv=%d\n", schmitt, driv);
        } else {
            if (translate) {
                snprintf(e, sizeof(e), "PERI_%c", 'A' + func - 1);
                cp = e;
                b[0] = '\0';
                translate_peri(b, sizeof(b), pioc_num, bit_num, func, do_dir);
                if (strlen(b) > 0)
                    cp = b;
                printf("%s pds=%d [ods=%d]", cp, pds, ods);
            } else
                printf("func=%d pds=%d [ods=%d]", func, pds, ods);
            printf("opd=%d ifen=%d im=%d is=%d puen=%d\n"
                   "ifscen=%d scd=%d pden=%d evtsel=%d\nlocks=%d wpm=0x%x",
                   opd, ifen, im, is, puen, ifscen, scd, pden, evtsel, locks,
                   wpm);
            if (write_prot)
                printf(" wps=0x%x", wps);
            printf(" schmitt*=%d io_driv=%d\n", schmitt, driv);
        }
    } else if (brief > 1) {
        if (0 == func)
            cp = "GPIO";
        else {
            snprintf(e, sizeof(e), "PERI_%c", 'A' + func - 1);
            cp = e;
            if (translate) {
                b[0] = '\0';
                translate_peri(b, sizeof(b), pioc_num, bit_num, func, do_dir);
                if (strlen(b) > 0)
                    cp = b;
            }
        }
        if (2 == brief) {
            if (0 == func)
                printf(" %-2d: %s pds=%d ods=%d opd=%d ifen=%d "
                       "puen=%d%s\n", bit_num, cp, pds, ods, opd,
                       ifen, puen, ((1 == pden) ? " pden=1" : ""));
            else
                printf(" %-2d: %s pds=%d opd=%d ifen=%d puen=%d%s\n",
                       bit_num, cp, pds, opd, ifen, puen,
                       ((1 == pden) ? " pden=1" : ""));
        } else {
            if (0 == func)
                printf(" %-2d: %s pds=%d ods=%d\n", bit_num, cp, pds, ods);
            else
                printf(" %-2d: %s pds=%d opd=%d\n", bit_num, cp, pds, opd);
        }
    }

    if (mstat.mmap_ok) {
        if (-1 == munmap(mstat.mmap_ptr, MAP_SIZE)) {
            pr2serr("mmap_ptr=%p:\n", mstat.mmap_ptr);
            perror("    munmap");
            return 1;
        } else if (verbose > 2)
            pr2serr("trailing munmap() ok, mmap_ptr=%p\n", mstat.mmap_ptr);
    }
    return 0;
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

static int
do_show_all(int show_val, int do_dir)
{
    int k, j, n, num;
    int res = 1;
    int mem_fd = -1;
    unsigned int bit_mask, cfgr;
    char b[32];
    size_t blen = sizeof(b) - 1;
    void * mmap_ptr = (void *)-1;
    struct mmap_state mstat;
    volatile unsigned int * mmp;
    bool dir = (do_dir > 0) || (show_val > 1);

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        perror("open of " DEV_MEM " failed");
        return 1;
    } else if (verbose > 2)
        printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");
    memset(&mstat, 0, sizeof(mstat));

    num = PIO_BANKS_SAMA5D2;
    printf("PIN  PIO_A             PIO_B             PIO_C             "
           "PIO_D\n");
    for (k = 0; k < LINES_PER_BANK; ++k) {
        if (k > 9)
            printf("%d:  ", k);
        else
            printf("%d:   ", k);
        bit_mask = 1 << k;
        for (j = 0; j < num; ++j) {
            mmap_ptr = check_mmap(mem_fd, pio_mskr[j], &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            mmp = mmp_add(mmap_ptr, pio_mskr[j] & MAP_MASK);
            if (bit_mask != *mmp)
                *mmp = bit_mask;
            mmap_ptr = check_mmap(mem_fd, pio_cfgr[j], &mstat);
            if (NULL == mmap_ptr)
                goto clean_up;
            mmp = mmp_add(mmap_ptr, pio_cfgr[j] & MAP_MASK);
            cfgr = *mmp;
            if (0 == (CFGR_FUNC_MSK & cfgr))
                snprintf(b, blen, "%s", "GPIO");
            else {
                n = (0x7 & cfgr);
                translate_peri(b, blen, j, k, n, dir);
                if (strlen(b) < 1)
                    snprintf(b, blen, "P%c%d: sel=%d", 'A' + j, k, n);
            }
            printf("%-18s", b);
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
    int do_dir = 0;
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
    const char * str = NULL;
    char ch;
    char bank = '\0';

    while ((opt = getopt(argc, argv, "ab:Bdef:hip:sStvVw")) != -1) {
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
                if ((ch >= 'A') && (ch <= 'D'))
                    bank = ch;
                else {
                    pr2serr("'-b' expects a letter ('A' to 'D')\n");
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
        case 'B':
            ++brief;
            ++brief_given;
            break;
        case 'd':
            ++do_dir;
            break;
        case 'e':
            ++enumerate;
            break;
        case 'f':
            str = optarg;
            break;
        case 'h':
            ++do_help;
            break;
        case 'i':
            ++interrupt;
            break;
        case 'p':
            cp = optarg;
            if (isalpha(cp[0])) {
                ch = toupper(cp[0]);
                if ((ch >= 'A') && (ch <= 'D'))
                    bank = ch;
                else {
                    pr2serr("'-p' expects a letter ('A' to 'D')\n");
                    exit(EXIT_FAILURE);
                }
            } else if (isdigit(*optarg)) {
                k = atoi(optarg);
                if ((k < 0) || (k > 159)) {
                    pr2serr("'-p' expects a letter or a number 0 to 159\n");
                    exit(EXIT_FAILURE);
                }
                knum = k;
            } else {
                pr2serr("'-p' expects a letter ('A' to 'D') or a number\n");
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
                pr2serr("Unexpected extra argument: %s\n", argv[optind]);
            do_help = 1;
            ret = 1;
        }
    }
    if (do_help) {
        usage(do_help);
        exit(ret ? EXIT_FAILURE : EXIT_SUCCESS);
    }
    if (str) {  /* -f <str>  */
        struct periph_name ** bpnpp;
        struct periph_name * pnp;
        char mystr[16];
        bool got_one;

        if ((strlen(str) >= 3) && ('P' == toupper(str[0])) &&
            (toupper(str[1]) >= 'A') && (toupper(str[1]) <= 'D') &&
            isdigit(str[2])) {
            if ((1 != sscanf(str + 2, "%d", &k)) || (k < 0) || (k > 31)) {
                pr2serr("expected to find PIO name like 'PC13' but didn't\n");
                return 1;
            }
            pnp = bank_pn_arr[toupper(str[1]) - 'A'];
            for (got_one = false ; pnp->pin >= 0; ++pnp) {
                if (pnp->pin > k)
                    break;
                if (pnp->pin < k)
                    continue;
                if (! got_one) {
                    got_one = true;
                    printf("P%c%d: ", toupper(str[1]), k);
                } else
                    printf(", ");
                if (do_dir)
                    printf("<%c> %s [%s]", pnp->periph + 'A' - 1, pnp->name,
                           dir_arr[pnp->dir]);
                else
                    printf("%c>> %s", pnp->periph + 'A' - 1, pnp->name);
            }
            if (got_one)
                printf("\n");
            return 0;
        }

        for (k = 0; k < (int)strlen(str); ++k) {
            if (k >= (int)(sizeof(mystr) - 1))
                break;
            if (islower(str[k]))
                mystr[k] = toupper(str[k]);
            else
                mystr[k] = str[k];
        }
        mystr[k] = '\0';

        for (bpnpp = bank_pn_arr, k = 0; *bpnpp; ++bpnpp, ++k) {
            if (bank && (k != bank - 'A'))
                continue;
            for (pnp = *bpnpp; pnp->pin >= 0; ++pnp) {
                if (strstr(pnp->name, mystr)) {
                    printf("P%c%d[%c]: %s\n", 'A' + k, pnp->pin,
                           'A' + pnp->periph - 1, pnp->name);

                }
            }
        }
        return 0;
    }

    if (stat(GPIO_BANK_ORIGIN, &sb) >= 0) {
        if (verbose > 1)
            pr2serr("%s found so kernel pin numbers start at 0 (for PA0)\n",
                    GPIO_BANK_ORIGIN);
        ++origin0;
    } else if (verbose > 2)
        pr2serr("%s not found so kernel pin numbers start at 32 (for PA0)\n",
                GPIO_BANK_ORIGIN);

    if (enumerate)
        return do_enumerate(enumerate, bank, origin0, do_dir);
    if (show_all)
        return do_show_all(show_all, do_dir);

    if (knum >= 0) {
        if (bit_num >= 0) {
            pr2serr("Give either '-p <knum>' or ('-b <bn>' and '-p <bank>') "
                    "but not both\n");
            exit(EXIT_FAILURE);
        }
        if ((! origin0) && (knum < 32)) {
            pr2serr("since %s not found assume kernel pin numbers start at "
                    "32\n(for PA0) so %d is too low\n", GPIO_BANK_ORIGIN,
                    knum);
            exit(EXIT_FAILURE);
        }
    } else if (bank) {
        if (do_all)
            knum = origin0 ? 0 : 32;
        else if (bit_num < 0) {
            if (write_prot)
                knum = origin0 ? 0 : 32;
            else {
                pr2serr("If '-p <bank>' given then also need '-b <bn>'\n");
                exit(EXIT_FAILURE);
            }
        } else
            knum = (((! origin0) + bank - 'A') * 32) + bit_num;
    } else {
        if (do_all) {
            printf(">>> Assuming bank A, use '-p <port>' to change\n");
            knum = origin0 ? 0 : 32;
        } else {
            pr2serr("Need to give gpio line with '-p <port>' and/or "
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
                printf("%s%d:\n", bank_str_arr[pioc_num], bit_num);
            res = pio_status(mem_fd, bit_mask, bit_num, brief, interrupt,
                             translate, pioc_num, write_prot, do_dir);
            if (res)
                break;
        }
    } else {
        if (brief < 2)
            printf("%s%d:\n", bank_str_arr[pioc_num], bit_num);
        res = pio_status(mem_fd, bit_mask, bit_num, brief, interrupt,
                         translate, pioc_num, write_prot, do_dir);
    }

    if (mem_fd >= 0)
        close(mem_fd);
    return res;
}
