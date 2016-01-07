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

/* This utility sets an exit status of 0 when __ARM_EABI__ defined and the
 * device tree indicates it is in the SAMA5D2 family. If string not found
 * returns 1.
 */

#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>


static const char * version_str = "0.91 20160104";

#define FAM_NAME "SAMA5D2"
#define FAM_NAME_LC "sama5d2"

#define CPUINFO "/proc/cpuinfo"
#define DEVTREE_MODEL "/proc/device-tree/model"
#define DEVTREE_COMPAT "/proc/device-tree/compatible"

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "is_%s [-h] [-p] [-v] [-V]\n"
            "  where:\n"
            "    -h           print usage message\n"
            "    -p           prints '0' to stdout if in %s family "
            "else prints '1'\n"
            "    -v           increase verbosity\n"
            "    -V           print version string then exit\n\n"
            "Check to see if '%s' in the device-tree model or compatible\n"
            "string. If so assume this is a %s family SoC and set\n"
            "exit status to 0 (true for scripts). Otherwise set an exit\n"
            "status of 1. When '-p' noption given also send the same value "
            "to stdout.\n", FAM_NAME_LC, FAM_NAME, FAM_NAME, FAM_NAME);
}


int
main(int argc, char *argv[])
{
    int opt;
    int print_stdout = 0;
    int ret = 1;
#ifdef __ARM_EABI__
    int n, num;
    FILE *fp;
    char b[1024];
    const char * ccp;
    struct stat a_stat;
#endif

    while ((opt = getopt(argc, argv, "hpvV")) != -1) {
        switch (opt) {
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        case 'p':
            ++print_stdout;
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
            break;
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
#ifdef __ARM_EABI__
    if (0 == stat(DEVTREE_MODEL, &a_stat)) {
        if ((fp = fopen(DEVTREE_MODEL, "r"))) {
            num = fread(b, 1, sizeof(b) - 1, fp);
            if (num > 2) {
                b[num] = '\0';
                if (strstr(b, FAM_NAME) || strstr(b, FAM_NAME_LC)) {
                    if (verbose > 2)
                        fprintf(stderr, "'%s' found in model line: %s\n",
                                FAM_NAME, b);
                    ret = 0;
                }
            } else if (verbose && ferror(fp))
                fprintf(stderr, "Failed to read: %s\n", DEVTREE_MODEL);
            fclose(fp);
        } else if (verbose)
            fprintf(stderr, "Failed to open: %s\n", DEVTREE_MODEL);
        if (1 == ret) {
            if ((fp = fopen(DEVTREE_COMPAT, "r"))) {
                num = fread(b, 1, sizeof(b) - 1, fp);
                b[num] = '\0';
                ccp = b;
                while (num > 1) {
                    if (strstr(ccp, FAM_NAME_LC)) {
                        if (verbose > 2)
                            fprintf(stderr, "'%s' found in compatible "
                                    "string: %s\n", FAM_NAME, ccp);
                        ret = 0;
                        break;
                    }
                    n = strlen(ccp) + 1;
                    num -= n;
                    ccp += n;
                }
                if (verbose && ferror(fp))
                    fprintf(stderr, "Failed to read: %s\n", DEVTREE_COMPAT);
                fclose(fp);
            } else if (verbose)
                fprintf(stderr, "Failed to open: %s\n", DEVTREE_COMPAT);
        }
    } else if (verbose)
        fprintf(stderr, "Failed to stat: %s\n", DEVTREE_MODEL);
#endif
    if (verbose)
        fprintf(stderr, "'%s' string %sfound in %s or %s\nso assume this "
                "is %sa %s family SoC\n", FAM_NAME, ret ? "not " : "",
                DEVTREE_MODEL, DEVTREE_COMPAT, ret ? "not " : "", FAM_NAME);
    if (print_stdout)
        printf("%d\n", ret);
    return ret;
}
