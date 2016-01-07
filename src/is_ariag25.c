/*
 * Copyright (c) 2010-2015 Douglas Gilbert.
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

/* This utility sets an exit status of 0 when __ARM_EABI__ defined and 'G25'
 * found of Hardware line of the output of /proc/cpuinfo otherwise
 * yields 1.
 *
 */

#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>


static const char * version_str = "0.94 20150317";

#define CPUINFO "/proc/cpuinfo"
#define DEVTREE_MODEL "/proc/device-tree/model"

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "is_ariag25 [-h] [-p] [-v] [-V]\n"
            "  where:\n"
            "    -h           print usage message\n"
            "    -p           prints '0' to stdout if Aria G25 else prints "
            "'1'\n"
            "    -v           increase verbosity\n"
            "    -V           print version string then exit\n\n"
            "Check %s to see if 'G25' on Hardware line or the device-tree\n"
            "model line. If so assume this is a Aria G25 and set an exit "
            "status of\n0 (true for scripts). Otherwise set an exit status "
            "of 1. When '-p'\noption given also send the same value to "
            "stdout.\n", CPUINFO);
}


int
main(int argc, char *argv[])
{
    int opt;
    int print_stdout = 0;
    int ret = 1;
#ifdef __ARM_EABI__
    int num;
    FILE *fp;
    char b[1024];
    char * cp;
    char * c2p;
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
    if ((fp = fopen(CPUINFO, "r"))) {
        num = fread(b, 1, sizeof(b) - 1, fp);
        if (num > 10) {
            b[num] = '\0';
            if ((cp = strstr(b, "\nHardware"))) {
                cp += 9;
                if ((c2p = strchr(cp, '\n'))) {
                    *c2p = '\0';
                    if (verbose > 2)
                        fprintf(stderr, "Checking this line: %s\n", cp);
                    if (strstr(cp, "G25"))
                        ret = 0;
                }
            }
            if (0 != ret) {
                if (verbose > 2)
                    fprintf(stderr, "Didn't find 'G25' in %s, now check %s "
                            "file\n", CPUINFO, DEVTREE_MODEL);
                fclose(fp);
                if ((fp = fopen(DEVTREE_MODEL, "r"))) {
                    num = fread(b, 1, sizeof(b) - 1, fp);
                    if (num > 2) {
                        b[num] = '\0';
                        if (strstr(b, "G25")) {
                            if (verbose > 2)
                                fprintf(stderr, "'G25' found in model line: "
                                        "%s\n", b);
                            ret = 0;
                        }
                    } else if (verbose && ferror(fp))
                        fprintf(stderr, "Failed to read: %s\n",
                                DEVTREE_MODEL);
                } else if (verbose)
                    fprintf(stderr, "Failed to open: %s\n", DEVTREE_MODEL);
            }
        } else if (verbose && ferror(fp))
            fprintf(stderr, "Failed to read: %s\n", CPUINFO);
        fclose(fp);
    } else if (verbose)
        fprintf(stderr, "Failed to open: %s\n", CPUINFO);
#endif
    if (verbose)
        fprintf(stderr, "'G25' string %sfound in %s or %s\nso assume this "
                "is %sa Aria G25\n", ret ? "not " : "", CPUINFO,
                DEVTREE_MODEL, ret ? "not " : "");
    if (print_stdout)
        printf("%d\n", ret);
    return ret;
}
