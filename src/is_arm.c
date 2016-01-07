/*
 * Copyright (c) 2010-2012 Douglas Gilbert.
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

/* This utility sets an exit status of 0 when __ARM_EABI__ defined otherwise
 * yields 1. So if __ARM_EABI__ defined assume hardware has ARM CPU.
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


static const char * version_str = "0.92 20121114";

static int verbose = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "is_arm [-h] [-p] [-v] [-V]\n"
            "  where:\n"
            "    -h           print usage message\n"
            "    -p           prints '0' to stdout if ARM cpu else prints "
            "'1'\n"
            "    -v           increase verbosity\n"
            "    -V           print version string then exit\n"
            "Check if compiler saw __ARM_EABI__ defined. If so assume this "
            "is a ARM and\nset an exit status of 0 (true for scripts). "
            "Otherwise set an exit status of 1.\nWhen '-p' option given "
            "also send the same value to stdout.\n");
}


int
main(int argc, char *argv[])
{
    int opt;
    int print_stdout = 0;
    int ret = 1;

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
    ret = 0;
#endif
    if (verbose)
        fprintf(stderr, "__ARM_EABI__ %sdefined so assume this has %san "
                "ARM CPU\n", ret ? "not " : "", ret ? "not " : "");
    if (print_stdout)
        printf("%d\n", ret);
    return ret;
}
