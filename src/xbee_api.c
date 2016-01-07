/*
 * Copyright (c) 2014-2015 Douglas Gilbert.
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


/*
 * This utility aims to test Xbee devices using their API interface via
 * a serial port. This utility reads ACSII hex pairs from an input
 * file (or stdin) then sends the corresponding binary to the given <tty>.
 * Then, optionally, it reads the response from <tty> and prints it in
 * ASCII hex on stdout
 */

#define _GNU_SOURCE
#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>


static const char * version_str = "1.01 20150205";

#define DEF_BAUD_RATE B9600
#define DEF_BAUD_RATE_STR "9600"
#define DEF_NON_CANONICAL_TIMEOUT 20     /* unit: 100ms so 20-> 2 seconds */

static struct termios tty_saved_attribs;
static int tty_saved_fd = -1;
static int timeout_100ms = DEF_NON_CANONICAL_TIMEOUT;
static int verbose = 0;
static int warn = 0;
static int xopen = 0;


static void
usage(void)
{
    fprintf(stderr, "Usage: "
            "xbee_api [-a] [-b <baud>] [-B <nbits>] [-c] [-D] [-F] [-h]\n"
            "                [-H <hex_file>] [-i <hex_file>] [-n] [-N] "
            "[-P N|E|O]\n"
            "                [-r <num>] [-R] [-S <sbits>] [-T <secs[,rep]>] "
            "[-v] [-V]\n"
            "                [-w] [-x] <tty>\n"
            "  where:\n"
            "    -a           with '-r <num>' show bytes in ASCII as well\n"
            "    -b <baud>    baud rate of <tty> (default: %s)\n"
            "    -B <nbits>    number of data bits: 5, 6, 7 or 8 (default)\n"
            "    -c           hardware handshake (RTS+CTS); use twice "
            "to disable\n"
            "    -D           set DTR, use twice to clear DTR (need '-n' "
            "and '-x'\n"
            "                 to keep level after this utility completes)\n"
            "    -F           no flush (def: flush input+output after <tty> "
            "open)\n"
            "    -h           print usage message\n"
            "    -H <hex_file>    file containing ASCII hex to send to "
            "<tty>\n"
            "                     (def: read from stdin)\n"
            "    -i <hex_file>    same as '-H <hex_file>'\n"
            "    -n           no HUPCL (stop RTS+DTR being cleared on "
            "close)\n"
            "                 use twice: set HUPCL (Hang UP on CLose)\n"
            "    -N           send nothing. Useful with '-r <num>' or '-x'\n"
            "    -P N|E|O     parity: N->none (default), E->even, O->odd\n"
            "    -r <num>     read <num> bytes from <tty>, print in ASCII "
            "hex on\n"
            "                 stdout. Unless -N or -x given, read is after "
            "send\n"
            "    -R           set RTS, use twice to clear RTS (may need "
            "'-n -x')\n"
            "    -S <sbits>   number of stop bits, 1 (default) or 2\n"
            "    -T <secs[,rep]>    <secs> timeout on reads, <rep> repeats "
            "(def:\n"
            "                       2,0; max <secs> is 25 seconds)\n"
            "                       if <secs>=0 then poll() with 1 second "
            "timeout\n"
            "    -v           increase verbosity (more written to stderr)\n"
            "    -V           print version string then exit\n"
            "    -w           warn about hardware RTS/CTS handshake with "
            "clear CTS\n"
            "    -x           will not restore previous settings on exit; "
            "if used\n"
            "                 only once will not send nor read\n\n"
            "Decode ASCII hex in <hex_file> or from stdin, then prefix with "
            "xbee API\nlead-in and length plus append checksum, then send "
            "packet to <tty>.\n"
            "The hex can be in two digit pairs, single digit hex needs to "
            "be separated\nby whitespace or commas. Hex can appear on "
            "multiple lines, anything after\n"
            "a '#' on a line is regarded as a comment. Restores previous "
            "<tty> settings\n(unless '-x' is given one or more times). "
            "Default framing is 8-N-1 .\n"
            "Examples:\n"
            "  send AT command ND; read up to 200 chars for up to 60 "
            "seconds:\n"
            "    echo 8 1 4e 44 | xbee_api -a -b 9600 -r 200 -T 0,60 -w "
            "/dev/ttyS1\n"
            "  leave settings on <tty> after exit:\n"
            "    xbee_api -b 38400 -c -n -x /dev/ttyS1\n",
            DEF_BAUD_RATE_STR);
}

static char *
serr(void)
{
    return strerror(errno);
}

static void
termination_handler(int signum)
{
    // tcsetattr(STDIN_FILENO, TCSANOW, &stdin_saved_attributes);
    if (tty_saved_fd >= 0) {
        if (xopen) {
            if (verbose > 1)
                fprintf(stderr, "keeping new <tty> settings due to '-x' "
                        "option [signum=%d]\n", signum);
        } else {
            if (verbose > 1)
                fprintf(stderr, "restoring <tty> settings to previous "
                        "settings [signum=%d]\n", signum);
            tcsetattr(tty_saved_fd, TCSANOW, &tty_saved_attribs);
        }
        /* close() might hang (bad driver code) so skip */
        /* close(tty_saved_fd); */
        /* tty_saved_fd = -1; */
    }
    fprintf(stderr, "Termination signal causes exit\n");
    signal(signum, SIG_DFL);
    kill(getpid(), signum);     /* propagate signal */
    /* exit(0); */
}

static bool
poll_in_for(int fd, int millisecs)
{
    int num;
    struct pollfd apfd;

    if (fd >= 0) {
        apfd.fd = fd;
        apfd.events = POLLIN;
        apfd.revents = 0;
        num = poll(&apfd, 1, millisecs);
        return (num > 0) && (POLLIN & apfd.revents);
    }
    return false;
}

static int
xb_tty_open(const char * tty_dev, int tty_speed, int dtr, int rts,
            int hhandshake, int no_hupcl, int nbits, int parit, int sbits)
{
    int tty_fd, res, mask, mbits;
    struct termios new_attributes;

    // tty_fd = open(tty_dev, (O_RDWR | O_NOCTTY | O_NONBLOCK));
    if (verbose > 2)
        fprintf(stderr, "%s: about to open(%s)\n", __func__, tty_dev);
    tty_fd = open(tty_dev, (O_RDWR | O_NOCTTY | O_SYNC));
    if (tty_fd < 0) {
        fprintf(stderr, "%s: open() of %s failed: %s\n", __func__,
                tty_dev, strerror(errno));
        return -1;
    }

    tcgetattr(tty_fd, &tty_saved_attribs);
    new_attributes = tty_saved_attribs;

    // Set the new attributes for the serial port, 'man termios'
    cfsetospeed(&new_attributes, tty_speed);
    cfsetispeed(&new_attributes, tty_speed);

    // c_cflag
    new_attributes.c_cflag |= CREAD;        // Enable receiver
    new_attributes.c_cflag &= ~CSIZE;       // clear size mask
    switch (nbits) {
    case 5:
        new_attributes.c_cflag |= CS5;
        break;
    case 6:
        new_attributes.c_cflag |= CS6;
        break;
    case 7:
        new_attributes.c_cflag |= CS7;
        break;
    case 8:
    default:
        new_attributes.c_cflag |= CS8;          // 8 data bit
        break;
    }
    switch (parit) {
    case 'E':
        new_attributes.c_cflag |= PARENB;
        new_attributes.c_cflag &= ~PARODD;
        break;
    case 'O':
        new_attributes.c_cflag |= PARENB;
        new_attributes.c_cflag |= PARODD;
        break;
    case 'N':
        new_attributes.c_cflag &= ~PARENB;
        break;
    }
    if (1 == sbits)
        new_attributes.c_cflag &= ~CSTOPB;
    else
        new_attributes.c_cflag |= CSTOPB;
    if (no_hupcl) {     // clear Hang Up on CLose (effects DTR+RTS)
        if (1 == no_hupcl) {
            new_attributes.c_cflag &= ~HUPCL;
            if (verbose)
                fprintf(stderr, "clearing HUPCL so RTS+DTR keep setting "
                        "after close\n");
        } else {
            new_attributes.c_cflag |= HUPCL;
            if (verbose)
                fprintf(stderr, "setting HUPCL so RTS+DTR go inactive "
                        "after close\n");
        }
    }
    if (hhandshake) {
        if (1 == hhandshake) {
            new_attributes.c_cflag |= CRTSCTS;
            if (verbose)
                fprintf(stderr, "set hardware RTS/CTS handshake; those "
                        "lines should be wired\n");
        } else {
            new_attributes.c_cflag &= ~CRTSCTS;
            if (verbose)
                fprintf(stderr, "clear hardware RTS/CTS handshake\n");
        }
    }

    // c_iflag
    if ('N' == parit)
        new_attributes.c_iflag |= IGNPAR;       // Ignore framing and parity
    // Suggested by Michael Kerrisk [The Linux Programming Interface].
    // He also included "| PARMRK".
    new_attributes.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR |
                                INPCK | ISTRIP | IXON);

    // c_oflag     [Turn off corrupting output post-processing]
    new_attributes.c_oflag &= ~(OPOST);

    // c_lflag
    new_attributes.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ISIG);

    // Next two only apply for non-canonical reads (which we have set)
    new_attributes.c_cc[VMIN] = 0;     // Min chars to read
    new_attributes.c_cc[VTIME] = timeout_100ms; // units: 100 ms, max 25.5 secs

    res = tcsetattr(tty_fd, TCSANOW, &new_attributes);
    if (res < 0) {
        fprintf(stderr, "%s: tcsetattr() failed: %s\n", __func__,
                strerror(errno));
        close(tty_fd);
        tty_fd = -1;
    }
    if (verbose > 1) {
        if (ioctl(tty_fd, TIOCMGET, &mbits) >= 0)
            fprintf(stderr, "modem lines set: %s%s%s%s [0x%x]\n",
                    ((mbits & TIOCM_DSR) ? "DSR," : ""),
                    ((mbits & TIOCM_RNG) ? "RING," : ""),
                    ((mbits & TIOCM_CAR) ? "DCD," : ""),
                    ((mbits & TIOCM_CTS) ? "CTS," : ""),
                    mbits);
    }
    if (dtr) {
        mask = TIOCM_DTR;
        if (1 == dtr)
            ioctl(tty_fd, TIOCMBIS, &mask);
        else
            ioctl(tty_fd, TIOCMBIC, &mask);
    }
    if (rts) {
        mask = TIOCM_RTS;
        if (1 == rts)
            ioctl(tty_fd, TIOCMBIS, &mask);
        else
            ioctl(tty_fd, TIOCMBIC, &mask);
    }
    if ((verbose || warn) && (tty_saved_attribs.c_cflag & CRTSCTS) &&
        (0 == hhandshake)) {
        int cts_clear = 0;

        if ((-1 != mbits) || (ioctl(tty_fd, TIOCMGET, &mbits) >= 0)) {
            cts_clear = !(mbits & TIOCM_CTS);
            fprintf(stderr, ">>> hardware RTS/CTS handshake active, not "
                    "being changed\n>>> and CTS line is %s\n",
                    cts_clear ? "clear (low), this could cause lockup" :
                    "set (high), might be okay");
            if (cts_clear)
                fprintf(stderr, ">>> could use '-cc' to disable RTS/CTS "
                        "handshake\n");
        } else
            fprintf(stderr, ">>> hardware RTS/CTS handshake active, not "
                    "being changed\n");
    }
    if ((tty_fd >= 0) && (verbose > 3)) {
        char b[128];

        snprintf(b, sizeof(b) - 1, "stty -a -F %s", tty_dev);
        printf(">>> Output from this command line invocation: '%s' is:\n", b);
        res = system(b);
        if (WIFSIGNALED(res) &&
            (WTERMSIG(res) == SIGINT || WTERMSIG(res) == SIGQUIT))
            raise(WTERMSIG(res));
        /* ignore res of not a signal */
    }
    return tty_fd;
}

/* print bytes in ASCII-hex, optionally with leading address and/or
 * trailing ASCII. 'addr_ascii' allows for 4 output types:
 *     > 0     each line has address then up to 16 ASCII-hex bytes
 *     = 0     in addition, the bytes are listed in ASCII to the right
 *     = -1    only the ASCII-hex bytes are listed (i.e. without address)
 *     < -1    ASCII-hex bytes with ASCII to right (and without address) */
static void
dStrHex(const char * str, int len, int addr_ascii)
{
    const char* p = str;
    unsigned char c;
    char buff[82];
    int a = 0;
    const int bpstart = 5;
    const int cpstart = 60;
    int cpos = cpstart;
    int bpos = bpstart;
    int i, k;

    if (len <= 0)
        return;
    memset(buff, ' ', 80);
    buff[80] = '\0';
    if (-1 == addr_ascii) {
        for (k = 0; k < len; k++) {
            c = *p++;
            bpos += 3;
            if (bpos == (bpstart + (9 * 3)))
                bpos++;
            sprintf(&buff[bpos], "%.2x", (int)(unsigned char)c);
            buff[bpos + 2] = ' ';
            if ((k > 0) && (0 == ((k + 1) % 16))) {
                printf("%.60s\n", buff);
                bpos = bpstart;
                memset(buff, ' ', 80);
            }
        }
        if (bpos > bpstart)
            printf("%.60s\n", buff);
        return;
    }
    /* addr_ascii >= 0, start each line with address (offset) */
    if (addr_ascii >= 0) {
        k = sprintf(buff + 1, "%.2x", a);
        buff[k + 1] = ' ';
    }

    for (i = 0; i < len; i++) {
        c = *p++;
        bpos += 3;
        if (bpos == (bpstart + (9 * 3)))
            bpos++;
        sprintf(&buff[bpos], "%.2x", (int)(unsigned char)c);
        buff[bpos + 2] = ' ';
        if (addr_ascii > 0)
            buff[cpos++] = ' ';
        else {
            if ((c < ' ') || (c >= 0x7f))
                c = '.';
            buff[cpos++] = c;
        }
        if (cpos > (cpstart + 15)) {
            printf("%.76s\n", buff);
            bpos = bpstart;
            cpos = cpstart;
            a += 16;
            memset(buff, ' ', 80);
            if (addr_ascii >= 0) {
                k = sprintf(buff + 1, "%.2x", a);
                buff[k + 1] = ' ';
            }
        }
    }
    if (cpos > cpstart)
        printf("%.76s\n", buff);
}


int
main(int argc, char *argv[])
{
    int opt, baud, num, k, n, from;
    int ooff = 0;
    int and_ascii = 0;
    int tty_speed = DEF_BAUD_RATE;
    int hhandshake = 0;
    int dtr = 0;
    int no_flush = 0;
    int no_hupcl = 0;
    int num_bits = 8;
    int no_send = 0;
    int parity = 'N';
    int repeat = 0;
    int rts = 0;
    int stop_bits = 1;
    int to_read = 0;
    char c1, c2;
    uint8_t t;
    unsigned char bny[2048];
    char hex[2048];
    char *cp;
    FILE * fp = NULL;
    const char * tty_dev = NULL;
    const char * hex_file = NULL;

    memset(bny, 0, sizeof(bny));
    memset(hex, 0, sizeof(bny));
    while ((opt = getopt(argc, argv, "ab:B:cDFhH:i:nNP:r:RS:T:vVwx")) !=
           -1) {
        switch (opt) {
        case 'a':
            ++and_ascii;
            break;
        case 'b':
            baud = 0;
            baud = atoi(optarg);
            switch (baud) {
            case 300: tty_speed = B300; break;
            case 1200: tty_speed = B1200; break;
            case 2400: tty_speed = B2400; break;
            case 4800: tty_speed = B4800; break;
            case 9600: tty_speed = B9600; break;
            case 19200: tty_speed = B19200; break;
            case 38400: tty_speed = B38400; break;
            case 57600: tty_speed = B57600; break;
            case 115200: tty_speed = B115200; break;
            case 230400: tty_speed = B230400; break;
            default:
                fprintf(stderr, "Allowable rates: 300, 1200, 2400, 4800, "
                        "9600, 19200, 38400, 57600\n115200 or 230400\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'B':
            k = atoi(optarg);
            if ((k < 5) || ( k > 8)) {
                fprintf(stderr, "<nbits> should be 5, 6, 7 or 8\n");
                exit(EXIT_FAILURE);
            }
            num_bits = k;
            break;
        case 'c':
            ++hhandshake;
            break;
        case 'D':
            ++dtr;
            break;
        case 'F':
            ++no_flush;
            break;
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        case 'H':
            hex_file = optarg;
            break;
        case 'i':
            hex_file = optarg;
            break;
        case 'n':
            ++no_hupcl;
            break;
        case 'N':
            ++no_send;
            break;
        case 'P':
            switch((parity = toupper(optarg[0]))) {
            case 'N':
            case 'E':
            case 'O':
                break;
            default:
                fprintf(stderr, "expect '-P' argument to be 'N', 'E' or "
                        "'O'\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'r':
            k = atoi(optarg);
            if ((k < 0) || ( k > (int)sizeof(bny))) {
                fprintf(stderr, "<num> to read cannot exceed %d or be "
                        "negative\n", (int)sizeof(bny));
                exit(EXIT_FAILURE);
            }
            to_read = k;
            break;
        case 'R':
            ++rts;
            break;
        case 'S':
            k = atoi(optarg);
            if ((k < 1) || ( k > 2)) {
                fprintf(stderr, "<sbits> should be 1 or 2\n");
                exit(EXIT_FAILURE);
            }
            stop_bits = k;
            break;
        case 'T':
            k = atoi(optarg);
            if ((k < 0) || ( k > 25)) {
                fprintf(stderr, "<secs> timeout should be 0 to 25 second\n");
                exit(EXIT_FAILURE);
            }
            timeout_100ms = k * 10;
            cp = strchr(optarg, ',');
            if (cp) {
                k = atoi(cp + 1);
                if ((k < 0) || ( k > 9999)) {
                    fprintf(stderr, "<rep> should be 0 to 9999 second\n");
                    exit(EXIT_FAILURE);
                }
                repeat = k;
            }
            break;
        case 'v':
            ++verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
        case 'w':
            ++warn;
            break;
        case 'x':
            ++xopen;
            break;
        default: /* '?' */
            usage();
            exit(EXIT_FAILURE);
        }
    }
    if (optind < argc) {
        if (NULL == tty_dev) {
            tty_dev = argv[optind];
            ++optind;
        }
        if (optind < argc) {
            for (; optind < argc; ++optind)
                fprintf(stderr, "Unexpected extra argument: %s\n",
                        argv[optind]);
            usage();
            exit(EXIT_FAILURE);
        }
    }
    if (NULL == tty_dev) {
        fprintf(stderr, "missing <tty> argument\n");
        usage();
        exit(EXIT_FAILURE);
    }

    if (signal(SIGINT, termination_handler) == SIG_IGN)
        signal(SIGINT, SIG_IGN);    /* handler was SIG_IGN, so reinstate */
    if (signal(SIGHUP, termination_handler) == SIG_IGN)
        signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN)
        signal(SIGTERM, SIG_IGN);

    if ((1 == xopen) || no_send)
        goto bypass_input_read;
    else if (hex_file) {
        if ((fp = fopen(hex_file, "r")) == NULL) {
            fprintf(stderr, "fopen on %s failed with %s\n", hex_file,
                    strerror(errno));
            exit(EXIT_FAILURE);
        }
        num = fread(hex, 1, sizeof(hex) - 1, fp);
        if (num <= 0) {
            fprintf(stderr, "<hex_file> %s empty or some other problem\n",
                    hex_file);
            return EXIT_SUCCESS;
        }
    } else {    // read from stdin
        if (verbose > 2)
            fprintf(stderr, "about to read from stdin ...\n");
        num = fread(hex, 1, sizeof(hex) - 1, stdin);
        if (num <= 0) {
            fprintf(stderr, "nothing read on stdin\n");
            return EXIT_SUCCESS;
        }
    }
    hex[num - 1] = '\0';
    if (verbose > 1)
        fprintf(stderr, "read %d bytes from input\n", num);

    for (ooff = 3, cp = hex; (cp < (hex + num)) && *cp; ) {
        c1 = *cp++;
        if ('#' == c1) {
            cp = strchr(cp, '\n');
            if(NULL == cp)
                break;
             continue;
        }
        if ((isspace(c1)) || (',' == c1)) {
            continue;
        }
        if (isxdigit(c1)) {
            if (c1 < 'A')
                k = c1 - '0';
            else if (c1 < 'a')
                k = (c1 - 'A') + 10;
            else
                k = (c1 - 'a') + 10;
            c2 = *cp;
            if (isxdigit(c2)) {
                bny[ooff] = k << 4;
                ++cp;
                if (c2 < 'A')
                    k = c2 - '0';
                else if (c2 < 'a')
                    k = (c2 - 'A') + 10;
                else
                    k = (c2 - 'a') + 10;
                bny[ooff++] |= k;
            } else
                bny[ooff++] = k;
        } else {
            fprintf(stderr, "bad syntax starting at %.*s\n", 8, cp);
            break;
        }
    }
    if (fp)
        fclose(fp);
    if (ooff > 3) {
        bny[0] = 0x7e;
        n = ooff - 3;
        bny[1] = (n >> 8) & 0xff;
        bny[2] = n & 0xff;
        for (k = 0, t = 0; k < n; ++k)
            t += bny[3 + k];
        bny[ooff++] = 0xff - t;
    } else
        ooff = 0;       /* don't sent degenerate packet */
    if (verbose > 1) {
        if (ooff <= 0)
            fprintf(stderr, "NO ASCII hex bytes decoded\n");
        else {
            fprintf(stderr, "decoded %d bytes of ASCII hex with xbee wrap:\n",
                    ooff);
            for (k = 0; k < ooff; ++k) {
                if ((k > 0) && (0 == (k % 16)))
                    printf("\n");
                fprintf(stderr, " %02x", bny[k]);
            }
            fprintf(stderr, "\n");
        }
    }

bypass_input_read:
    if ((tty_saved_fd = xb_tty_open(tty_dev, tty_speed, dtr, rts, hhandshake,
                                    no_hupcl, num_bits, parity, stop_bits))
        < 0)
        exit(EXIT_FAILURE);
    else if (verbose)
        fprintf(stderr, "opened <tty> %s without problems\n", tty_dev);
    if (1 == xopen)
        goto the_end;

    if (! no_flush) {
        if (tcflush(tty_saved_fd, TCIOFLUSH) < 0) {
            fprintf(stderr, "tcflush(TCIOFLUSH) on %s failed: %s\n", tty_dev,
                    serr());
            exit(EXIT_FAILURE);
        } else if (verbose > 1)
            fprintf(stderr, "flushed <tty> without problems\n");
    }

    if (ooff > 0) {
        num = write(tty_saved_fd, bny, ooff);
        if (num < 0)
            fprintf(stderr, "write() to <tty> failed: %s\n", serr());
        if (verbose)
            fprintf(stderr, "wrote %d bytes to <tty>\n", ooff);
    }
    if (to_read > 0) {
        if (verbose)
            fprintf(stderr, "About to read %d bytes from <tty>\n", to_read);
        for (k = 0, from = 0; k < to_read; k += num) {
            num = 0;
            if (timeout_100ms > 0)
                num = read(tty_saved_fd, bny + k, to_read - k);
            else if (poll_in_for(tty_saved_fd, 1000 /* millisecond */))
                num = read(tty_saved_fd, bny + k, to_read - k);
            if ((verbose > 3) && (num > 0))
                fprintf(stderr, "read() got %d byte%s\n", num,
                        (num > 1) ? "s" : "");
            if (num <= 0) {
                if (num < 0)
                    break;
                if (repeat > 0) {
                    --repeat;
                    if (k > from) {
                        dStrHex((const char *)bny + from , k - from,
                                (and_ascii ? -2 : -1));
                        from = k;
                    }
                    continue;
                }
                break;
            }
        }
        if (num < 0)
            fprintf(stderr, "read() from <tty> failed: %s, exit\n", serr());
        if (k > from)
            dStrHex((const char *)bny + from, k - from,
                    (and_ascii ? -2 : -1));
        if (verbose)
            fprintf(stderr, "read() fetched %d byte%s\n", k,
                    (1 == k) ? "" : "s");
    }

the_end:
    if (tty_saved_fd >= 0) {
        if (0 == xopen) {
            if (verbose > 1)
                fprintf(stderr, "restoring <tty> settings to previous "
                        "state\n");
            tcsetattr(tty_saved_fd, TCSANOW, &tty_saved_attribs);
        } else if (verbose > 1)
            fprintf(stderr, "leaving raw <tty> settings in place\n");
        close(tty_saved_fd);
        tty_saved_fd = -1;
    }
    return EXIT_SUCCESS;
}
