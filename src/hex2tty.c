/*
 * Copyright (c) 2010-2016 Douglas Gilbert.
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
 * This utility aims to test devices with a serial interface that use a
 * command response type protocol. The serial interface is one derived
 * from RS232 or V24. This utility reads ACSII hex pairs from an input
 * file (or stdin) then sends the corresponding binary to the given <tty>.
 * Then, optionally, it reads the response from <tty> and prints it in
 * ASCII hex on stdout
 */

#define _XOPEN_SOURCE 500
#define _GNU_SOURCE 1

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

#include <linux/serial.h>       /* for RS485 */


static const char * version_str = "1.10 20160121";

#define DEF_BAUD_RATE B38400
#define DEF_BAUD_RATE_STR "38400"
#define DEF_NON_CANONICAL_TIMEOUT 20     /* unit: 100ms so 20-> 2 seconds */

#ifndef TIOCGRS485
#define TIOCGRS485 0x542e
#endif
#ifndef TIOCSRS485
#define TIOCSRS485 0x542f
#endif

#define RS485_MS_NOT_GIVEN -1001

static struct termios tty_saved_attribs;
static int tty_saved_fd = -1;
static int timeout_100ms = DEF_NON_CANONICAL_TIMEOUT;
static int verbose = 0;
static int warn = 0;
static int xopen = 0;

#ifndef SER_RS485_RX_DURING_TX
struct my_serial_rs485 {
        uint32_t   flags;                  /* RS485 feature flags */
#define SER_RS485_RX_DURING_TX          (1 << 4)
        uint32_t   delay_rts_before_send;  /* Delay before send (millisecs) */
        uint32_t   delay_rts_after_send;   /* Delay after send (millisecs) */
        uint32_t   padding[5];             /* Memory is cheap, new structs
                                              are a royal PITA .. */
};
#define MY_SERIAL_RS485
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
usage(void)
{
    pr2serr("Usage: hex2tty [-a] [-b <baud>] [-B <nbits>] [-c] [-d] [-D] "
            "[-F] [-h]\n"
            "               [-H <hex_file>] [-i <hex_file>] [-n] [-N] "
            "[-P N|E|O] [-q]\n"
            "               [-r <num>] [-R] [-S <sbits>] [-T <secs[,rep]>] "
            "[-v] [-V]\n"
            "               [-w] [-x] [-y <rs485_ms>] <tty>\n"
            "  where:\n"
            "    -a           with '-r <num>' show bytes in ASCII as well\n"
            "    -b <baud>    baud rate of <tty> (default: %s)\n"
            "    -B <nbits>    number of data bits: 5, 6, 7 or 8 (default)\n"
            "    -c           hardware handshake (RTS+CTS); use twice "
            "to disable\n"
            "    -d           parse <hexfile> or stdin as ASCII decimal "
            "(def:\n"
            "                 parse as ASCII hex)\n"
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
            "    -q           open <tty>, query control lines then exit\n"
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
            "                 only once will not send nor read\n"
            "    -y <rs485_ms>    RS485 RTS trailing delay (millisecs). "
            "Enable\n"
            "                     RS485 when <rs485_ms> >= 0, disable when "
            "= -1\n\n"
            "Send bytes, decoded from ASCII hex in <hex_file> or stdin, to "
            "<tty>.\n"
            "The hex can be in two digit pairs, single digit hex needs to "
            "be separated\nby whitespace or commas. Hex can appear on "
            "multiple lines, anything after\n"
            "a '#' on a line is regarded as a comment. Restores previous "
            "<tty> settings\n(unless '-x' is given one or more times). "
            "Default framing is 8-N-1 .\n"
            "Examples:\n"
            "  send AT\\r, then read: 'echo 41 54 D | hex2tty -b 9600 "
            "-r 6 -w /dev/ttyS1'\n"
            "  leave settings after exit: 'hex2tty -b 38400 -c -n -x "
            "/dev/ttyS1'\n"
            "  send nothing, read <tty>: 'hex2tty -b 9600 -n -N -r 20 -T "
            "10 /dev/ttyS1'\n",
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
                pr2serr("keeping new <tty> settings due to '-x' option "
                        "[signum=%d]\n", signum);
        } else {
            if (verbose > 1)
                pr2serr("restoring <tty> settings to previous settings "
                        "[signum=%d]\n", signum);
            tcsetattr(tty_saved_fd, TCSANOW, &tty_saved_attribs);
        }
        /* close() might hang (bad driver code) so skip */
        /* close(tty_saved_fd); */
        /* tty_saved_fd = -1; */
    }
    pr2serr("Termination signal causes exit\n");
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
tty_query(const char * tty_dev)
{
    int tty_fd, m;

    if (verbose) {
        pr2serr("For TTL voltage levels (say 3.3 volts) 'Active' is low "
                "(near 0 volts)\n");
        pr2serr("while 'Inactive' is high (near 3.3 volts).\n");
        pr2serr("For TxD and RxD the idle state is inactive (MARK) thus "
                "high;\n");
        pr2serr("For V24 voltage levels (+- 13 volts) 'Active' is +13 "
                "volts.\n");
        pr2serr("Grey area is between -3 and +3 volts [V24]; 0.6 and 1.5 "
                "volts [TTL]\n");
    }
    if (verbose > 2)
        pr2serr("%s: about to open(%s)\n", __func__, tty_dev);
    tty_fd = open(tty_dev, (O_RDWR | O_NOCTTY | O_SYNC));
    if (tty_fd < 0) {
        pr2serr("%s: open() of %s failed: %s\n", __func__, tty_dev, serr());
        return -1;
    }
    if (ioctl(tty_fd, TIOCMGET, &m) >= 0) {
        pr2serr("modem lines settings from other end (DCE ?)\n");
        pr2serr("  CTS: %sctive\n", (m & TIOCM_CTS) ? "A" : "Ina");
        pr2serr("  DSR: %sctive\n", (m & TIOCM_DSR) ? "A" : "Ina");
        pr2serr("  DCD: %sctive\n", (m & TIOCM_CAR) ? "A" : "Ina");
        pr2serr("  RING: %sctive\n", (m & TIOCM_RNG) ? "A" : "Ina");
        pr2serr("modem lines set by this end (DTE)\n");
        pr2serr("  RTS: %sctive\n", (m & TIOCM_RTS) ? "A" : "Ina");
        pr2serr("  DTR: %sctive\n", (m & TIOCM_DTR) ? "A" : "Ina");
    } else {
        pr2serr("%s: ioctl(TIOCMGET) of %s failed: %s\n", __func__, tty_dev,
                serr());
        close(tty_fd);
        return -1;
    }
    close(tty_fd);
    return 0;
}

static int
tty_open(const char * tty_dev, int tty_speed, int dtr, int rts,
         int hhandshake, int no_hupcl, int nbits, int parit, int sbits,
         int rs485_ms)
{
    int tty_fd, res, mask, mbits;
    struct termios new_attributes;

    // tty_fd = open(tty_dev, (O_RDWR | O_NOCTTY | O_NONBLOCK));
    if (verbose > 2)
        pr2serr("%s: about to open(%s)\n", __func__, tty_dev);
    tty_fd = open(tty_dev, (O_RDWR | O_NOCTTY | O_SYNC));
    if (tty_fd < 0) {
        pr2serr("%s: open() of %s failed: %s\n", __func__, tty_dev, serr());
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
                pr2serr("clearing HUPCL so RTS+DTR keep setting after "
                        "close\n");
        } else {
            new_attributes.c_cflag |= HUPCL;
            if (verbose)
                pr2serr("setting HUPCL so RTS+DTR go inactive after close\n");
        }
    }
    if (hhandshake) {
        if (1 == hhandshake) {
            new_attributes.c_cflag |= CRTSCTS;
            if (verbose)
                pr2serr("set hardware RTS/CTS handshake; those lines should "
                        "be wired\n");
        } else {
            new_attributes.c_cflag &= ~CRTSCTS;
            if (verbose)
                pr2serr("clear hardware RTS/CTS handshake\n");
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
        pr2serr("%s: tcsetattr() failed: %s\n", __func__, serr());
        close(tty_fd);
        tty_fd = -1;
    }
    mbits = -1;
    if (verbose > 1) {
        if (ioctl(tty_fd, TIOCMGET, &mbits) >= 0)
            pr2serr("modem lines set: %s%s%s%s [0x%x]\n",
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
        if (verbose > 1)
            pr2serr("%s: %sing RTS line\n", __func__,
                    (1 == rts) ? "sett" : "clear");
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
            pr2serr(">>> hardware RTS/CTS handshake active, not being "
                    "changed\n>>> and CTS line is %s\n",
                    cts_clear ? "clear (low), this could cause lockup" :
                    "set (high), might be okay");
            if (cts_clear)
                pr2serr(">>> could use '-cc' to disable RTS/CTS handshake\n");
        } else
            pr2serr(">>> hardware RTS/CTS handshake active, not being "
                    "changed\n");
    }

    if (rs485_ms != RS485_MS_NOT_GIVEN) {
#ifdef SER_RS485_ENABLED
#ifdef MY_SERIAL_RS485
        struct my_serial_rs485 rs485conf;
#else
        struct serial_rs485 rs485conf;
#endif

        memset(&rs485conf, 0, sizeof(rs485conf));
        if (rs485_ms >= 0) {
            rs485conf.flags |= SER_RS485_ENABLED;
            if (verbose)
                pr2serr("RS485 enable with delay_rts_after_send=%d\n",
                        rs485_ms);
            if (rs485_ms > 0)
                rs485conf.delay_rts_after_send = rs485_ms;    // milliseconds
        } else if (verbose)
            pr2serr("disabling RS485; return to RS232 mode\n");
        if (ioctl(tty_fd, TIOCSRS485, &rs485conf) < 0) {
            pr2serr("%s: ioctl(TIOCSRS485) failed: %s\n", __func__, serr());
            close(tty_fd);
            tty_fd = -1;
        }
#endif
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
    int opt, baud, num, k, from;
    int ooff = 0;
    const char * tty_dev = NULL;
    const char * hex_file = NULL;
    int rs485_ms = RS485_MS_NOT_GIVEN;
    int and_ascii = 0;
    int as_decimal = 0;
    int tty_speed = DEF_BAUD_RATE;
    int hhandshake = 0;
    int dtr_num = 0;
    int no_flush = 0;
    int no_hupcl = 0;
    int num_bits = 8;
    int no_send = 0;
    int parity = 'N';
    int query = 0;
    int repeat = 0;
    int rts_num = 0;
    int stop_bits = 1;
    int to_read = 0;
    unsigned char bny[2048];
    char hex[2048];
    char *cp;
    char c1, c2, c3;
    FILE * fp = NULL;

    while ((opt = getopt(argc, argv, "ab:B:cdDFhH:i:nNP:qr:RS:T:vVwxy:")) !=
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
                pr2serr("Allowable rates: 300, 1200, 2400, 4800, 9600, "
                        "19200, 38400, 57600\n115200 or 230400\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'B':
            k = atoi(optarg);
            if ((k < 5) || ( k > 8)) {
                pr2serr("<nbits> should be 5, 6, 7 or 8\n");
                exit(EXIT_FAILURE);
            }
            num_bits = k;
            break;
        case 'c':
            ++hhandshake;
            break;
        case 'd':
            ++as_decimal;
            break;
        case 'D':
            ++dtr_num;
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
                pr2serr("expect '-P' argument to be 'N', 'E' or 'O'\n");
                exit(EXIT_FAILURE);
            }
            break;
        case 'q':
            ++query;
            break;
        case 'r':
            k = atoi(optarg);
            if ((k < 0) || ( k > (int)sizeof(bny))) {
                pr2serr("<num> to read cannot exceed %d or be negative\n",
                        (int)sizeof(bny));
                exit(EXIT_FAILURE);
            }
            to_read = k;
            break;
        case 'R':
            ++rts_num;
            break;
        case 'S':
            k = atoi(optarg);
            if ((k < 1) || ( k > 2)) {
                pr2serr("<sbits> should be 1 or 2\n");
                exit(EXIT_FAILURE);
            }
            stop_bits = k;
            break;
        case 'T':
            k = atoi(optarg);
            if ((k < 0) || ( k > 25)) {
                pr2serr("<secs> timeout should be 0 to 25 second\n");
                exit(EXIT_FAILURE);
            }
            timeout_100ms = k * 10;
            cp = strchr(optarg, ',');
            if (cp) {
                k = atoi(cp + 1);
                if ((k < 0) || ( k > 9999)) {
                    pr2serr("<rep> should be 0 to 9999 second\n");
                    exit(EXIT_FAILURE);
                }
                repeat = k;
            }
            break;
        case 'v':
            ++verbose;
            break;
        case 'w':
            ++warn;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
        case 'x':
            ++xopen;
            break;
        case 'y':
            if ('-' == optarg[0])
                rs485_ms = -1;
            else if (isdigit(optarg[0])) {
                k = atoi(optarg);
                if (k > 100 * 1000) {
                    pr2serr("<rs485_ms> exceeds arbitrary be 100 second "
                            "maximum\n");
                    exit(EXIT_FAILURE);
                }
                rs485_ms = k;
            } else {
                pr2serr("<rs485_ms> should be a number (option '-y')\n");
                exit(EXIT_FAILURE);
            }
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
                pr2serr("Unexpected extra argument: %s\n", argv[optind]);
            usage();
            exit(EXIT_FAILURE);
        }
    }
    if (NULL == tty_dev) {
        pr2serr("missing <tty> argument\n");
        usage();
        exit(EXIT_FAILURE);
    }

    if (rs485_ms != RS485_MS_NOT_GIVEN) {
        if (rts_num) {
            pr2serr("Can't specify RS485 and set (or clear) RTS\n");
            exit(EXIT_FAILURE);
        }
#ifndef SER_RS485_ENABLED
        pr2serr("Seems that this Linux kernel doesn't support RS485\n");
        exit(EXIT_FAILURE);
#endif
    }

    if (signal(SIGINT, termination_handler) == SIG_IGN)
        signal(SIGINT, SIG_IGN);    /* handler was SIG_IGN, so reinstate */
    if (signal(SIGHUP, termination_handler) == SIG_IGN)
        signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN)
        signal(SIGTERM, SIG_IGN);

    if ((1 == xopen) || no_send || query)
        goto bypass_input_read;
    else if (hex_file) {
        if ((fp = fopen(hex_file, "r")) == NULL) {
            pr2serr("fopen on %s failed with %s\n", hex_file, serr());
            exit(EXIT_FAILURE);
        }
        num = fread(hex, 1, sizeof(hex) - 1, fp);
        if (num <= 0) {
            pr2serr("<hex_file> %s empty or some other problem\n", hex_file);
            return EXIT_SUCCESS;
        }
    } else {    // read from stdin
        if (verbose > 2)
            pr2serr("about to read from stdin ...\n");
        num = fread(hex, 1, sizeof(hex) - 1, stdin);
        if (num <= 0) {
            pr2serr("nothing read on stdin\n");
            return EXIT_SUCCESS;
        }
    }
    hex[num - 1] = '\0';
    if (verbose > 1)
        pr2serr("read %d bytes from input\n", num);

    for (ooff = 0, cp = hex; (cp < (hex + num)) && *cp; ) {
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
        if (as_decimal) {
            k = 0;
            if (isdigit(c1)) {
                k = (c1 - '0');
                c2 = *cp;
                if (isdigit(c2)) {
                    ++cp;
                    k *= 10;
                    k += (c2 - '0');
                    c3 = *cp;
                    if (isdigit(c3)) {
                        ++cp;
                        k *= 10;
                        k += (c3 - '0');
                    }
                }
            } else {
                pr2serr("bad syntax starting near %.*s\n", 8, cp - 1);
                break;
            }
            if ((k < 0) || (k > 255)) {
                pr2serr("decimals need to be from 0 to 255 inclusive, "
                        "starting near %.*s\n", 8, cp - 1);
                break;
            }
            bny[ooff++] = k;
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
            pr2serr("bad syntax starting at %.*s\n", 8, cp);
            break;
        }
    }
    if (fp)
        fclose(fp);
    if (verbose > 1) {
        if (ooff <= 0)
            pr2serr("NO ASCII %s bytes decoded\n",
                    (as_decimal ? "decimal" : "hex"));
        else {
            pr2serr("decoded %d bytes of ASCII %s:\n", ooff,
                    (as_decimal ? "decimal" : "hex"));
            for (k = 0; k < ooff; ++k) {
                if ((k > 0) && (0 == (k % 16)))
                    printf("\n");
                pr2serr(" %02x", bny[k]);
            }
            pr2serr("\n");
        }
    }

bypass_input_read:
    if (query) {
        k = tty_query(tty_dev);
        return k ? 1 : 0;
    }
    if ((tty_saved_fd = tty_open(tty_dev, tty_speed, dtr_num, rts_num,
                                 hhandshake, no_hupcl, num_bits, parity,
                                 stop_bits, rs485_ms)) < 0)
        exit(EXIT_FAILURE);
    else if (verbose)
        pr2serr("opened <tty> %s without problems\n", tty_dev);
    if (1 == xopen)
        goto the_end;

    if (! no_flush) {
        if (tcflush(tty_saved_fd, TCIOFLUSH) < 0) {
            pr2serr("tcflush(TCIOFLUSH) on %s failed: %s\n", tty_dev, serr());
            exit(EXIT_FAILURE);
        } else if (verbose > 1)
            pr2serr("flushed <tty> without problems\n");
    }

    if (ooff > 0) {
        num = write(tty_saved_fd, bny, ooff);
        if (num < 0)
            pr2serr("write() to <tty> failed: %s\n", serr());
        if (verbose)
            pr2serr("wrote %d bytes to <tty>\n", ooff);
    }
    if (to_read > 0) {
        if (verbose)
            pr2serr("About to read %d bytes from <tty>\n", to_read);
        for (k = 0, from = 0; k < to_read; k += num) {
            num = 0;
            if (timeout_100ms > 0)
                num = read(tty_saved_fd, bny + k, to_read - k);
            else if (poll_in_for(tty_saved_fd, 1000 /* millisecond */))
                num = read(tty_saved_fd, bny + k, to_read - k);
            if ((verbose > 3) && (num > 0))
                pr2serr("read() got %d byte%s\n", num,
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
            pr2serr("read() from <tty> failed: %s, exit\n", serr());
        if (k > from)
            dStrHex((const char *)bny + from, k - from,
                    (and_ascii ? -2 : -1));
        if (verbose)
            pr2serr("read() fetched %d byte%s\n", k, (1 == k) ? "" : "s");
    }

the_end:
    if (tty_saved_fd >= 0) {
        if (0 == xopen) {
            if (verbose > 1)
                pr2serr("restoring <tty> settings to previous state\n");
            tcsetattr(tty_saved_fd, TCSANOW, &tty_saved_attribs);
        } else if (verbose > 1)
            pr2serr("leaving raw <tty> settings in place\n");
        close(tty_saved_fd);
        tty_saved_fd = -1;
    }
    return EXIT_SUCCESS;
}
