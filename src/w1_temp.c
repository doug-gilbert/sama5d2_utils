// #define _XOPEN_SOURCE 500

#define _XOPEN_SOURCE 600
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>


static const char * version_str = "0.98 20160121";

#define SYSFS_W1_DEVS "/sys/bus/w1/devices"
#define DS18S20_DS1820_FAM 0x10
#define DS18B20_FAM 0x28

struct opts_t {
    int both;
    int dev_fam;
    int family;
    int fixed_pnt;
    const char * ofile;
    const char * new_fn;
    int verbose;
    int serial_num;
    FILE * out_fp;
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
usage(void)
{
    pr2serr("Usage: w1_temp [-a <afile>] [-b] [-f] [-F] [-h] [-o <ofile>] "
            "[-r <new_fn>]\n"
            "               [-s] [-v] [-V] [-w <fcode>]\n"
            "  where:\n"
            "    -a <afile>   reads W1 addresses from <afile> then outputs "
            "temperature\n"
            "                 of corresponding device or '-' if not "
            "found\n"
            "    -b           check both 0x10 and 0x28 families\n"
            "    -f           fixed point, up to 3 decimal places (def: "
            "rounded integer)\n"
            "    -F           print family before serial number\n"
            "    -h           print usage message\n"
            "    -o <ofile>    send output to <ofile> rather than stdout\n"
            "    -r <new_fn>   unlink <new_fn> and rename <ofile> to "
            "<new_fn> just\n"
            "                  before exiting. Ignored unless '-o <ofile>' "
            "given\n"
            "    -s           print device serial number, not temperature\n"
            "    -v           increase verbosity\n"
            "    -V           print version string then exit\n"
            "    -w <fcode>    family code (hex if leading '0x'), or 'B' "
            "for 0x28\n"
            "                  or 'S' for 0x10. 'B' is for DS18B20 part, 'S' "
            "for DS18S20\n"
            "Fetch temperature from one wire (w1) device and write to "
            "<ofile> or stdout.\nUses Linux sysfs interface and assumes "
            "W1_SLAVE_THERM is configured in\nkernel. Default fcode is "
            "0x10 for the DS18S20 and DS1820.\n");
}

#if 0
/* for b != 0 . Returns non negative number */
static int
mod(int a, int b)
{
    int ret;

    if (b < 0)
        return mod(-a, -b);
    ret = a % b;
    if(ret < 0)
        ret += b;
    return ret;
}
#endif

static int
get_temp(const char * dev, const char * pathp, struct opts_t * op)
{
    int fd, milliTemp, mt, t;
    ssize_t numRead;
    const char * ccp;
    char * cp;
    char buf[256];     // Data from device
    char tmpData[5];   // Temp C * 1000 reported by device
    char devPath[128];

    if (op->verbose)
        pr2serr("found W1 temperature device: %s\n", dev);
    if (op->serial_num) {
        if ((ccp = strchr(dev, '-'))) {
            if (1 == sscanf(ccp + 1, "%14s", buf)) {
                if (op->out_fp)
                    ;
                else if (op->ofile) {
                    op->out_fp = fopen(op->ofile, "w");
                    if (NULL == op->out_fp) {
                        pr2serr("Unable to open %s\n", op->ofile);
                        return 1;
                    }
                } else
                    op->out_fp = stdout;
                if (op->family)
                    fprintf(op->out_fp, "%.20s\n", dev);
                else
                    fprintf(op->out_fp, "%s\n", buf);
            }
        }
        return 0;
    }
    snprintf(devPath, sizeof(devPath), "%s/%s/w1_slave", pathp, dev);
    fd = open(devPath, O_RDONLY);
    if(fd < 0) {
        perror("Couldn't open the w1 device");
        pr2serr("  [%s]\n", devPath);
        return 1;
    }
    numRead = read(fd, buf, 255);
    if (numRead < 0) {
        perror("failed reading W1 temperature device\n");
        return 1;
    } else if (0 == numRead) {
        pr2serr("read nothing from W1 temperature device\n");
        return 1;
    }
    buf[numRead] = '\0';
    close(fd);
    cp = strstr(buf, "t=");
    if (NULL == cp) {
        pr2serr("unable to find 't=' string\n");
        return 1;
    }
    strcpy(tmpData, cp + 2);
    cp = strchr(tmpData, '\n');
    if (cp)
        *cp = '\0';
    if (op->verbose)
        pr2serr("Raw temperature string: %s\n", tmpData);
    if (1 != sscanf(tmpData, "%d", &milliTemp)) {
        pr2serr("unable to decode temperature raw string\n");
        return 1;
    }
    if (op->verbose)
        pr2serr("temperature in C x1000: %d\n", milliTemp);

    if (op->out_fp)
        ;
    else if (op->ofile) {
        op->out_fp = fopen(op->ofile, "w");
        if (NULL == op->out_fp) {
            pr2serr("Unable to open %s\n", op->ofile);
            return 1;
        }
    } else
        op->out_fp = stdout;
    t = milliTemp / 1000;
    if (milliTemp < 0)
        mt = (- milliTemp) % 1000;
    else
        mt = milliTemp % 1000;
    if (op->fixed_pnt)
        fprintf(op->out_fp, "%d.%.03d\n", t, mt);
    if (1 != op->fixed_pnt) {
        if ((t >= 0) && (mt >= 500))
            ++t;
        else if ((t < 0) && (mt >= 500))
            --t;
        fprintf(op->out_fp, "%d\n", t);
    }
    return 0;
}

int
main(int argc, char *argv[])
{
    int opt, len, ch, k, n, found;
    DIR *dir;
    FILE * fip;
    struct dirent *dirent;
    char dt_str[8];
    char dev[20];
    char b[256];
    char x[256];
    char y[256];
    const char * afilep = NULL;
    char * cp;
    const char path[] = SYSFS_W1_DEVS;
    struct opts_t opts;
    struct opts_t * op;

    op = &opts;
    memset(op, 0, sizeof(opts));
    op->dev_fam = DS18S20_DS1820_FAM;        /* default */
    while ((opt = getopt(argc, argv, "a:bfFho:r:svVw:")) != -1) {
        switch (opt) {
        case 'a':
            if (afilep) {
                pr2serr("only expect a single '-a <afile>' option\n");
                exit(EXIT_FAILURE);
            }
            afilep = optarg;
            break;
        case 'b':
            ++op->both;
            break;
        case 'f':
            ++op->fixed_pnt;
            break;
        case 'F':
            ++op->family;
            break;
        case 'h':
            usage();
            exit(EXIT_SUCCESS);
        case 'o':
            op->ofile = optarg;
            break;
        case 'r':
            op->new_fn = optarg;
            break;
        case 's':
            ++op->serial_num;
            break;
        case 'v':
            ++op->verbose;
            break;
        case 'V':
            printf("%s\n", version_str);
            exit(EXIT_SUCCESS);
            break;
        case 'w':
            len = strlen(optarg);
            if (len > 0) {
                ch = optarg[0];
                if ('S' == toupper(ch))
                    op->dev_fam = DS18S20_DS1820_FAM;
                else if ('B' == toupper(ch))
                    op->dev_fam = DS18B20_FAM;
                else if (len > 2) {
                    if (('0' == ch) && 'X' == toupper(optarg[1])) {
                        k = sscanf(optarg + 2, "%x",
                                   (unsigned int *)&op->dev_fam);
                        if (1 != k) {
                            pr2serr("-w can't decode hex\n");
                            exit(EXIT_FAILURE);
                        }
                    } else if (isdigit(ch)) {
                        k = sscanf(optarg, "%d", &op->dev_fam);
                        if (1 != k) {
                            pr2serr("-w can't decode decimal\n");
                            exit(EXIT_FAILURE);
                        }
                    } else {
                        pr2serr("-w can't decode anything\n");
                        exit(EXIT_FAILURE);
                    }
                } else if (isdigit(ch)) {
                    k = sscanf(optarg, "%d", &op->dev_fam);
                    if (1 != k) {
                        pr2serr("-w can't decode decimal\n");
                        exit(EXIT_FAILURE);
                    }
                } else {
                    pr2serr("-w can't decode\n");
                    exit(EXIT_FAILURE);
                }
            } else {
                pr2serr("-w empty argument ??\n");
                exit(EXIT_FAILURE);
            }
            break;
        default: /* '?' */
            usage();
            exit(EXIT_FAILURE);
        }
    }
    if (optind < argc) {
        if (optind < argc) {
            for (; optind < argc; ++optind)
                pr2serr("Unexpected extra argument: %s\n",
                        argv[optind]);
            usage();
            exit(EXIT_FAILURE);
        }
    }

    if (afilep) {
        if (! ((fip = fopen(afilep, "r")))) {
            pr2serr("unable to open %s\n", afilep);
            exit(EXIT_FAILURE);
        }
        if (op->verbose)
            pr2serr("path: %s\n", path);
        for (k = 0; k < 32; ++k) {
            cp = fgets(b, sizeof(b), fip);
            if (NULL == cp)
                break;
            n = strlen(cp);
            if ('\n' == cp[n - 1]) {
                cp[n - 1] = '\0';
                --n;
            }
            if ('\r' == cp[n - 1]) {
                cp[n - 1] = '\0';
                --n;
            }
            if (op->verbose > 2)
                pr2serr("read line %d: %s\n", k + 1, cp);
            snprintf(x, sizeof(x), "%02x-%s", DS18S20_DS1820_FAM, cp);
            snprintf(y, sizeof(y), "%02x-%s", DS18B20_FAM, cp);

            dir = opendir(path);
            if (dir != NULL) {
                found = 0;
                while ((dirent = readdir(dir))) {
                    // W1 devices are links starting with 28- (S part) or 10-
                    if (dirent->d_type == DT_LNK) {
                        if ((op->both || (DS18S20_DS1820_FAM == op->dev_fam))
                            && (0 == strcmp(x, dirent->d_name))) {
                            if (op->verbose > 2)
                                pr2serr("Found matching link: %s\n", x);
                            if (get_temp(x, path, op))
                                goto exit_fail1;
                            ++found;
                            break;
                        }
                        if ((op->both || (DS18B20_FAM == op->dev_fam))
                            && (0 == strcmp(y, dirent->d_name))) {
                            if (op->verbose > 2)
                                pr2serr("Found matching link: %s\n", y);
                            if (get_temp(y, path, op))
                                goto exit_fail1;
                            ++found;
                            break;
                        }
                    }
                }
                (void) closedir(dir);
                if (! found) {
                    if (op->out_fp)
                        ;
                    else if (op->ofile) {
                        op->out_fp = fopen(op->ofile, "w");
                        if (NULL == op->out_fp) {
                            pr2serr("Unable to open %s\n", op->ofile);
                            goto exit_fail1;
                        }
                    } else
                        op->out_fp = stdout;
                    fprintf(op->out_fp, "-\n");
                }
            } else {
                perror("Couldn't open the w1 devices directory");
                pr2serr("  [%s]\n", path);
                goto exit_fail1;
            }
        }
        fclose(fip);
        if (op->out_fp && (stdout != op->out_fp)) {
            fclose(op->out_fp);
            if (op->new_fn) {
                unlink(op->new_fn);
                rename(op->ofile, op->new_fn);
            }
        }
        return 0;
exit_fail1:
        if (op->out_fp && (stdout != op->out_fp))
            fclose(op->out_fp);
        exit(EXIT_FAILURE);
    }

    for (k = 0; k < 2; ++k) {
        if (k > 0) {
            if (! op->both)
                break;
            if (DS18S20_DS1820_FAM == op->dev_fam)
                op->dev_fam = DS18B20_FAM;
            else
                op->dev_fam = DS18S20_DS1820_FAM;
        }
        snprintf(dt_str, sizeof(dt_str), "%02x-", op->dev_fam);
        if (op->verbose)
            pr2serr("dev_fam string: %s\npath: %s\n", dt_str, path);
        // 1st pass counts devices
        dir = opendir(path);
        if (dir != NULL) {
            found = 0;
            while ((dirent = readdir(dir))) {
                // W1 devices are links starting with 28- (S part) or 10-
                if (dirent->d_type == DT_LNK &&
                    strstr(dirent->d_name, dt_str) != NULL) {
                    strncpy(dev, dirent->d_name, sizeof(dev) - 1);
                    dev[sizeof(dev) - 1] = '\0';
                    ++found;
                    if (op->verbose > 2)
                        pr2serr("Found matching link: %s\n", dirent->d_name);
                    if (get_temp(dev, path, op))
                        exit(EXIT_FAILURE);
                } else if ((op->verbose > 2) && (dirent->d_type == DT_LNK))
                    pr2serr("Found non-matching link: %s\n", dirent->d_name);
            }
            (void) closedir(dir);
        } else {
            perror("Couldn't open the w1 devices directory");
            pr2serr("  [%s]\n", path);
            goto exit_fail;
        }
    }
    if (op->out_fp && (stdout != op->out_fp)) {
        fclose(op->out_fp);
        if (op->new_fn) {
            unlink(op->new_fn);
            rename(op->ofile, op->new_fn);
        }
    }
    if (! (found || op->both)) {
        pr2serr("No W1 device matching device family 0x%x found\n",
                op->dev_fam);
        goto exit_fail;
    }
    return 0;

exit_fail:
    if (op->out_fp && (stdout != op->out_fp))
        fclose(op->out_fp);
    exit(EXIT_FAILURE);
}
