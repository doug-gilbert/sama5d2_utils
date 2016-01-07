#
# Regular cron jobs for the sama5d2-utils package
#
0 4	* * *	root	[ -x /usr/bin/sama5d2-utils_maintenance ] && /usr/bin/sama5d2-utils_maintenance
