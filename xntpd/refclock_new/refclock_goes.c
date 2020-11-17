/*
 * refclock_goes - clock driver for the Kinemetrics Truetime GOES
 *	Receiver Version 2.0
 */

#if defined(REFCLOCK) && defined(GOES)

#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_refclock.h"
#include "ntp_unixtime.h"
#include "ntp_stdlib.h"

/*
 * Support for Kinemetrics Truetime 468-DC GOES Receiver
 *
 * Most of this code is copied from refclock_goes.c with thanks.
 *
 * The time code looks like follows; Send the clock a 'R' or 'C' and
 * once per second a timestamp will appear that looks like this:
 *
 * ADDD:HH:MM:SSQCL
 * A - control A
 * Q Quality indication: indicates possible error of
 *     ?     +/- 500 milliseconds	#     +/- 50 milliseconds
 *     *     +/- 5 milliseconds		.     +/- 1 millisecond
 *   space   less than 1 millisecond
 * C - Carriage return
 * L - Line feed
 *
 * The carriage return start bit begins on 0 seconds and extends to 1
 * bit time. Unless you live on 125 degrees west longitude, you can't
 * set your clock propagation delay settings correctly and still use
 * automatic mode. The manual says to use a compromise when setting the
 * switches. This results in significant errors. The solution; use fudge
 * time1 and time2 to incorporate corrections. If your clock is set for
 * 50 and it should be 58 for using the west and 46 for using the east,
 * use the line
 *
 * fudge 127.127.5.0 time1 +0.008 time2 -0.004
 *
 * This corrects the 4 milliseconds advance and 5 milliseconds retard
 * needed. The software will ask the clock which satellite it sees.
 *
 * Flag1 set to 1 will silence the clock side of xntpd, just reading the
 * clock without trying to write to it.  This is usefull if several
 * xntpds listen to the same clock. This has not been tested yet...
 */

/*
 * Definitions
 */
#define	MAXUNITS	4	/* max number of GOES units */
#define	GOES232	"/dev/goes%d"
#define	SPEED232	B9600	/* 9600 baud */

/*
 * Radio interface parameters
 */
#define	GOESMAXDISPERSE	(FP_SECOND>>1) /* max error for synchronized clock (0.5 s as an u_fp) */
#define	GOESSKEWFACTOR	17	/* skew factor (for about 32 ppm) */
#define	GOESPRECISION	(-10)	/* precision assumed (about 1 ms) */
#define	GOESREFID	"GOES"	/* reference id */
#define	GOESDESCRIPTION	"TrueTime GPS/GOES Receivers" /* WRU */
#define GMT		0	/* hour offset from Greenwich */
#define	NCODES		3	/* stages of median filter */
#define	LENGOES0	13	/* format 0 timecode length */
#define	LENGOES2	21	/* format 2 timecode length */
#define FMTGOESU	0	/* unknown format timecode id */
#define FMTGOES0	1	/* format 0 timecode id */
#define FMTGOES2	2	/* format 2 timecode id */
#define	DEFFUDGETIME	0	/* default fudge time (ms) */
#define BMAX		50	/* timecode buffer length */
#define	CODEDIFF	0x20000000	/* 0.125 seconds as an l_fp fraction */

/*
 * Tag which satellite we see
 */
#define GOES_SAT_NONE   0
#define GOES_SAT_WEST   1
#define GOES_SAT_EAST   2
#define GOES_SAT_STAND  3

/*
 * Hack to avoid excercising the multiplier.  I have no pride.
 */
#define	MULBY10(x)	(((x)<<3) + ((x)<<1))

/*
 * Imported from the timer module
 */
extern u_long current_time;
extern struct event timerqueue[];

/*
 * Imported from ntp_loopfilter module
 */
extern int fdpps;		/* pps file descriptor */

/*
 * Imported from ntpd module
 */
extern int debug;		/* global debug flag */

/*
 * GOES unit control structure
 */
struct goesunit {
	struct peer *peer;		/* associated peer structure */
	struct refclockio io;		/* given to the I/O handler */
	l_fp lastrec;			/* last receive time */
	l_fp lastref;			/* last timecode time */
	l_fp offset[NCODES];		/* recent sample offsets */
	char lastcode[BMAX];		/* last timecode received */
	u_short satellite;		/* which satellite we saw */
	u_short polled;			/* Hand in a time sample? */
	u_char format;			/* timecode format */
	u_char lencode;			/* length of last timecode */
	u_long lasttime;		/* last time clock heard from */
	u_char unit;			/* unit number for this guy */
	u_char status;			/* clock status */
	u_char lastevent;		/* last clock event */
	u_char reason;			/* reason for last abort */
	u_char year;			/* year of eternity */
	u_short day;			/* day of year */
	u_char hour;			/* hour of day */
	u_char minute;			/* minute of hour */
	u_char second;			/* seconds of minute */
	u_char leap;			/* leap indicators */
	u_short msec;			/* millisecond of second */
	u_char quality;			/* quality char from format 2 */
	u_long yearstart;		/* start of current year */
	/*
	 * Status tallies
 	 */
	u_long polls;			/* polls sent */
	u_long noreply;			/* no replies to polls */
	u_long coderecv;		/* timecodes received */
	u_long badformat;		/* bad format */
	u_long baddata;			/* bad data */
	u_long timestarted;		/* time we started this */
};

/*
 * Function prototypes
 */
static	int	goes_start	P((int, struct peer *));
static	void	goes_shutdown	P((int, struct peer *));
static	void	goes_receive	P((struct recvbuf *));
static	char	goes_process	P((struct goesunit *, l_fp *, u_fp *));
static	void	goes_poll	P((int, struct peer *));
static	void	goes_send	P((struct goesunit *, char *));

/*
 * Transfer vector
 */
struct	refclock refclock_goes = {
	goes_start,		/* start up driver */
	goes_shutdown,		/* shut down driver */
	goes_poll,		/* transmit poll message */
	noentry,		/* not used (old goes_control) */
	noentry,		/* initialize driver (not used) */
	noentry,		/* not used (old goes_buginfo) */
	NOFLAGS			/* not used */
};


/*
 * goes_start - open the devices and initialize data for processing
 */
static int
goes_start(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct goesunit *up;
	struct refclockproc *pp;
	int fd;
	char device[20];

	/*
	 * Open serial port
	 */
	(void)sprintf(device, GOES232, unit);
	if (!(fd = refclock_open(device, SPEED232, 0)))
		return (0);

	/*
	 * Allocate and initialize unit structure
	 */
	if (!(up = (struct goesunit *)
	    emalloc(sizeof(struct goesunit)))) {
		(void) close(fd);
		return (0);
	}
	memset((char *)up, 0, sizeof(struct goesunit));
	up->io.clock_recv = goes_receive;
	up->io.srcclock = (caddr_t)up;
	up->io.datalen = 0;
	up->io.fd = fd;
	if (!io_addclock(&up->io)) {
		(void) close(fd);
		free(up);
		return (0);
	}
	up->peer = peer;
	pp = peer->procptr;
	pp->unitptr = (caddr_t)up;

	/*
	 * Initialize miscellaneous variables
	 */
	peer->precision = GOESPRECISION;
	pp->clockdesc = GOESDESCRIPTION;
	memcpy((char *)&pp->refid, GOESREFID, 4);
	return (1);
}


/*
 * goes_shutdown - shut down the clock
 */
static void
goes_shutdown(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct goesunit *up;
	struct refclockproc *pp;

	pp = peer->procptr;
	up = (struct goesunit *)pp->unitptr;
	io_closeclock(&up->io);
	free(up);
}


/*
 * goes_receive - receive data from the serial interface on a
 * Kinimetrics clock
 */
static void
goes_receive(rbufp)
	struct recvbuf *rbufp;
{
	register int i;
	register struct goesunit *up;
	struct refclockproc *pp;
	struct peer *peer;
	register u_char *dpt;
	register char *cp;
	register u_char *dpend;
	l_fp tstmp;
	u_fp dispersion;

	/*
	 * Get the clock this applies to and a pointers to the data
	 */
	up = (struct goesunit *)rbufp->recv_srcclock;
	peer = up->peer;
	pp = peer->procptr;
	dpt = (u_char *)&rbufp->recv_space;

	/*
	 * Edit timecode to remove control chars
	 */
	dpend = dpt + rbufp->recv_length;
	cp = up->lastcode;
	while (dpt < dpend) {
		if ((*cp = 0x7f & *dpt++) >= ' ') cp++; 
#ifdef GOESCLK
		else if (*cp == '\r') {
			if (dpend - dpt < 8) {
				/* short timestamp */
				return;
			}
			if (!buftvtots(dpt,&up->lastrec)) {
				/* screwy timestamp */
				return;
			}
			dpt += 8;
		}
#endif
	}
	*cp = '\0';
	up->lencode = cp - up->lastcode;
	if (up->lencode == 0) return;
#ifndef GOESCLK
	up->lastrec = rbufp->recv_time;
#endif /* GOESCLK */
	tstmp = up->lastrec;

#ifdef DEBUG
	if (debug)
        	printf("goes: timecode %d %s\n",
		    up->lencode, up->lastcode);
#endif

	/*
	 * We get down to business, check the timecode format and decode
	 * its contents. This code checks for and decodes both format 0
	 * and format 2 and need not be told which in advance.
	 */
	cp = up->lastcode;
	up->leap = 0;
	up->format = FMTGOESU;
	if (up->lencode == LENGOES0) {

		/*
	 	 * Check timecode format 0
	 	 */
		if (!isdigit(cp[0]) ||	/* day of year */
			!isdigit(cp[1]) ||
			!isdigit(cp[2]) ||
			cp[3] != ':' ||		/* <sp> */
			!isdigit(cp[4]) ||	/* hours */
			!isdigit(cp[5]) ||
			cp[6] != ':' ||		/* : separator */
			!isdigit(cp[7]) ||	/* minutes */
			!isdigit(cp[8]) ||
			cp[9] != ':' ||		/* : separator */
			!isdigit(cp[10]) ||	/* seconds */
			!isdigit(cp[11])) {
				refclock_report(peer, CEVNT_BADREPLY);
				return;
			}
		else up->format = FMTGOES0;

		/*
		 * Convert format 0 and check values 
		 */
		up->year = 0;		/* fake */
		up->day = cp[0] - '0';
		up->day = MULBY10(up->day) + cp[1] - '0';
		up->day = MULBY10(up->day) + cp[2] - '0';
		up->hour = MULBY10(cp[4] - '0') + cp[5] - '0';
		up->minute = MULBY10(cp[7] - '0') + cp[8] -  '0';
		up->second = MULBY10(cp[10] - '0') + cp[11] - '0';
		up->msec = 0;

		if (cp[12] != ' ' && cp[12] != '.' && cp[12] != '*')
			up->leap = LEAP_NOTINSYNC;
		else
			up->lasttime = current_time;

		if (up->day < 1 || up->day > 366) {
			refclock_report(peer, CEVNT_BADDATE);
			return;
		}
		if (up->hour > 23 || up->minute > 59
		    || up->second > 59) {
			refclock_report(peer, CEVNT_BADTIME);
			return;
		}

	} else if (up->lencode == LENGOES2) {

		/*
		 * Extended precision satelite location info
		 */
		if (!isdigit(cp[0]) ||		/* longitude */
			!isdigit(cp[1]) ||
			!isdigit(cp[2]) ||
			cp[3] != '.' ||
			!isdigit(cp[4]) ||
			!isdigit(cp[5]) ||
			!isdigit(cp[6]) ||
			!isdigit(cp[7]) ||
			(cp[8] != '+' && cp[8] != '-') ||
			!isdigit(cp[9]) ||	/*latitude */
			cp[10] != '.' ||
			!isdigit(cp[11]) ||
			!isdigit(cp[12]) ||
			!isdigit(cp[13]) ||
			!isdigit(cp[14]) ||
			(cp[15] != '+' && cp[15] != '-') ||
			!isdigit(cp[16]) ||	/* height */
			!isdigit(cp[17]) ||
			!isdigit(cp[18]) ||
			cp[19] != '.' ||
			!isdigit(cp[20])) {
				refclock_report(peer, CEVNT_BADREPLY);
				return;
			}
		else up->format = FMTGOES2;

		/*
		 * Figure out which satellite this is.
		 * This allows +-5 degrees from nominal.
		 */
		if (cp[0] == '1' && (cp[1] == '3' || cp[1] == '2'))
			up->satellite = GOES_SAT_WEST;
		else if (cp[0] == '1' && cp[1] == '0')
			up->satellite = GOES_SAT_STAND;
		else if (cp[0] == '0' && cp[1] == '7')
			up->satellite = GOES_SAT_EAST;
		else
			up->satellite = GOES_SAT_NONE;

#ifdef DEBUG
		if (debug)
			printf("goes_receive: select satellite %d\n",
				up->satellite);
#endif

		/*
		 * Switch back to on-second time codes.
		 */
		goes_send(up, "C");

		/*
		 * Since this is not a time code, just return...
		 */
		return;
	} else {
		refclock_report(peer, CEVNT_BADREPLY);
		return;
	}

	/*
	 * The clock will blurt a timecode every second but we only
	 * want one when polled.  If we havn't been polled, bail out.
	 */
	if (!up->polled)
		return;

	/*
	 * Now, compute the reference time value. Use the heavy
	 * machinery for the seconds and the millisecond field for the
	 * fraction when present.
         *
	 * this code does not yet know how to do the years
	 */
	tstmp = up->lastrec;
	if (!clocktime(up->day, up->hour, up->minute,
	    up->second, GMT, tstmp.l_ui,
	    &up->yearstart, &up->lastref.l_ui)) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}
	MSUTOTSF(up->msec, up->lastref.l_uf);


	/*
	 * Slop the read value by fudgefactor1 or fudgefactor2 depending
	 * on which satellite we are viewing last time we checked.
	 */

#ifdef DEBUG
	if (debug)
		printf("GOES_RECEIVE: Slopping for satellite %d\n",
			up->satellite);
#endif
	if (up->satellite == GOES_SAT_WEST)
		L_ADD(&up->lastref, &pp->fudgetime1);
	else if (up->satellite == GOES_SAT_EAST)
		L_ADD(&up->lastref, &pp->fudgetime2);
/*	else if (up->satellite == GOES_SAT_STAND)
		L_ADD(&up->lastref, &((fudgefactor1[up->unit] +
			fudgefactor2[up->unit]) / 2)); */

	i = ((int)(up->coderecv)) % NCODES;
	up->offset[i] = up->lastref;
	L_SUB(&up->offset[i], &tstmp);
	if (up->coderecv == 0)
		for (i = 1; i < NCODES; i++)
			up->offset[i] = up->offset[0];

	up->coderecv++;

	/*
	 * Check the satellite position
	 */
	goes_send(up, "E");

	/*
	 * Process the median filter, add the fudge factor and pass the
	 * offset and dispersion along. We use lastrec as both the
	 * reference time and receive time in order to avoid being cute,
	 * like setting the reference time later than the receive time,
	 * which may cause a paranoid protocol module to chuck out the
	 * data.
 	 */
	if (!goes_process(up, &tstmp, &dispersion)) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}
	refclock_receive(up->peer, &tstmp, GMT, dispersion,
	    &up->lastrec, &up->lastrec, up->leap);

	/*
	 * We have succedded in answering the poll.  Turn off the flag
	 */
	up->polled = 0;
}


/*
 * goes_send - time to send the clock a signal to cough up a time sample
 */
static void
goes_send(up, cmd)
	struct goesunit *up;
	char *cmd;
{
	struct peer *peer;

	/*
	 * Send a command to the clock. C for on-second timecodes.
	 * E for extended resolution satelite postion information.
	 */
	peer = up->peer;
	if (write(up->io.fd, cmd, 1) != 1) {
		refclock_report(up->peer, CEVNT_FAULT);
	} else {
		up->polls++;
	}
}


/*
 * goes_process - process a pile of samples from the clock
 */
static char
goes_process(up, offset, dispersion)
	struct goesunit *up;
	l_fp *offset;
	u_fp *dispersion;
{
	register int i, j;
	register u_long tmp_ui, tmp_uf;
	int not_median1 = -1;	/* XXX correct? */
	int not_median2 = -1;	/* XXX correct? */
	int median;
	u_fp disp_tmp, disp_tmp2;

	/*
	 * This code implements a three-stage median filter. First, we
         * check if the samples are within 125 ms of each other. If not,
	 * dump the sample set. We take the median of the three offsets
	 * and use that as the sample offset. We take the maximum
	 * difference and use that as the sample dispersion. There
	 * probably is not much to be gained by a longer filter, since
	 * the clock filter in ntp_proto should do its thing.
	 */
	disp_tmp2 = 0;
	for (i = 0; i < NCODES-1; i++) {
		for (j = i+1; j < NCODES; j++) {
			tmp_ui = up->offset[i].l_ui;
			tmp_uf = up->offset[i].l_uf;
			M_SUB(tmp_ui, tmp_uf, up->offset[j].l_ui,
				up->offset[j].l_uf);
			if (M_ISNEG(tmp_ui, tmp_uf)) {
				M_NEG(tmp_ui, tmp_uf);
			}
			if (tmp_ui != 0 || tmp_uf > CODEDIFF) {
				return 0;
			}
			disp_tmp = MFPTOFP(0, tmp_uf);
			if (disp_tmp > disp_tmp2) {
				disp_tmp2 = disp_tmp;
				not_median1 = i;
				not_median2 = j;
			}
		}
	}

	/*
	 * It seems as if all are within 125 ms of each other.
	 * Now to determine the median of the three. Whlie the
	 * 125 ms check was going on, we also subtly catch the
	 * dispersion and set-up for a very easy median calculation.
	 * The largest difference between any two samples constitutes
	 * the dispersion. The sample not involve in the dispersion is
	 * the median sample. EASY!
	 */
	if (up->lasttime == 0 || disp_tmp2 > GOESMAXDISPERSE)
	    disp_tmp2 = GOESMAXDISPERSE;
	if (not_median1 == 0) {
		if (not_median2 == 1)
		    median = 2;
		else
		    median = 1;
        } else {
		median = 0;
        }
	*offset = up->offset[median];
	*dispersion = disp_tmp2;
	return 1;
}

/*
 * goes_poll - called by the transmit procedure
 */
static void
goes_poll(unit, peer)
	int unit;
	struct peer *peer;
{
	struct goesunit *up;
	struct refclockproc *pp;

	/*
	 * You don't need to poll this clock.  It puts out timecodes
	 * once per second.  If asked for a timestamp, take note.
	 * The next time a timecode comes in, it will be fed back.
	 */
	pp = peer->procptr;
	up = (struct goesunit *)pp->unitptr;
	if ((current_time - up->lasttime) > 150) {
		refclock_report(peer, CEVNT_TIMEOUT);
	}

	/*
	 * polled every 64 seconds. Ask GOES_RECEIVE to hand in a
	 * timestamp.
	 */
	up->polled = 1;
	up->polls++;

	goes_send(up, "C");
}

#endif
