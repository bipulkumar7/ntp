/*
 * refclock_acts - clock driver for the NIST Automated Computer Time
 *	Service (ACTS)
 */
#if defined(REFCLOCK) && defined(ACTS)

#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_refclock.h"
#include "ntp_stdlib.h"

/*
 * This driver supports the Almagamated Containerized Trash Service.
 * It periodically dials a prespecified telephone number, boogies with
 * the digits and calculates the local clock correction. This code
 * was lifted shamefully from refclock_acts.c and awaits beautification
 * by Judah Levine.
 *
 * Note there are two main data structures used by this code, the peer
 * structure, which is used by the prototol machinery, and the unit
 * structure, which is used by this code. The actsunits[] is an array of
 * pointers to the unit structures, while the peer member of that
 * structure points to the peer structure. While this may seem silly,
 * the generic clock drivers actually support multiple units of the same
 * type, so it is possible in principle to bring up a herd of ACTS
 * telephones, each maybe to a different country when ACTS goes
 * international. It's probably wise to keep this distraction of
 * abstraction intact in the inerest of uniformity and it doesn't cost
 * much.
 *
 * 1. Edit the acts_receive() routine to decode the modem line as
 * received. The driver code snatches a timestamp of each <cr> and
 * passes it in the receive buffer. If the optional CLK line discipline
 * is configured, the timestamp is captured as close to the tty driver
 * as possible and appears in the data stream. The code to figure out
 * which is the case and decode the best timestamp is at the beginning
 * of the acts_receive() routine.
 *
 * 2. Most clock drivers assemble the day, hour, minute and second from
 * the radio timecode as received. The clocktime() routine assembles
 * these data to form a NTP timestamp. However, some radios have no
 * provision for the year, so a fiddle is used where the clocktime()
 * routine is passed the seconds of the current unix timeval and the
 * routine figures from there. It probably won't hurt much to keep this
 * bit of drivel in the expectation it can be fixed someday.
 *
 * 3. Some radios provide the seconds fraction, which in this code is
 * added in separately by the MSUTOTSF macro. The result should be the
 * precise epoch represented by the timecode string. The difference
 * between that and the timestamp so carefully snatched earlier is the
 * local clock offset. In the driver here, these offsets are processed
 * by an ugly-hacked median filter. Other drivers use qsort to do that
 * in a beauteous way. The results of this dance are passed to the
 * refclock_receive() routine which does all the rest. Note the leap
 * bits, which are determined by the driver and passed along as well.
 *
 * 4. The acts_poll() routine is called at 64-s intervals, but this can
 * be changed using the minpoll parameter in the configuration file. Its
 * range is clamped elsewhere between 16 s and 1024 s. With suitable
 * scaling, this might be a useful way to bracket the calling intervals.
 * Most drivers use this routine to send messages to the radio; however,
 * some radios put the offset computations in this routine and call
 * refclock_receive() from there, instead of the acts_receive() routine.
 * No sweat either way.
 *
 * 5. The acts_control() and acts_buginfo() routines are called by the
 * configuration machinery and by xntpdc. This makes it convenient to
 * watch and reconfigure the machinery from afar, assuming the proper
 * authentication codes are handy. The acts_control() routine can stay
 * as it is, unless you want to pirate a time or flag function. Leave
 * the fudgevalues alone; they are used to set the stratum and reference
 * identifier in odd cases. The acts_buginfo() routine is used as a
 * debugging aid. The values returned can be smertched with abandon,
 * since nobody but xntpdc clkbug command ever see them.
 *
 * 6. The remaining routines can probably be left as-is. It is real hard
 * to watch modem signals with this thing and it's probably not a good
 * idea anyway, since the carrier-detect lead might have been hijacked
 * for the pps signal.
 *
 * TBD: Assuming you want to wiggle the clock frequency directly by
 * using your own hybrid loop, instead of the crufty NTP one, we may
 * need to provide a way to do that which works whether or not the
 * kernel mods are configured. The ntp_loopfilter.c module does the
 * actual work. It shouldn't be too hard to find a path to adjust the
 * frequency as well as the time and dissuade the kernel routines from
 * further mischief. Note the drift_comp global variable controls the
 * frequency when the kernel mods are not in use, but the variable is
 * not used if the mods are in use.
 */

/*
 * Interface definitions
 */
#define	DEVICE		"/dev/acts%d" /* device name and unit */
#define	SPEED232	B300	/* uart speed (300 cowardly baud) */
#define	PRECISION	(-13)	/* precision assumed (about 100 us) */
#define	REFID		"ACTS"	/* reference ID */
#define	DESCRIPTION	"NIST Automated Computer Time Service" /* WRU */

#define	NSAMPLES	3	/* stages of median filter */
#define	LENACTS0	22	/* format 0 timecode length */
#define	LENACTS2	24	/* format 2 timecode length */
#define MONLIN		15	/* number of monitoring lines */

/*
 * Imported from ntp_timer module
 */
extern	u_long	current_time;	/* current time (s) */

/*
 * Imported from ntpd module
 */
extern	int	debug;		/* global debug flag */

/*
 * Unit control structure
 */
struct actsunit {
	struct	refclockio io;	/* I/O handler structure */
	char	lastcode[BMAX];	/* last timecode received */
	u_char	lencode;	/* length of last timecode */
	int	pollcnt;	/* poll message counter */

	u_char	tcswitch;	/* timecode switch */
	l_fp	laststamp;	/* last receive timestamp */
	u_char	lasthour;	/* last hour (for monitor) */
	u_char	linect;		/* count ignored lines (for monitor */
};

/*
 * Function prototypes
 */
static	void	acts_init	P((void));
static	int	acts_start	P((int, struct peer *));
static	void	acts_shutdown	P((int, struct peer *));
static	void	acts_receive	P((struct recvbuf *));
static	void	acts_poll	P((int, struct peer *));

/*
 * Transfer vector
 */
struct	refclock refclock_acts = {
	acts_start,		/* start up driver */
	acts_shutdown,		/* shut down driver */
	acts_poll,		/* transmit poll message */
	noentry,		/* not used (old acts_control) */
	acts_init,		/* initialize driver */
	noentry,		/* not used (old acts_buginfo) */
	NOFLAGS			/* not used */
};


/*
 * acts_init - initialize driver - nothing to do here
 */
static void
acts_init()
{
}


/*
 * acts_start - open the devices and initialize data for processing
 */
static int
acts_start(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct actsunit *up;
	struct refclockproc *pp;
	int fd;
	char device[20];

	/*
	 * Open serial port. Use CLK line discipline, if available.
	 */
	(void)sprintf(device, DEVICE, unit);
	if (!(fd = refclock_open(device, SPEED232, LDISC_CLK)))
		return (0);

	/*
	 * Allocate and initialize unit structure
	 */
	if (!(up = (struct actsunit *)
	    emalloc(sizeof(struct actsunit)))) {
		(void) close(fd);
		return (0);
	}
	memset((char *)up, 0, sizeof(struct actsunit));
	pp = peer->procptr;
	pp->io.clock_recv = acts_receive;
	pp->io.srcclock = (caddr_t)peer;
	pp->io.datalen = 0;
	pp->io.fd = fd;
	if (!io_addclock(&pp->io)) {
		(void) close(fd);
		free(up);
		return (0);
	}
	pp->unitptr = (caddr_t)up;

	/*
	 * Initialize miscellaneous variables
	 */
	peer->precision = PRECISION;
	pp->clockdesc = DESCRIPTION;
	memcpy((char *)&pp->refid, REFID, 4);
	up->pollcnt = 2;
	return (1);
}


/*
 * acts_shutdown - shut down the clock
 */
static void
acts_shutdown(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct actsunit *up;
	struct refclockproc *pp;

	pp = peer->procptr;
	up = (struct actsunit *)pp->unitptr;
	io_closeclock(&pp->io);
	free(up);
}


/*
 * acts_receive - receive data from the serial interface
 */
static void
acts_receive(rbufp)
	struct recvbuf *rbufp;
{
	register struct actsunit *up;
	struct refclockproc *pp;
	struct peer *peer;
	l_fp trtmp;
	u_long ltemp;
	char syncchar;		/* synchronization indicator */
	char leapchar;		/* leap indicator */
	char qualchar;		/* quality indicator */

	/*
	 * Initialize pointers and read the timecode and timestamp
	 */
	peer = (struct peer *)rbufp->recv_srcclock;
	pp = peer->procptr;
	up = (struct actsunit *)pp->unitptr;
	pp->lencode = refclock_gtlin(rbufp, pp->lastcode, BMAX, &trtmp);

	/*
	 * Note we get a buffer and timestamp for both a <cr> and <lf>,
	 * but only the <cr> timestamp is retained. Note: in format 0 on
	 * a Netclock/2 or upgraded 8170 the start bit is delayed 100
	 * +-50 us relative to the pps; however, on an unmodified 8170
	 * the start bit can be delayed up to 10 ms. In format 2 the
	 * reading precision is only to the millisecond. Thus, unless
	 * you have a pps gadget and don't have to have the year, format
	 * 0 provides the lowest jitter.
	 */
	if (pp->lencode == 0) {
		if (up->tcswitch == 0) {
			up->tcswitch = 1;
			up->laststamp = trtmp;
		} else
			up->tcswitch = 0;
		return;
	}
	pp->lastrec = up->laststamp;
	up->laststamp = trtmp;
	up->tcswitch = 1;
	up->pollcnt = 2;
	record_clock_stats(&peer->srcadr, pp->lastcode);
#ifdef DEBUG
	if (debug)
        	printf("acts: timecode %d %s\n", pp->lencode,
		    pp->lastcode);
#endif

	/*
	 * We get down to business, check the timecode format and decode
	 * its contents. This code uses the timecode length to determine
	 * whether format 0 or format 2. If the timecode has invalid
	 * length or is not in proper format, we declare bad format and
	 * exit.
	 */
	switch (pp->lencode) {

		case LENACTS0:

		/*
	 	 * Timecode format 0: "I  ddd hh:mm:ss  TZ=nn"
	 	 */
		qualchar = leapchar = ' ';
		if (sscanf(pp->lastcode, "%c %3d %2d:%2d:%2d",
		    &syncchar, &pp->day, &pp->hour, &pp->minute,
		    &pp->second) == 5)
			break;

		case LENACTS2:

		/*
	 	 * Timecode format 2: "IQyy ddd hh:mm:ss.mmm LD"
	 	 */
		if (sscanf(pp->lastcode, "%c%c %2d %3d %2d:%2d:%2d.%3d %c",
		    &syncchar, &qualchar, &pp->year, &pp->day,
		    &pp->hour, &pp->minute, &pp->second, &pp->msec,
		    &leapchar) == 9)
			break;

		default:

		if (up->linect > 0)
			up->linect--;
		else
			refclock_report(peer, CEVNT_BADREPLY);
		return;
	}

	/*
	 * Decode synchronization, quality and leap characters. If
	 * unsynchronized, set the leap bits accordingly and exit.
	 * Otherwise, set the leap bits according to the leap character.
	 * Once synchronized, the dispersion depends only on when the
	 * clock was last heard. The first time the clock is heard, the
	 * time last heard is faked based on the quality indicator. The
	 * magic numbers (in seconds) are from the clock specifications.
	 */
	switch (qualchar) {

		case ' ':

		ltemp = 0;
		break;

		case 'A':

		ltemp = 800;
		break;

		case 'B':

		ltemp = 5300;
		break;

		case 'C':

		ltemp = 25300;
		break;

		case 'D':

		ltemp = NTP_MAXAGE;
		break;

		default:

		refclock_report(peer, CEVNT_BADREPLY);
		return;
	}
	if (syncchar != ' ')
		pp->leap = LEAP_NOTINSYNC;
	else {
		if (leapchar == 'L')
			pp->leap = LEAP_ADDSECOND;
		else
			pp->leap = 0;
		pp->lasttime = current_time - ltemp;
	}

	/*
	 * If the monitor flag is set (flag4), we dump the internal
	 * quality table at the first timecode beginning the day.
	 */
        if (pp->sloppyclockflag & CLK_FLAG4 && pp->hour <
	    up->lasthour)
		up->linect = MONLIN;
	up->lasthour = pp->hour;

	/*
	 * Process the new sample in the median filter and determine the
	 * reference clock offset and dispersion. We use lastrec as both
	 * the reference time and receive time in order to avoid being
	 * cute, like setting the reference time later than the receive
	 * time, which may cause a paranoid protocol module to chuck out
	 * the data.
 	 */
	if (!refclock_process(pp, NSAMPLES, NSAMPLES)) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}
	L_CLR(&trtmp);
	trtmp.l_ui = pp->lasttime;
	refclock_receive(peer, &pp->offset, 0, pp->dispersion,
	    &trtmp, &pp->lastrec, pp->leap);
}


/*
 * acts_poll - called by the transmit procedure
 */
static void
acts_poll(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct actsunit *up;
	struct refclockproc *pp;
	char poll;

	/*
	 * This routine is called at intervals determined by the minpoll
	 * configuration parameter on the server configuration line.
	 * Normally, this is at 64-s intervals.
	 */
	pp = peer->procptr;
	up = (struct actsunit *)pp->unitptr;
	if (up->pollcnt > 0) {
		up->pollcnt--;
		if (up->pollcnt == 0)
		    refclock_report(peer, CEVNT_TIMEOUT);
	}

	/*
	 * Insert your favorite message here; poll points to it. Of
	 * course, a message does not have to be sent at each call to
	 * this routine.
	 */
	if (write(pp->io.fd, &poll, 1) != 1) {
		refclock_report(peer, CEVNT_FAULT);
	} else {
		pp->polls++;
	}
}

#endif
