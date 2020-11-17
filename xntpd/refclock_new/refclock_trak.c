/*
 * refclock_trak.c - clock driver for the TRAK 8810 GPS STATION CLOCK
 *		Tsuruoka Tomoaki Oct 30, 1993 
 *		tsuruoka@nc.fukuoka-u.ac.jp
 *		Faculty of Engineering,
 *		Fukuoka University, Fukuoka, JAPAN
 */
#if defined(REFCLOCK) && defined(TRAK)

#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_refclock.h"
#include "ntp_unixtime.h"

static void trak_send();

#include "ntp_stdlib.h"

/*
 * This driver supports the TRAK 8810 GPS Receiver with
 * Buffered RS-232-C Interface Module. 
 *
 * Most of codes are copied from refclock_as2201.c, Thanks a lot.
 *
 * The program expects the radio responses once per seccond
 * ( by "rqts,u" command or panel control )
 * of the form "*RQTS U,ddd:hh:mm:ss.0,Q\r\n for UTC" where 
 * ddd= day of year
 * hh=  hours
 * mm=  minutes
 * ss=  seconds
 * Q=   Quality byte. Q=0 Phase error > 20 us
 *		      Q=6 Phase error < 20 us
 *				      > 10 us
 *		      Q=5 Phase error < 10 us
 *				      > 1 us
 *		      Q=4 Phase error < 1 us
 *				      > 100 ns
 *		      Q=3 Phase error < 100 ns
 *				      > 10 ns
 *		      Q=2 Phase error < 10 ns
 *  (note that my clock almost stable at 1 us per 10 hours)
 *
 * Request leap second status - if needed.
 *	send:	rqls\n
 *	reply:	RQLS yy,mm,dd
 *	where:	yy is year
 *		mm is month
 *		dd is day of month.baud
 *	Note:	Default data is all zeros
 *		i.e. RQLS 00,00,00
 */

/*
 * Definitions
 */
#define	TRAK232	"/dev/gps%d"	/* name of radio device */
#define	SPEED232	B9600	/* uart speed (9600 bps) */

/*
 * Radio interface parameters
 */
#define	TRAKPRECISION	(-20)	/* precision assumed (about 1 us) */
#define	TRAKREFID	"TRAK"	/* reference id */
#define	TRAKDESCRIPTION	"TRAK 8810 GPS Receiver" /* who we are */
#define	TRAKHSREFID	0x7f7f020a /* 127.127.2.10 refid hi strata */
#define	NCODES		3	/* stages of median filter */
#define	LENTOC		25	/* *RQTS U,ddd:hh:mm:ss.0,Q datecode length */
#define BMAX		100	/* timecode buffer length */
#define	CODEDIFF	0x20000000	/* 0.125 seconds as an l_fp fraction */

/*
 * Hack to avoid excercising the multiplier.  I have no pride.
 */
#define	MULBY10(x)	(((x)<<3) + ((x)<<1))

/*
 * Imported from ntp_timer module
 */
extern u_long current_time;	/* current time (s) */

/*
 * Imported from ntpd module
 */
extern int debug;		/* global debug flag */

/*
 * GPS unit control structure.
 */
struct trakunit {
	struct	peer *peer;	/* associated peer structure */
	struct	refclockio io;	/* given to the I/O handler */
	struct	refclockproc proc; /* process structure */
	l_fp	lastrec;	/* last data receive time */
	l_fp	lastref;	/* last timecode time */
	l_fp	offset[NCODES];	/* recent sample offsets */
	char	lastcode[BMAX];	/* last timecode received */
	u_short	polled;		/* when polled, means a last sample */
	u_char	lencode;	/* length of last received ASCII string */
	u_long lasttime;	/* last time clock heard from */
#ifdef TRAKPPS
	u_long lastev;		/* last ppsclock second */
#endif /* TRAKPPS */
	u_char unit;		/* unit number for this guy */
	u_char status;		/* clock status */
	u_char lastevent;	/* last clock event */
	u_char reason;		/* reason for last abort */
	u_char year;		/* year of eternity */
	u_short day;		/* day of year */
	u_char hour;		/* hour of day */
	u_char minute;		/* minute of hour */
	u_char second;		/* seconds of minute */
	u_short msec;		/* milliseconds of second */
	u_char leap;		/* leap indicators */
	u_long yearstart;	/* start of current year */
	/*
	 * Status tallies
 	 */
	u_long polls;		/* polls sent */
	u_long noreply;		/* no replies to polls */
	u_long coderecv;	/* timecodes received */
	u_long badformat;	/* bad format */
	u_long baddata;		/* bad data */
	u_long timestarted;	/* time we started this */
};

/*
 * Function prototypes
 */
static	int	trak_start	P((int, struct peer *));
static	void	trak_shutdown	P((int, struct peer *));
static	void	trak_receive	P((struct recvbuf *));
static	void	trak_poll	P((int, struct peer *));
static	char	trak_process	P((struct trakunit *, l_fp *, u_fp *));
static	void	trak_send	P((struct trakunit *, char *));

/*
 * Transfer vector
 */
struct	refclock refclock_trak = {
	trak_start,		/* start up driver */
	trak_shutdown,		/* shut down driver */
	trak_poll,		/* transmit poll message */
	noentry,		/* not used (old trak_control) */
	noentry,		/* initialize driver (not used) */
	noentry,		/* not used (old trak_buginfo) */
	NOFLAGS			/* not used */
};


/*
 * trak_start - open the devices and initialize data for processing
 */
static int
trak_start(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct trakunit *up;
	struct refclockproc *pp;
	int fd;
	char device[20];

	/*
	 * Open serial port
	 */
	(void)sprintf(device, TRAK232, unit);
	if (!(fd = refclock_open(device, SPEED232, 0)))
		return (0);

	/*
	 * Allocate and initialize unit structure
	 */
	if (!(up = (struct trakunit *)
	    emalloc(sizeof(struct trakunit)))) {
		(void) close(fd);
		return (0);
	}
	memset((char *)up, 0, sizeof(struct trakunit));
	up->io.clock_recv = trak_receive;
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
	peer->precision = TRAKPRECISION;
	pp->clockdesc = TRAKDESCRIPTION;
	memcpy((char *)&pp->refid, TRAKREFID, 4);

	/*
	 * request to give time code
	 */
	trak_send(up, "\rRQTS,U\r");
	trak_send(up, "SEL 00\r");
	return (1);
}


/*
 * trak_shutdown - shut down the clock
 */
static void
trak_shutdown(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct trakunit *up;
	struct refclockproc *pp;

	pp = peer->procptr;
	up = (struct trakunit *)pp->unitptr;
	io_closeclock(&up->io);
	free(up);
}


/*
 * trak_receive - receive data from the serial interface
 */
static void
trak_receive(rbufp)
	struct recvbuf *rbufp;
{
	register struct trakunit *up;
	struct refclockproc *pp;
	struct peer *peer;

#if defined(TRAKPPS)
	struct ppsclockev ev;
	l_fp trtmp;
#endif /* TRAKPPS */
	int i, cmdtype;
	u_char *dpt;
	u_char *cp;
	u_char *dpend;
	l_fp tstmp;
	u_fp dispersion;

	/*
	 * Get the clock this applies to and pointers to the data.
	 * Edit the timecode to remove control chars and trashbits.
	 */
	up = (struct trakunit *)rbufp->recv_srcclock;
	pp = &up->proc;
	peer = up->peer;
	dpt = (u_char *)&rbufp->recv_space;
	dpend = dpt + rbufp->recv_length;
	cp = (u_char *)up->lastcode;

	while (dpt < dpend) {
#ifdef TRAKCLK	/* prior to TRAKPPS due to timestamp */
		if ((*cp = 0x7f & *dpt++) != '*' ) cp++;
		else if (*cp == '*' ) { /* caught magic character */
			if ( dpend - dpt < 8) {
				/* short timestamp */
				if(debug) puts("gps: short timestamp.");
				return;
			}
			if (!buftvtots(dpt,&up->lastrec)) {
				/* screwy timestamp */
				if(debug) puts("gps: screwy timestamp.");
				return;
			}
			dpt += 8;
		}
#else
#ifdef TRAKPPS
		if ((*cp = 0x7f & *dpt++) >= ' ') cp++;
#else
	/* both are not specified */
#endif /* TRAKPPS */
#endif /* TRAKCLK */
	}
	*cp = '\0';
	up->lencode = cp - (u_char *)up->lastcode;
	if (up->lencode == 0) return;

#ifdef DEBUG
	if (debug)
        	printf("gps: timecode %d %s\n",
		    up->lencode, up->lastcode);
#endif

	/*
	 * We check the timecode format and decode its contents. The
	 * timecode has format *........RQTS U,ddd:hh:mm:ss.0,Q\r\n).
         *                                     012345678901234567890123
	 */
#define RQTS	0
#define RQLS	1
	cp = (u_char *)up->lastcode;
	up->leap = 0;
	cmdtype=0;
	if(strncmp(cp,"*RQTS",5)==0) {
		cmdtype=RQTS;
		cp += 8;
		}
	else if(strncmp(cp,"RQTS",4)==0) {
		cmdtype=RQTS;
		cp += 7;
		}
	else if(strncmp(cp,"RQLS",4)==0) {
		cmdtype=RQLS;
		cp += 5;
		}
	else
		return;

	switch( cmdtype ) {
	case RQTS:
		/*
		 *	Check time code format of TRAK 8810
		 */
		if(	!isdigit(cp[0]) ||
			!isdigit(cp[1]) ||
			!isdigit(cp[2]) ||
			cp[3] != ':'	||
			!isdigit(cp[4]) ||
			!isdigit(cp[5]) ||
			cp[6] != ':'	||
			!isdigit(cp[7]) ||
			!isdigit(cp[8]) ||
			cp[9] != ':'	||
			!isdigit(cp[10])||
			!isdigit(cp[11])) {
				refclock_report(peer, CEVNT_BADREPLY);
				return;
			}
		break;
	case RQLS:
		/*
		 * reply for leap second request
		 */
		if (cp[0] !='0' || cp[1] != '0' ) up->leap = LEAP_ADDSECOND;
		return;
	default:
		return;

	}

	/*
	 * Convert date and check values.
	 */
	up->day = cp[0] - '0';
	up->day = MULBY10(up->day) + cp[1] - '0';
	up->day = MULBY10(up->day) + cp[2] - '0';
	if (up->day < 1 || up->day > 366) {
		refclock_report(peer, CEVNT_BADDATE);
		return;
	}
	/*
	 * Convert time and check values.
	 */
	up->hour = MULBY10(cp[4] - '0') + cp[5] - '0';
	up->minute = MULBY10(cp[7] - '0') + cp[8] -  '0';
	up->second = MULBY10(cp[10] - '0') + cp[11] - '0';
	up->msec = 0; 
	if (up->hour > 23 || up->minute > 59 || up->second > 59) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}

	if (!up->polled) return;

	/*
	 * Test for synchronization  Check for quality byte.
	 */
/*
	switch( cp[15] ) {
	case '0':
		if(up->peer->stratum == stratumtouse[up->unit]) {
			up->peer->stratum = 10 ;
			memset(&up->peer->refid, 0, 4);
		}
		break;
	default:
		if(up->peer->stratum != stratumtouse[up->unit]) {
			up->peer->stratum = stratumtouse[up->unit]   ;
			memmove(GPSREFID, &up->peer->refid, 4);
		}
		break;
	}
*/
	if( cp[15] == '0') /* TRAK derailed from tracking satellites */
		{
		up->leap = LEAP_NOTINSYNC;
		refclock_report(peer, CEVNT_TIMEOUT);
		}
	else
		{	
		up->lasttime = current_time;
		if( up->lastevent == CEVNT_TIMEOUT ) {
			refclock_report(peer, CEVNT_NOMINAL);
			}
		}

	/*
	 * Now, compute the reference time value. Use the heavy
	 * machinery for the second, which presumably is the one which
	 * occured at the last pps pulse and which was captured by the
	 * loop_filter module. All we have to do here is present a
	 * reasonable facsimile of the time at that pulse so the clock-
	 * filter and selection machinery declares us truechimer. The
	 * precision offset within the second is really tuned by the
	 * loop_filter module. Note that this code does not yet know how
	 * to do the years and relies on the clock-calendar chip for
	 * sanity.
	 */

#if defined(TRAKPPS)

	/*
	 *  timestamp must be greater than previous one.
	 */
	if (ioctl(fdpps, CIOGETEV, (caddr_t)&ev) >= 0) {
		ev.tv.tv_sec += (u_long)JAN_1970;
		TVTOTS(&ev.tv,&up->lastrec);
		if (up->lastev < ev.tv.tv_sec) {
			up->lastev = ev.tv.tv_sec;
		} else {	/* in case of 1-pps missing */
			up->lastev = ev.tv.tv_sec;
			return;
		}
	}
	else
		return;	/* failed to get timestamp */
#endif /* TRAKPPS */

	if (!clocktime(up->day, up->hour, up->minute,
	    up->second, GMT, up->lastrec.l_ui,
	    &up->yearstart, &up->lastref.l_ui)) {
		refclock_report(peer, CEVNT_BADTIME);
#ifdef DEBUG		
		if(debug) printf("gps: bad date \n");
#endif
		return;
	}
	MSUTOTSF(up->msec, up->lastref.l_uf);
	tstmp = up->lastref;

	L_SUB(&tstmp, &up->lastrec);
	L_ADD(&tstmp, &pp->fudgetime1);
	i = ((int)(up->coderecv)) % NCODES;
	up->offset[i] = tstmp;
	up->coderecv++;
#if DEBUG
	if (debug)
		printf("gps: times %s %s %s\n",
		    ulfptoa(&up->lastref, 6), ulfptoa(&up->lastrec, 6),
		    lfptoa(&tstmp, 6));
#endif
/*	if( tstmp.l_ui != 0 ) return;  something wrong */

	/*
	 * Process the samples in the median filter, add the fudge
	 * factor and pass the offset and dispersion along. We use
	 * lastref as both the reference time and receive time in order
	 * to avoid being cute, like setting the reference time later
	 * than the receive time, which may cause a paranoid protocol
	 * module to chuck out the data.
 	 */
	if (up->coderecv < NCODES)
		return;
	if (!trak_process(up, &tstmp, &dispersion)) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}
	refclock_receive(up->peer, &tstmp, GMT, dispersion,
	    &up->lastrec, &up->lastrec, up->leap);
	/*
	 *	after all, clear polled flag
	*/
	up->polled = 0;
}

/*
 * ==================================================================
 *	trak_send(gps,cmd)  Sends a command to the GPS receiver.
 *	 as	trak_send(up,"rqts,u\r");
 * ==================================================================
 */
static void
trak_send(up, cmd)
	struct trakunit *up;
	char *cmd;
{
	struct peer *peer;

	peer = up->peer;
	if (write(up->io.fd, cmd, strlen(cmd)) == -1) {
		refclock_report(peer,CEVNT_FAULT);
	} else {
		up->polls++;
	}
}

/*
 * trak_process - process a pile of samples from the clock
 *
 * This routine uses a three-stage median filter to calculate offset and
 * dispersion and reduce jitter. The dispersion is calculated as the
 * span of the filter (max - min).
 */
static char
trak_process(up, offset, dispersion)
	struct trakunit *up;
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
	 * and use that as the sample offset. There probably is not much
	 * to be gained by a longer filter, since the clock filter in
	 * ntp_proto should do its thing.
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
				return (0);
			}
			disp_tmp = MFPTOFP(0, tmp_uf);
			if (disp_tmp > disp_tmp2) {
				disp_tmp2 = disp_tmp;
				not_median1 = i;
				not_median2 = j;
			}
		}
	}
	if (up->lasttime == 0)
	    disp_tmp2 = NTP_MAXDISPERSE;
	else
	    disp_tmp2 = current_time - up->lasttime;
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
	return (1);
}

/*
 * trak_poll - called by the transmit procedure
 *
 * We go to great pains to avoid changing state here, since there may be
 * more than one eavesdropper receiving the same timecode.
 */
static void
trak_poll(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct trakunit *up;
	struct refclockproc *pp;

	pp = peer->procptr;
	up = (struct trakunit *)pp->unitptr;
	if ((current_time - up->lasttime) > 150)
		refclock_report(peer, CEVNT_TIMEOUT);
	/*
	 * usually trak_receive can get a timestamp every second
	 */
#if !defined(TRAKPPS) && !defined(TRAKCLK)
	gettstamp(&up->lastrec);
#endif
	up->polls++;
	/*
	 * may be polled every 16 seconds (minpoll 4)
	 */
	up->polled = 1;
}

#endif
