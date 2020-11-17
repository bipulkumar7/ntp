/*
 * refclock_local - local pseudo-clock driver
 */
#if defined(REFCLOCK) && defined(LOCAL_CLOCK)

#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>

#include "ntpd.h"
#include "ntp_refclock.h"
#include "ntp_stdlib.h"

/*
 * This is a hack to allow a machine to use its own system clock as a
 * "reference clock", i.e. to free run against its own clock at a non-
 * infinity stratum.  This is certainly useful if you want to use NTP in
 * an isolated environment with no radio clock (not that this is a good
 * idea) to synchronize the machines together. Pick a machine that you
 * figure has a good clock and configure it with a local reference
 * clock. Then point all the other machines at the one you're using as
 * the reference or use broadcast mode to distribute time.
 *
 * The other thing this is good for is if you want to use a particular
 * server's clock as the last resort, when all radio time has gone away.
 * This is especially good if that server has an ovenized oscillator or
 * something which will keep the time stable for extended periods, since
 * then all the other machines can benefit from this. For this you would
 * configure a local clock at a higher stratum (say 3 or 4) to prevent
 * the server's stratum from falling below here.
 *
 * The stratum for this driver LCLSTRATUM is set at 3 by default, but
 * can be changed by configuration commands and/or the xntpdc utility.
 * The reference ID is "LCL" by default, but can be changed by
 * configuration commands and/or the xntpdc utility. *NEVER* configure
 * this driver to operate at stratum zero or at a stratum which might
 * possibly disrupt a client with access to a bona fide primary server.
 * *NEVER NEVER* configure a server which might devolve to a local clock
 * to use multicast mode.
 */

/*
 * Local interface definitions
 */
#define	PRECISION	(-7)	/* about 10 ms precision */
#define	REFID		"LCL\0"	/* reference ID */
#define	DESCRIPTION	"Undisciplined local clock" /* WRU */

#define STRATUM	3		/* default stratum */
#define DISPERSION	(FP_SECOND / 100) /* default dispersion (10 ms) */

/*
 * Imported from the timer module
 */
extern u_long current_time;

/*
 * Imported from ntp_proto
 */
extern s_char sys_precision;

/*
 * Function prototypes
 */
static	int	local_start	P((int, struct peer *));
static	void	local_poll	P((int, struct peer *));

/*
 * Transfer vector
 */
struct	refclock refclock_local = {
	local_start,		/* start up driver */
	noentry,		/* shut down driver (not used) */
	local_poll,		/* transmit poll message */
	noentry,		/* not used (old lcl_control) */
	noentry,		/* initialize driver (not used) */
	noentry,		/* not used (old lcl_buginfo) */
	NOFLAGS			/* not used */
};


/*
 * local_start - start up the clock
 */
static int
local_start(unit, peer)
	int unit;
	struct peer *peer;
{
	register struct refclockproc *pp;

	pp = peer->procptr;

	/*
	 * Initialize miscellaneous variables
	 */
	peer->precision = sys_precision;
	pp->clockdesc = DESCRIPTION;
	pp->stratum = STRATUM;
	peer->stratum = pp->stratum;
	memcpy((char *)&pp->refid, REFID, 4);
	return (1);
}


/*
 * local_poll - called by the transmit procedure
 */
static void
local_poll(unit, peer)
	int unit;
	struct peer *peer;
{
	struct refclockproc *pp;

	pp = peer->procptr;
	pp->polls++;
	pp->lasttime = current_time;

	/*
	 * Ramble through the usual filtering and grooming code, which
	 * is essentially a no-op and included mostly for pretty
	 * billboards.
	 */
	pp->dispersion = DISPERSION;
	gettstamp(&pp->lastrec);
	refclock_receive(peer, &pp->offset, 0, pp->dispersion,
	    &pp->lastrec, &pp->lastrec, pp->leap);
}

#endif	/* REFCLOCK */
