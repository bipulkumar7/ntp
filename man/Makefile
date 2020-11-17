MANDIR=/usr/local/man/cat8

all:;

install:
	/usr/man/manroff ntpd.8 > /usr/local/man/cat8/ntpd.0
	/usr/man/manroff ntp.8 >/usr/local/man/cat8/ntp.0
	/usr/man/manroff ntpdc.8 >/usr/local/man/cat8/ntpdc.0

clean:
	@rm -f *~
