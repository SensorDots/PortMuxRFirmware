# SensorDots Port MuxR Firmware

Firmware sources for the Port MuxR

## Installation

For setting up the Arduino environment for development or firmware update, please see https://sensordots.org/portmuxr_arduino
To load the latest precompiled hex file without installing the Arduino IDE you can use avrdude with the following command (substitute /dev/ttyUSBx with comX under Windows):

avrdude -v -p atmega328pb -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:portmuxr.hex:i


## Firmware Versions
The Port MuxR can be queried for its firmware version with the z serial command. 

   - v1_00 - Release Version

## Serial Menu

	h         - print menu
	i         - print board addr
	a[s]      - set all ports
				e.g. a1 -all ports on
	p[p][c][s]- set port
				e.g. p2a1 -port 2 channel a on
	v[p][s]   - set vcc, alt. to p[p]v[s]
				e.g. v21 -port 2 vcc on
	s         - get port and vcc states
	g[g][p][c]- add port to group
				e.g. g21b -add port 1-b to group 2
	G[g][p][c]- remove port from group
	x[g][s]   - set group
				e.g. x21 -group 2 on
	f         - get group map
	r         - reset groups
	m[m]      - set operate mode
				e.g. m1 - set bbm
	d[ms]     - mode switch delay ms<=900
	z         - check fw vers
	[options] - s =state (1=on/0=off)
				p =port (1-8)
				c =channel (a/b/v=vcc)
				g =group number (1-9)
				m =mode (0=man toggle,
				1=break-b4-make,2=make-b4-break)
	chain     - e.g. v11,v21 -port 1,2 vcc on (64 chars)
