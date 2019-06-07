# SensorDots Port MuxR Firmware

Firmware sources for the Port MuxR

## Installation

For setting up the Arduino environment for development or firmware update, please see https://sensordots.org/portmuxr_arduino

To load the latest precompiled hex file without installing the Arduino IDE you can use avrdude with the following command (substitute /dev/ttyUSBx with comX under Windows):

avrdude -v -p atmega328pb -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:portmuxr.hex:i

You should get the following output:

    avrdude: Version 6.3, compiled on Feb 17 2016 at 09:25:53
         Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
         Copyright (c) 2007-2014 Joerg Wunsch

         System wide configuration file is "avrdude.conf"

         Using Port                    : com17
         Using Programmer              : arduino
         Overriding Baud Rate          : 57600
         AVR Part                      : ATmega328PB
         Chip Erase delay              : 9000 us
         PAGEL                         : PD7
         BS2                           : PC2
         RESET disposition             : dedicated
         RETRY pulse                   : SCK
         serial program mode           : yes
         parallel program mode         : yes
         Timeout                       : 200
         StabDelay                     : 100
         CmdexeDelay                   : 25
         SyncLoops                     : 32
         ByteDelay                     : 0
         PollIndex                     : 3
         PollValue                     : 0x53
         Memory Detail                 :

                                  Block Poll               Page                       Polled
           Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
           ----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
           eeprom        65    20     4    0 no       1024    4      0  3600  3600 0xff 0xff
           flash         65     6   128    0 yes     32768  128    256  4500  4500 0xff 0xff
           lfuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           hfuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           lock           0     0     0    0 no          1    0      0  4500  4500 0x00 0x00
           calibration    0     0     0    0 no          1    0      0     0     0 0x00 0x00
           signature      0     0     0    0 no          3    0      0     0     0 0x00 0x00
           efuse          0     0     0    0 no          1    0      0  4500  4500 0x00 0x00

         Programmer Type : Arduino
         Description     : Arduino
         Hardware Version: 3
         Firmware Version: 4.4
         Vtarget         : 0.3 V
         Varef           : 0.3 V
         Oscillator      : 28.800 kHz
         SCK period      : 3.3 us
    
    avrdude: AVR device initialized and ready to accept instructions
    
    Reading | ################################################## | 100% 0.02s
    
    avrdude: Device signature = 0x1e9516 (probably m328pb)
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: reading input file "portmuxr_v1_00.hex"
    avrdude: writing flash (12480 bytes):
    
    Writing | ################################################## | 100% 4.14s
    
    avrdude: 12480 bytes of flash written
    avrdude: verifying flash memory against portmuxr_v1_00.hex:
    avrdude: load data flash data from input file portmuxr_v1_00.hex:
    avrdude: input file portmuxr_v1_00.hex contains 12480 bytes
    avrdude: reading on-chip flash data:
    
    Reading | ################################################## | 100% 3.82s
    
    avrdude: verifying ...
    avrdude: 12480 bytes of flash verified
    
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: safemode: Fuses OK (E:00, H:00, L:00)
    
    avrdude done.  Thank you.


## Firmware Versions
The Port MuxR can be queried for its firmware version with the z serial command. 

   - v1_00 - Release Version - [commit](https://github.com/SensorDots/PortMuxRFirmware/commit/463ab5feef9b8fb956a4d9e0db8824780c17636a)
   - v1_10 - Fix I2C lockups and minor bugs.

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
