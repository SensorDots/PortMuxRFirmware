/**
   SensorDots Port MuxR

   Copyright (C) 2019 SensorDots

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef _PORTMUXR_H_
#define _PORTMUXR_H_

#define EEPROM_SETTINGS_START 0x10
#define EEPROM_GROUP_START 0x50

#define VCC_1 TCA6424A_P00
#define VCC_2 TCA6424A_P01
#define VCC_3 TCA6424A_P02
#define VCC_4 TCA6424A_P03
#define VCC_5 TCA6424A_P04
#define VCC_6 TCA6424A_P05
#define VCC_7 TCA6424A_P06
#define VCC_8 TCA6424A_P07
#define PORT8_2 TCA6424A_P10
#define PORT8_1 TCA6424A_P11
#define PORT7_2 TCA6424A_P12
#define PORT7_1 TCA6424A_P13
#define PORT6_2 TCA6424A_P14
#define PORT6_1 TCA6424A_P15
#define PORT5_2 TCA6424A_P16
#define PORT5_1 TCA6424A_P17
#define PORT4_2 TCA6424A_P20
#define PORT4_1 TCA6424A_P21
#define PORT3_2 TCA6424A_P22
#define PORT3_1 TCA6424A_P23
#define PORT2_2 TCA6424A_P24
#define PORT2_1 TCA6424A_P25
#define PORT1_2 TCA6424A_P26
#define PORT1_1 TCA6424A_P27

#define NUM_OF_GROUPS 9

#define LED_STAT    15
#define ADDR0       7
#define ADDR1       6
#define ADDR2       5

#define IO_RST_OUTPUT (DDRB = (1<<DDB6))
#define IO_RST_HIGH (PORTB = (1<<PORTB6))

#define BUFFER_SIZE 64
#define I2C_BUFFER_SIZE 24

void delayMs(int ms)
{
    while (0 < ms)
    {
        _delay_ms(1);
        --ms;
    }
}

void printMenuSpace()
{
    Serial.print(F("            "));
}

void printMenu()
{

    Serial.println(F("h         - print menu"));
    Serial.println(F("i         - print board addr"));
    Serial.println(F("a[s]      - set all ports"));
    printMenuSpace();
    Serial.println(F(            "e.g. a1 -all ports on"));
    Serial.println(F("p[p][c][s]- set port"));
    printMenuSpace();
    Serial.println(F(            "e.g. p2a1 -port 2 channel a on"));
    Serial.println(F("v[p][s]   - set vcc, alt. to p[p]v[s]"));
    printMenuSpace();
    Serial.println(F(            "e.g. v21 -port 2 vcc on"));
    Serial.println(F("s         - get port and vcc states"));
    Serial.println(F("g[g][p][c]- add port to group"));
    printMenuSpace();
    Serial.println(F(            "e.g. g21b -add port 1-b to group 2"));
    Serial.println(F("G[g][p][c]- remove port from group"));
    Serial.println(F("x[g][s]   - set group"));
    printMenuSpace();
    Serial.println(F(            "e.g. x21 -group 2 on"));
    Serial.println(F("f         - get group map"));
    Serial.println(F("r         - reset groups"));
    Serial.println(F("m[m]      - set operate mode"));
    printMenuSpace();
    Serial.println(F(            "e.g. m1 - set bbm"));
    Serial.println(F("d[ms]     - mode switch delay ms<=900"));
    Serial.println(F("z         - check fw vers"));
    Serial.println(F("[options] - s =state (1=on/0=off)"));
    printMenuSpace();
    Serial.println(F(            "p =port (1-8)"));
    printMenuSpace();
    Serial.println(F(            "c =channel (a/b/v=vcc)"));
    printMenuSpace();
    Serial.print  (F(            "g =group number (1-"));
    Serial.print(NUM_OF_GROUPS);
    Serial.println(F(")"));
    printMenuSpace();
    Serial.println(F(            "m =mode (0=man toggle,"));
    printMenuSpace();
    Serial.println(F(            "1=break-b4-make,2=make-b4-break)"));
    //Serial.println(F(            " note - pin a,b,c,d = channel a"));
    //Serial.println(F(            " note - pin e,f,g,h = channel b"));
    Serial.println(F("chain     - e.g. v11,v21 -port 1,2 vcc on (64 chars)"));

    Serial.println("");

}

void print_command_error() {
    Serial.println(F("command error"));
}

void serial_delay() {
    delayMs(5);
}

#endif
