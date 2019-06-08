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

#include "TCA6424A.h"
#include "I2Cdev.h"
#include "port_muxr_firmware.h"
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <Wire1.h>
#include <EEPROM.h>

#define FIRMWARE_VERSION "v1_10"

/***                        ***/
/*** Global State Variables ***/
/***                        ***/
uint8_t vcc[8] =
{
  VCC_1, VCC_2, VCC_3, VCC_4, VCC_5, VCC_6, VCC_7, VCC_8
};

uint8_t ports_a[8] =
{
  PORT1_1, PORT2_1, PORT3_1, PORT4_1, PORT5_1, PORT6_1, PORT7_1, PORT8_1
};

uint8_t ports_b[8] =
{
  PORT1_2, PORT2_2, PORT3_2, PORT4_2, PORT5_2, PORT6_2, PORT7_2, PORT8_2
};

uint32_t group_matrix[NUM_OF_GROUPS] = {0};
uint32_t port_state = 0;
uint8_t operating_mode = 0;
uint16_t mode_delay = 200;

uint8_t board_address = 0x50;

uint8_t i2c_buffer[I2C_BUFFER_SIZE];
uint8_t i2c_ret_data_buffer[I2C_BUFFER_SIZE];
uint8_t i2c_ret_data_buffer_len = 0;
uint8_t i2c_receive_command_ready = 0;

TCA6424A digital_io;


/***                ***/
/*** Port Functions ***/
/***                ***/

/** Set all ports to state. VCC ports are set to input (HiZ) to turn off and set output low to turn on.
   @param state boolean state value (1=on, 0=off)
   @return void
*/
void setAllPorts(uint8_t state)
{
  if (state == 1) {
    //Set all ports to on
    digital_io.setAllDirection(0x00, 0x00, 0x00);
    digital_io.writeAll(0x00, 0x00, 0x00);
  } else {
    //Set all ports to off
    digital_io.setAllDirection(0xFF, 0x00, 0x00);
    digital_io.writeAll(0xFF, 0xFF, 0xFF);
  }
}

/** TCA6424A get port state helper function
   @param port_num the port number with respect to TCA6424A
   @return boolean port value (1=on, 0=off)
*/
uint8_t getPortState(uint8_t port_num)
{
  if (port_num < 8)
  {
    return !digital_io.getPinDirection(port_num);
  } else {
    return !digital_io.getPinOutputLevel(port_num);
  }
}

/** Gets all ports state

    channel a port  | 1| 2| 3| 4| 5| 6| 7| 8|
    bits:           | 0| 1| 2| 3| 4| 5| 6| 7|

    channel b port  | 1| 2| 3| 4| 5| 6| 7| 8|
    bits:           | 8| 9|10|11|12|13|14|15|

    vcc port        | 1| 2| 3| 4| 5| 6| 7| 8|
    bits:           |16|17|18|19|20|21|22|23| 24-> ignore

   @return 24bit (lsb) integer representation of port state
*/
uint32_t getAllPortState()
{
  uint32_t pins = 0;
  uint8_t pin_position = 0;

  for (uint8_t i = 0; i < 8; i++)
  {
    pins |= ((uint32_t)getPortState(vcc[i])) << pin_position;
    pin_position++;
  }
  for (uint8_t i = 0; i < 8; i++)
  {
    pins |= ((uint32_t)getPortState(ports_a[i])) << pin_position;

    pin_position++;
  }
  for (uint8_t i = 0; i < 8; i++)
  {
    pins |= ((uint32_t)getPortState(ports_b[i])) << pin_position;

    pin_position++;
  }
  return pins;
}

/** TCA6424A set port state helper function
   @param port_num the port number with respect to TCA6424A
   @param state boolean state value (1=on, 0=off)
   @return void
*/
void setPort(uint8_t port_num, uint8_t state)
{
  if (state == 1)
  {
    if (port_num < 8)
    {
      digital_io.setPinDirection(port_num, 0); //VCC goes high Z
      digital_io.writePin(port_num, 0);
    } else {
      digital_io.writePin(port_num, 0);
    }
  } else { //Set port to off if 0 or unknown
    if (port_num < 8)
    {
      digital_io.setPinDirection(port_num, 1);
    } else {
      digital_io.writePin(port_num, 1);
    }
  }
}

/** Update EEPROM with group settings
   @return void
*/
void updateGroupEEPROM() {

  //Set group matrix in EEPROM
  for (int i = 0; i < NUM_OF_GROUPS; i++)
  {
    //Only write EEPROM if changed
    EEPROM.update(EEPROM_GROUP_START + ((i) * 3) + 2, (group_matrix[i] >> 16) & 0xff);
    EEPROM.update(EEPROM_GROUP_START + ((i) * 3) + 1, (group_matrix[i] >> 8 ) & 0xff);
    EEPROM.update(EEPROM_GROUP_START + ((i) * 3) + 0, (group_matrix[i]      ) & 0xff);
  }

  //Mark EEPROM as written to once (EEPROM is all 0xff and this affects group mappings on first run)
  EEPROM.update(0, 0);
}

/***               ***/
/*** I2C Functions ***/
/***               ***/

/** Function that executes whenever data is received over i2c
   @param bytes_received number of bytes received
   @return void
*/
void receiveEvent(int bytes_received)
{
  //Prevent interruption of currently operating command
  if (i2c_receive_command_ready == 0) { 
    for (int i = 0; i < bytes_received; i++)
    {
      if (i < I2C_BUFFER_SIZE && Wire.available())
      {
        i2c_buffer[i] = Wire.read();
      }
      else
      {
        Wire.read();  //If receive more data than allowed just throw it away
      }
    }
    i2c_receive_command_ready = 1;
  }
  
  //Flush Wire Buffer
  while (Wire.available()) Wire.read();
}

/** Function that executes whenever data is requested over i2c
   @return void
*/
void requestEvent() {
  Wire.write(i2c_ret_data_buffer, i2c_ret_data_buffer_len);
}

/***             ***/
/*** Board Setup ***/
/***             ***/

void setup() {

  //Digital IO reset uses XTAL1 (PB6)
  IO_RST_OUTPUT;

  //Turn on digital io chip (active high)
  IO_RST_HIGH;

  //Check if EEPROM has been written to before
  if ((uint8_t)EEPROM.read(0) != 0xff) {

    //Get group matrix from EEPROM
    for (int i = 0; i < NUM_OF_GROUPS; i++)
    {
      group_matrix[i] = ((uint32_t)((uint8_t)EEPROM.read(EEPROM_GROUP_START + (i * 3) + 2))) << 16 |
                        ((uint32_t)((uint8_t)EEPROM.read(EEPROM_GROUP_START + (i * 3) + 1))) << 8 |
                        (uint8_t)EEPROM.read(EEPROM_GROUP_START + (i * 3));
    }
  }

  //Set address pins to input pullup to read address
  pinMode(ADDR0, INPUT_PULLUP);
  pinMode(ADDR1, INPUT_PULLUP);
  pinMode(ADDR2, INPUT_PULLUP);

  board_address = board_address + ((!digitalRead(ADDR0)) | (!digitalRead(ADDR1)) << 1 | (!digitalRead(ADDR2)) << 2);

  //Set back to input to save power.
  pinMode(ADDR0, INPUT);
  pinMode(ADDR1, INPUT);
  pinMode(ADDR2, INPUT);

  //Set sleep modes
  set_sleep_mode(SLEEP_MODE_IDLE);
  power_adc_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();

  //Initialise slave (Wire)
  Wire.begin(board_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //Initialize the LED pin as an output.
  pinMode(LED_STAT, OUTPUT);
  digitalWrite(LED_STAT, LOW); //Turn on LED

  Serial.begin(57600);

  //Digital I/O chipset (Wire1)
  Wire1.begin();
  Wire1.setClock(100000);

  digital_io.initialize();

  delayMs(40);

  //Test digital I/O chip
  if (digital_io.testConnection()) digitalWrite(LED_STAT, HIGH); //Turn off LED
  else digitalWrite(LED_STAT, LOW);

  //Set all ports to off
  setAllPorts(0);
}

void loop() {

  //while (!Serial) {
  // Wait for serial port
  //}

  uint8_t char_position = 0;
  int8_t read_char = 0;

  uint8_t serial_buffer[BUFFER_SIZE];

  memset(serial_buffer, 0, BUFFER_SIZE);

  // Note, if i2c command comes in, it will cancel an existing serial command
  while (read_char != '\r' && read_char != '\n' && char_position < BUFFER_SIZE && !i2c_receive_command_ready)
  {
    if (Serial.available()) {
      read_char = Serial.read();
      if (read_char != -1) {
        Serial.write(read_char);
        serial_buffer[char_position] = read_char;
        char_position++;
      }
    }
    //sleep_mode();
    serial_delay();
  }

  digitalWrite(LED_STAT, LOW);   // set the LED on

  if (i2c_receive_command_ready)
  {
    i2c_ret_data_buffer_len = processCommand(i2c_buffer, 0, 2, i2c_ret_data_buffer);
    i2c_receive_command_ready = 0;
  } else {

    Serial.println("");

    char_position = 0;


    delayMs(30);

process_next_command:

    char_position = processCommand(serial_buffer, char_position, 1, NULL);

    read_char = serial_buffer[char_position];
    char_position++;
    if (read_char == ',') {
      goto process_next_command;
    }
  }
  digitalWrite(LED_STAT, HIGH); // set the LED off

}

/** Process serial or I2C command
   @param cmd command byte array
   @param char_position charater position of cmd array
   @param serial_mode:
   if serial_mode == 1; then serial and return value = char_pos
   if serial_mode == 2; then i2c receive and return value = return array length if applicable (-1 is error)
   @param ret_array i2c return array, can be NULL if mode is serial
   @return return value depends on i2c or serial command (see above)
*/
int8_t processCommand(uint8_t * cmd, uint8_t char_position, uint8_t serial_mode, uint8_t * ret_array)
{

  uint8_t channel = 0;
  uint8_t port = 0;
  uint8_t state = 0;
  uint8_t group = 0;
  uint8_t was_set = 0;
  uint8_t vcc_set = 0;
  uint8_t mode = 0;
  uint16_t mode_delay_temp = 0;
  uint32_t port_changing = 0;
  char temp_buffer[3] = {0, 0, 0};
  uint8_t bit_array_start = 0;
  int8_t read_char = 0;

  switch (cmd[char_position])
  {
    /* Print menu */
    case 'h':
      if (serial_mode == 1) {
        char_position++;
        printMenu();
      } else {
        return 0;
      }
      break;

    /* Print board i2c address */
    case 'i' :
      if (serial_mode == 1) {
        char_position++;
        Serial.print(F("i2c_addr: 0x"));
        Serial.println(board_address, HEX);
      } else {
        return 0;
      }
      break;

    /* Print firmware version */
    case 'z' :
      if (serial_mode == 1) {
        Serial.println(F(FIRMWARE_VERSION));
      } else {
        memcpy(ret_array, F(FIRMWARE_VERSION), 5);
        return 5;
      }
      break;

    /* Fancy scrolling vcc display */
    /*case '*':
      char_position++;
      setPort(0, 1);
      for (uint8_t i = 1; i < 8; i++)
      {
        //turn on next
        setPort(i, 1);
        delayMs(20);
        //turn off previous
        setPort(i - 1, 0);
        delayMs(20);

      }
      setAllPorts(0);
      break;*/

    /* Get port and vcc states */
    case 's':
      char_position++;

      port_state = getAllPortState();

      if (serial_mode == 1) {

        for (uint8_t i = 0; i < 24; i++)
        {
          if (i == 0) Serial.print(F("vcc:"));
          if (i == 8) Serial.print(F(" port_a:"));
          if (i == 16) Serial.print(F(" port_b:"));
          Serial.print((port_state >> i) & 0x01);
          if (i != 7 && i != 15 && i != 23) Serial.print(F(","));
        }
        Serial.println("");
      } else {
        ret_array[0] = (port_state >> 0) & 0xFF;
        ret_array[1] = (port_state >> 8) & 0xFF;
        ret_array[2] = (port_state >> 16) & 0xFF;
        return 3;

      }

      break;

    /* Set port switch delay */
    case 'd':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      read_char = cmd[char_position];
      char_position++;
      temp_buffer[1] = read_char;
      read_char = cmd[char_position];
      char_position++;
      temp_buffer[2] = read_char;
      mode_delay_temp = atoi(temp_buffer);
      if (mode_delay_temp <= 900) {
        mode_delay = mode_delay_temp;
        if (serial_mode == 1) {
          Serial.print(F("delay:"));
          Serial.println(mode_delay);
        } else {
          return 0;
        }
      } else {
        if (serial_mode == 1) {
          print_command_error();
        } else {
          return -1;
        }
      }

      break;

    /* Set vcc state */
    case 'v':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      port = atoi(temp_buffer);

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      state = atoi(temp_buffer);

      vcc_set = 1;
      channel = 'v';

    //Fall through to p case

    /* Set port state */
    case 'p':

      if (!vcc_set) {
        char_position++;

        read_char = cmd[char_position];
        char_position++;
        temp_buffer[0] = read_char;
        port = atoi(temp_buffer);

        read_char = cmd[char_position];
        char_position++;
        channel = read_char;

        read_char = cmd[char_position];
        char_position++;
        temp_buffer[0] = read_char;
        state = atoi(temp_buffer);
      }

      if (port >= 1 && port <= 8 && (state == 0 || state == 1) &&
          (channel == 'a' || channel == 'b' || channel == 'v')) {

        if (operating_mode == 1) { //Break before make
          setAllPorts(0);
          delayMs(mode_delay);
        }

        if (channel == 'a') { //Set port on
          setPort(ports_a[port - 1], state);
          port_changing = ports_a[port - 1];
          was_set = 1;
        }
        else if (channel == 'b')
        {
          setPort(ports_b[port - 1], state);
          port_changing = ports_b[port - 1];
          was_set = 1;
        }
        else if (channel == 'v')
        {
          setPort(vcc[port - 1], state);
          port_changing = vcc[port - 1];
          was_set = 1;
        }

        if (operating_mode == 2) //Make before break
        {
          delayMs(mode_delay);

          for (uint8_t i = 0; i < 24; i++)
          {
            if (i != port_changing) setPort(i, 0);
          }
        }
      }

      if (serial_mode == 1) {
        if (was_set) {

          if (vcc_set)
          {
            Serial.print(F("vccport:"));
          } else {
            Serial.print(F("port:"));
          }
          Serial.print(port);
          if (!vcc_set)
          { Serial.print(F(", channel:"));
            Serial.write(channel);
          }
          Serial.print(F(", state:"));
          Serial.println(state);
        } else {
          print_command_error();
        }
      } else {
        return 0;
      }
      break;

    /* Set port switch mode */
    case 'm':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      mode = atoi(temp_buffer);

      if (mode == 0 || mode == 1 || mode == 2)
      {
        operating_mode = mode;
        if (serial_mode == 1) {
          Serial.print(F("mode:"));
          if (operating_mode == 0) Serial.println(F("man"));
          else if (operating_mode == 1) Serial.println(F("bbm"));
          else if (operating_mode == 2) Serial.println(F("mbb"));
        } else {
          ret_array[0] = operating_mode;
          return 1;
        }
      } else {
        if (serial_mode == 1) {
          print_command_error();
        } else {
          return -1;
        };
      }
      break;

    /* Set all ports */
    case 'a':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      state = atoi(temp_buffer);

      if (state == 1 || state == 0) {
        setAllPorts(state);
        was_set = 1;
      }

      if (was_set) {
        if (serial_mode == 1) {
          Serial.print(F("all:"));
          Serial.println(state);
        } else {
          return 0;
        }
      } else {
        if (serial_mode == 1) {
          print_command_error();
        } else {
          return -1;
        }
      }

      break;

    /* Add port to group */
    case 'g':
      state = 1;
    //Fall through to G case

    /* Remove port from group */
    case 'G':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      group = atoi(temp_buffer);
      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      port = atoi(temp_buffer);
      channel = cmd[char_position];
      char_position++;

      if (group >= 1 && group <= NUM_OF_GROUPS)
      {
        if (port >= 1 && port <= 8) {
          if (channel == 'a') {
            bit_array_start = 0;
            was_set = 1;
          }
          else if (channel == 'b')
          {
            bit_array_start = 8;
            was_set = 1;
          }
          else if (channel == 'v')
          {
            bit_array_start = 16;
            was_set = 1;
          }
        }
      }

      if (was_set) {
        if (state) group_matrix[group - 1] |= ((uint32_t)1 << (bit_array_start + port - 1));
        else group_matrix[group - 1] &= ~((uint32_t)1 << (bit_array_start + port - 1));

        //Set group matrix in EEPROM
        updateGroupEEPROM();
        if (serial_mode == 1) {
          Serial.print(F("group:"));
          Serial.print(group);
          Serial.print(F(", port:"));
          Serial.print(port);
          Serial.print(F(", channel:"));
          Serial.write(channel);
          Serial.println("");
        } else {
          return 0;
        }
      } else {
        if (serial_mode == 1) {
          print_command_error();
        } else {
          return -1;
        }
      }

      break;

    /* Show group table */
    case 'f':
      char_position++;
      if (serial_mode == 1) {
        Serial.println(F("cha pt  |1|2|3|4|5|6|7|8| chb pt |1|2|3|4|5|6|7|8| vcc pt |1|2|3|4|5|6|7|8|"));
        for (int row = 0; row < NUM_OF_GROUPS; row++) {
          Serial.print(F("group "));
          Serial.print(row + 1);
          Serial.print(F(" "));
          for (int column = 0; column < 24; column++) {
            if (column == 8 || column == 16) Serial.print(F("|        "));
            Serial.print(F("|"));
            if ((group_matrix[row] >> column) & 0x01) Serial.print(F("x"));
            else Serial.print(F(" "));
          }
          Serial.println(F("|"));
        }
      } else {
        return 0;
      }
      break;

    /* Reset group table */
    case 'r':
      char_position++;
      for (int row = 0; row < NUM_OF_GROUPS; row++) {
        group_matrix[row] = 0;
      }

      //Set group matrix in EEPROM
      updateGroupEEPROM();
      if (serial_mode == 1) {
        Serial.println(F("groups cleared"));
      } else {
        return 0;
      }
      break;

    /* Set group state */
    case 'x':
      char_position++;

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      group = atoi(temp_buffer);

      read_char = cmd[char_position];
      char_position++;
      temp_buffer[0] = read_char;
      state = atoi(temp_buffer);
      if (group >= 1 && group <= NUM_OF_GROUPS && (state == 1 || state == 0))
      {
        if (operating_mode == 1) { //Break before make
          setAllPorts(0);
          delayMs(mode_delay);
        }

        for (int column = 0; column < 8; column++) {
          if ((group_matrix[group - 1] >> column) & 0x01) {
            setPort(ports_a[column], state);
            port_changing |= ((uint32_t)1) << column;
          }
          if ((group_matrix[group - 1] >> (column + 8)) & 0x01) {
            setPort(ports_b[column], state);
            port_changing |= ((uint32_t)1) << (column + 8);
          }
          if ((group_matrix[group - 1] >> (column + 16)) & 0x01) {
            setPort(vcc[column], state);
            port_changing |= ((uint32_t)1) << (column + 16);
          }
        }

        if (operating_mode == 2) //Make before break
        {
          delayMs(mode_delay);

          //Turn off ports that weren't set
          for (int column = 0; column < 8; column++) {
            if (!((port_changing >> column) & 0x01)) {
              setPort(ports_a[column], 0);
            }
            if (!((port_changing >> (column + 8)) & 0x01)) {
              setPort(ports_b[column], 0);
            }
            if (!((port_changing >> (column + 16)) & 0x01)) {
              setPort(vcc[column], 0);
            }
          }
        }
        was_set = 1;
      }

      if (was_set) {
        if (serial_mode == 1) {
          Serial.print(F("group:"));
          Serial.print(group);
          Serial.print(F(", state:"));
          Serial.println(state);
        } else {
          return 0;
        }
      } else {
        if (serial_mode == 1) {
          print_command_error();
        } else {
          return 0;
        }
      }

      break;

    default:
      if (serial_mode == 1) {
        Serial.println(F("unknown command"));
      } else {
        return -1;
      }
      break;
  }

  return char_position;
}
