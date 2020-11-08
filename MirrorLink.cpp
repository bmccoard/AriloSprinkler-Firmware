/*
  MirrorLink.cpp - LORA long range Mirror Link driver for OpenSprinkler

  Copyright (C) 2020 Javier Arigita

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

#include "program.h"
#include "defines.h"
#include "MirrorLink.h"
#include "OpenSprinkler.h"
#include "server_os.h"
#include <RadioLib.h>

#if defined(ESP32) && defined(MIRRORLINK_ENABLE)

// Pin definition
#define LORA_NSS  18
#define LORA_DIO1 33
#define LORA_DIO2 32
#define LORA_BUSY 26
#define LORA_SCLK 5
#define LORA_MOSI 27
#define LORA_MISO 19
#define LORA_RXEN 13
#define LORA_TXEN 25

// States of the MirrorLink driver
#if defined(MIRRORLINK_OSREMOTE)
enum MirrorlinkModes { MIRRORLINK_INIT, MIRRORLINK_ASSOCIATE, MIRRORLINK_BUFFERING, MIRRORLINK_SEND, MIRRORLINK_RECEIVE };
#else
enum MirrorlinkModes { MIRRORLINK_INIT, MIRRORLINK_ASSOCIATE, MIRRORLINK_SEND, MIRRORLINK_RECEIVE };
#endif

// Define buffers: need them to be sufficiently large to cover string option reading
extern char tmp_buffer[];

// ====== Object defines ======
extern OpenSprinkler os;
extern ProgramData pd;

// SX1262 has the following connections:
// NSS pin:   18
// DIO1 pin:  33
// DIO2 pin:  32
// BUSY pin:  26
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_DIO2, LORA_BUSY);

typedef union {
  uint8_t data;
  struct {
    uint8_t mirrorlinkState : 3;      // Operation mode of the MirrorLink system
    uint8_t receivedFlag : 1;         // Flag to indicate that a packet was received
    uint8_t transmittedFlag : 1;      // Flag to indicate that a packet was sent
    uint8_t enableInterrupt : 1;      // Disable interrupt when it's not needed
    uint8_t associated : 1;           // Shows if OS device is associated
    uint8_t flagRxTx : 1;             // Flag to indicate if module is receiving or transmitting
  };
} MirrorLinkStateBitfield;

struct MIRRORLINK {
  uint32_t timer;                           // Timer in seconds to control send timing
  int16_t module_state;                     // LORA module state
  MirrorLinkStateBitfield status;           // Bittfield including states as well as several flags
#if defined(MIRRORLINK_OSREMOTE)
  uint8_t bufferedCommands;                 // Number of buffered commands to be sent
  uint8_t bufferIndex;                      // Index of the element in the buffer to be sent
  uint32_t buffer[MIRRORLINK_BUFFERLENGTH]; // Buffer for queued commands to be sent to station
  uint32_t response;                        // Response from the station to the last command sent
#else
  uint32_t command;                         // Command to be executed by the station
#endif //MIRRORLINK_OSREMOTE
} MirrorLink;

void schedule_all_stations(ulong curr_time);
void schedule_station(byte sid, uint16_t duration);

// Set RX pin HIGH and TX pin LOW to switch to RECEIVE
void enableRX(void)
{
	digitalWrite(LORA_RXEN, HIGH);
	digitalWrite(LORA_TXEN, LOW);
	delay(100);
}

// Set TX pin HIGH and RX pin LOW to switch to TRANSMIT
void enableTX(void)
{
	digitalWrite(LORA_RXEN, LOW);
	digitalWrite(LORA_TXEN, HIGH);
	delay(100);
}

#if !defined(MIRRORLINK_DEBUGRF)
// this function is called when a complete packet
// is received or transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!MirrorLink.status.enableInterrupt) {
    return;
  }

  if (MirrorLink.status.flagRxTx == ML_RECEIVING) {
    MirrorLink.status.receivedFlag = (uint8_t)true;
  }
  else {
    MirrorLink.status.transmittedFlag = (uint8_t)true;
  }
}
#else
// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlagRx(void) {
  // check if the interrupt is enabled
  if(!MirrorLink.status.enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  MirrorLink.status.receivedFlag = (uint8_t)true;
}

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlagTx(void) {
  Serial.println(F("setFlagTx transmission finished!"));  // check if the interrupt is enabled
  if(!MirrorLink.status.enableInterrupt) {
    return;
  }
  // we sent a packet, set the flag
  MirrorLink.status.transmittedFlag = (uint8_t)true;
}
#endif // !defined(MIRRORLINK_DEBUGRF)

#if defined(MIRRORLINK_OSREMOTE)
void MirrorLinkBuffCmd(uint8_t cmd, uint32_t payload) {
  switch (cmd) {
    // Initial state
    case ML_TESTSTATION:
    	// Buffer message format: 
			// bit 0 = status (1 = On, 0 = Off)
			// bit 1 to 8 = sid
			// bit 9 to 24 = time(sec)
      // bit 25 to 26 = Not used
      // bit 27 to 31 = cmd
    case ML_PROGRAMSTARTTIME:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 8 = start time number (max. is 4 for each program)
      // bit 9 to 24 = start time
      // bit 25 = Starttime type
      // bit 26 = Not used
      // bit 27 to 31 = cmd
    case ML_PROGRAMDURATION:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 14 = sid
      // bit 15 to 25 = time (min)
      // bit 26 = Not used
      // bit 27 to 31 = cmd
      MirrorLink.buffer[0] = (((0x1F & (uint16_t)cmd) << 27) | payload);
      MirrorLink.bufferedCommands += 1;
      MirrorLink.bufferIndex = 0;
      break;
  }
}
#else
uint32_t MirrorLinkGetCmd(uint8_t cmd)
{
  uint32_t payload = 0;
  if(cmd == (MirrorLink.command >> 27)) {
    payload = (MirrorLink.command & 0x7FFFFFF);
  }
  else {
    payload = 0;
  }
  return payload;
}
#endif //defined(MIRRORLINK_OSREMOTE)

// MirrorLink module initialization
void MirrorLinkInit(void) {
#if !defined(MIRRORLINK_DEBUGRF)
  MirrorLink.module_state = ERR_NONE;
  MirrorLink.status.receivedFlag = (uint8_t)false;
  MirrorLink.status.transmittedFlag = (uint8_t)false;
  MirrorLink.status.enableInterrupt = (uint8_t)true;
  MirrorLink.status.mirrorlinkState = MIRRORLINK_INIT;
  MirrorLink.status.flagRxTx = ML_RECEIVING;
  MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
#if defined(MIRRORLINK_OSREMOTE)
  MirrorLink.bufferedCommands = 0;
  for (uint8_t i = 0; i < MIRRORLINK_BUFFERLENGTH; i++) MirrorLink.buffer[i] = 0;
  MirrorLink.bufferIndex = 0;
#endif // defined(MIRRORLINK_OSREMOTE)

  // TODO: Change this with flash check of associated adress:
  MirrorLink.status.associated = (uint8_t)true;

	Serial.begin(115200);

	// Configure LORA module pins
	pinMode(LORA_SCLK, OUTPUT); // SCLK
	pinMode(LORA_MISO, INPUT);  // MISO
	pinMode(LORA_MOSI, OUTPUT); // MOSI
	pinMode(LORA_NSS, OUTPUT);  // CS
	pinMode(LORA_DIO1, INPUT);  // DIO1
	pinMode(LORA_DIO2, INPUT);  // DIO1
	pinMode(LORA_BUSY, INPUT);  // DIO1

	// SCLK GPIO 5, MISO GPIO 19, MOSI GPIO 27, CS == NSS GPIO 18
	SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_NSS);
	Serial.print(F("[SX1262] Initializing ... "));
	// initialize SX1262 
	// carrier frequency:           868.0 MHz
	// bandwidth:                   125.0 kHz
	// spreading factor:            7
	// coding rate:                 5
	// sync word:                   0x1424 (private network)
	// output power:                0 dBm
	// current limit:               60 mA
	// preamble length:             8 symbols
	// CRC:                         enabled
	MirrorLink.module_state = lora.begin(868.0, 125.0, 7, 5, 0x1424, 0, 8);
	  if (MirrorLink.module_state == ERR_NONE) {
    Serial.println(F("success!"));
	} else {
		Serial.print(F("failed, code "));
		Serial.println(MirrorLink.module_state);
	}
	// eByte E22-900M uses DIO3 to supply the external TCXO
	if (lora.setTCXO(2.4) == ERR_INVALID_TCXO_VOLTAGE)
	{
		Serial.println(F("Selected TCXO voltage is invalid for this module!"));
	}

	// set the function that will be called
	// when new packet is transmitted or received
  lora.setDio1Action(setFlag);
#else
#if defined(MIRRORLINK_OSREMOTE)
	// set the function that will be called
	// when new packet is transmitted
	lora.setDio1Action(setFlag);

	// Important! To enable transmit you need to switch the SX126x antenna switch to TRANSMIT
	enableTX();

	// start transmitting the first packet
	Serial.print(F("[SX1262] Sending first packet ... "));

	// you can transmit C-string or Arduino string up to
	// 256 characters long
	MirrorLink.module_state = lora.startTransmit("Hello World!");

	// you can also transmit byte array up to 256 bytes long
	/*
		byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
						0x89, 0xAB, 0xCD, 0xEF};
		MirrorLink.module_state = lora.startTransmit(byteArr, 8);
	*/
#else
	// set the function that will be called
	// when new packet is received
	lora.setDio1Action(setFlagRx);

	// Important! To enable receive you need to switch the SX126x antenna switch to RECEIVE 
	enableRX();

	// start listening for LoRa packets
	Serial.print(F("[SX1262] Starting to listen ... "));
	MirrorLink.module_state = lora.startReceive();
	if (MirrorLink.module_state == ERR_NONE) {
		Serial.println(F("success!"));
	} else {
		Serial.print(F("failed, code "));
		Serial.println(MirrorLink.module_state);
		while (true);
	}
#endif // defined(MIRRORLINK_OSREMOTE)
#endif // defined(MIRRORLINK_DEBUGRF)
}

bool MirrorLinkTransmitStatus(void) {
  bool txSuccessful = false;
  if (MirrorLink.status.transmittedFlag == (uint8_t)true) {
    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint8_t)false;
    MirrorLink.status.transmittedFlag = (uint8_t)false;

    if (MirrorLink.module_state == ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

#if defined(MIRRORLINK_OSREMOTE)
      MirrorLink.buffer[MirrorLink.bufferIndex] = 0;
#endif // defined(MIRRORLINK_OSREMOTE)
      txSuccessful = true;
    } 
    else {
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.module_state);
    }
    MirrorLink.status.enableInterrupt = (uint8_t)true;
  }
  return txSuccessful;
}

void MirrorLinkTransmit(void) {
	// Important! To enable transmit you need to switch the SX126x antenna switch to TRANSMIT
	enableTX();
  Serial.println(F("[SX1262] Starting to transmit ... "));
#if defined(MIRRORLINK_OSREMOTE)
  // Transmit buffered commands
  // You can transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    MirrorLink.module_state = lora.startTransmit(byteArr, 8);
  */
  byte byteArr[4] = {(byte)(0xFF & MirrorLink.buffer[0] >> 24) , (byte)(0xFF & MirrorLink.buffer[0] >> 16) , (byte)(0xFF & MirrorLink.buffer[0] >> 8) , (byte)(0xFF & MirrorLink.buffer[0])};
  MirrorLink.module_state = lora.startTransmit(byteArr, 4);
#else
  // TODO: Transmit answer to command
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    MirrorLink.module_state = lora.startTransmit(byteArr, 8);
  */
  byte byteArr[4] = {(byte)(0xFF & MirrorLink.command >> 24) , (byte)(0xFF & MirrorLink.command >> 16) , (byte)(0xFF & MirrorLink.command >> 8) , (byte)(0xFF & MirrorLink.command)};
  MirrorLink.module_state = lora.startTransmit(byteArr, 4);
#endif // defined(MIRRORLINK_OSREMOTE)
  MirrorLink.status.flagRxTx = ML_TRANSMITTING;
}

bool MirrorLinkReceiveStatus(void) {
  bool rxSuccessful = false;
  // check if the flag is set
  if(MirrorLink.status.receivedFlag) {

    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint8_t)false;

    // reset flag
    MirrorLink.status.receivedFlag = (uint8_t)false;

    // You can read received data as byte array
    /*
      byte byteArr[8];
      MirrorLink.module_state = lora.readData(byteArr, 8);
    */
    byte byteArr[4];
    MirrorLink.module_state = lora.readData(byteArr, 4);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      MirrorLink.module_state = lora.readData(byteArr, 8);
    */

    if (MirrorLink.module_state == ERR_NONE) {

#if defined(MIRRORLINK_OSREMOTE)
      MirrorLink.response = (((uint16_t)byteArr[0] << 24) | ((uint16_t)byteArr[1] << 16) | ((uint16_t)byteArr[2] << 8) | ((uint16_t)byteArr[3]));
#else
      MirrorLink.command = (((uint16_t)byteArr[0] << 24) | ((uint16_t)byteArr[1] << 16) | ((uint16_t)byteArr[2] << 8) | ((uint16_t)byteArr[3]));
#endif // defined(MIRRORLINK_OSREMOTE)
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1262] Data:\t\t"));
#if defined(MIRRORLINK_OSREMOTE)
      Serial.println(MirrorLink.response);
#else
      Serial.println(MirrorLink.command);
#endif // defined(MIRRORLINK_OSREMOTE)

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1262] RSSI:\t\t"));
      Serial.print(lora.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(lora.getSNR());
      Serial.println(F(" dB"));

      rxSuccessful = true;
    } else if (MirrorLink.module_state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.module_state);
    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    MirrorLink.status.enableInterrupt = (uint8_t)true;
  }
  return rxSuccessful;
}

void MirrorLinkReceiveInit(void) {
  // Important! To enable receive you need to switch the SX126x antenna switch to RECEIVE 
  enableRX();

  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  MirrorLink.module_state = lora.startReceive();
  if (MirrorLink.module_state == ERR_NONE) {
    Serial.println(F("success!"));
    MirrorLink.status.flagRxTx = ML_RECEIVING;
  } else {
    Serial.print(F("failed, code "));
    Serial.println(MirrorLink.module_state);
  }
}

// MirrorLink module state machine change function, called once every second
void MirrorLinkState(void) {
  switch (MirrorLink.status.mirrorlinkState) {
    // Initial state
    case MIRRORLINK_INIT:
      Serial.println(F("STATE: MIRRORLINK_INIT"));
#if defined(MIRRORLINK_OSREMOTE)
      if (MirrorLink.status.associated == (uint8_t)true) {
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
      }
      else {
        Serial.println(F("STATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
      }
#else
      if (MirrorLink.status.associated == (uint8_t)true) {
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
      else {
        Serial.println(F("STATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
      }
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
    // Association state 
    case MIRRORLINK_ASSOCIATE:
      if (MirrorLink.status.associated == (uint8_t)true) {
#if defined(MIRRORLINK_OSREMOTE)
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
#else
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
#endif // defined(MIRRORLINK_OSREMOTE)
      }
      break;
#if defined(MIRRORLINK_OSREMOTE)
    // Buffering state
    case MIRRORLINK_BUFFERING:
      // If timer to be able to use the channel again empty
      // AND buffer not empty
      // change state to send
      // TODO: Correct timer use
      MirrorLink.timer = 0;
      if (  (MirrorLink.timer == 0)
          &&(MirrorLink.bufferedCommands > 0)) {
        Serial.println(MirrorLink.bufferedCommands);
        Serial.println(F("STATE: MIRRORLINK_SEND"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        MirrorLinkTransmit();
      }
      break;
#endif // defined(MIRRORLINK_OSREMOTE)
    // Send state
    case MIRRORLINK_SEND:
      // If send process if finished
      // empty the command in the buffer
      // change state to receive
      Serial.print(F("MirrorLink.timer: "));
      Serial.println(MirrorLink.timer);
#if defined(MIRRORLINK_OSREMOTE)
      if (MirrorLinkTransmitStatus() == true) {
        if (MirrorLink.bufferedCommands > 0) MirrorLink.bufferedCommands--;
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
      else if (MirrorLink.timer == 0) {
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        MirrorLinkReceiveInit();
      }
#else
      if (   (MirrorLinkTransmitStatus() == true)
          || (MirrorLink.timer == 0)) {
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
    // Receive state
    case MIRRORLINK_RECEIVE:
#if defined(MIRRORLINK_OSREMOTE)
      // If confirmation of buffer commands received
      // OR timeout
      // change state to Buffering
      if (   (MirrorLinkReceiveStatus() == true)
          || (MirrorLink.timer == 0)) {
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
#else
      // If commands received
      // execute and change state to Send
      if (MirrorLinkReceiveStatus() == true) {
        if (MirrorLink.command != 0) {
          Serial.print(F("Command:"));
          Serial.println(MirrorLink.command);
          Serial.println((MirrorLink.command >> 27));
          // Execute command
          // TODO: Check possibilities from set_station_data and server_change_program
          switch (MirrorLink.command >> 27) {
            // Initial state
            case ML_TESTSTATION:
              // Message format: 
              // bit 0 = status (1 = On, 0 = Off)
              // bit 1 to 8 = sid
              // bit 9 to 24 = time(sec)
              // bit 25 to 26 = Not used
              // bit 27 to 31 = cmd
              uint32_t payload = MirrorLinkGetCmd((uint8_t)ML_TESTSTATION);
              byte sid = (byte) (0xFF & (payload >> 1));
              uint8_t en = (uint8_t) (payload & 0x1);
              uint16_t timer = (uint16_t) ((0xFFFF) & (payload >> 9));     
              schedule_station(sid, timer);
              break;
            case ML_PROGRAMSTARTTIME:
              // Message format:
              // Standard start time (value between 0 to 1440, by bits 0 to 10)
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 to 8 = start time number (max. is 4 for each program)
              // bit 9 to 24 = start time
              // bit 25 = Starttime type
              // bit 26 = Not used
              // bit 27 to 31 = cmd
              uint32_t payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMSTARTTIME);
              int16_t pid = (int16_t)(payload & 0x7F);
              uint8_t stTimeNum = (uint8_t)((payload >> 7) & 0x3);
              int16_t stTime = (int16_t)((payload >> 9) & 0xFFFF);
              uint8_t stTimeType = (uint8_t)((payload >> 25) &0x1);
              break;
            case ML_PROGRAMDURATION:
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 to 14 = sid
              // bit 15 to 25 = time (min)
              // bit 26 = Not used
              // bit 27 to 31 = cmd
              uint32_t payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMDURATION);
              int16_t pid = (int16_t)(payload & 0x7F);
              byte sid = (byte)((payload >> 7) & 0xFF);
              uint16_t duration = (uint16_t)((payload >> 15) & 0xF);
              break;
          }
          MirrorLink.command = 0;
        }
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("SATE: MIRRORLINK_SEND"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
        MirrorLink.timer = MIRRORLINK_RXTX_MAX_TIME;
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
      }
  }
}

// MirrorLink module state maction actions function, called once every second
void MirrorLinkWork(void) {
  switch (MirrorLink.status.mirrorlinkState) {
    // Initial state
    case MIRRORLINK_INIT:
      // Do nothing
      break;
    // Association state 
    case MIRRORLINK_ASSOCIATE:
      // TODO: Association algorithm
#if defined(MIRRORLINK_OSREMOTE)
      // Check if associating beacon answer
      // Send associating beacon if no answer
      // Register station ID if answer
#else
      // Wait of associating beacon
      // If reception register remote ID and send association feedback with station ID
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
#if defined(MIRRORLINK_OSREMOTE)
    // Buffering state
    case MIRRORLINK_BUFFERING:
      // Do nothing, commands are getting buffered
      break;
#endif // defined(MIRRORLINK_OSREMOTE)
    // Send state
    case MIRRORLINK_SEND:
#if defined(MIRRORLINK_OSREMOTE)
      // Send buffer
      // If number of buffered commands > 0
      // AND previous command successfully sent
      if (   (MirrorLink.bufferedCommands > 0)
          && (MirrorLinkTransmitStatus() == true)) {
        MirrorLinkTransmit();
      }
      // Empty buffer
      // Calculate idle time until next send period
#else
      // Send answer if not yet transmitted
      if (   (MirrorLink.timer == (MIRRORLINK_RXTX_MAX_TIME - MIRRORLINK_RXTX_DEAD_TIME))
          && (MirrorLink.status.flagRxTx == ML_RECEIVING)) {
        MirrorLinkTransmit();
      }
#endif // defined(MIRRORLINK_OSREMOTE)
      if (MirrorLink.timer > 0) MirrorLink.timer--;
      break;
    // Receive state
    case MIRRORLINK_RECEIVE:
      if (MirrorLink.timer > 0) MirrorLink.timer--;
      break;
  }
}

// MirrorLink module function, called once every second
void MirrorLinkMain(void) {
#if !defined(MIRRORLINK_DEBUGRF)

  // State changes
  MirrorLinkState();

  // State actions
  MirrorLinkWork();

#else
// If MirrorLink LORA station is a remote controller
#if defined(MIRRORLINK_OSREMOTE)
  // check if the previous transmission finished
  if(MirrorLink.status.transmittedFlag) {
    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint8_t)false;

    // reset flag
    MirrorLink.status.transmittedFlag = (uint8_t)false;

    if (MirrorLink.module_state == ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.module_state);

    }

    // wait a second before transmitting again
    //delay(1000);

    // send another one
    Serial.print(F("[SX1262] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    MirrorLink.module_state = lora.startTransmit("Hello World!");

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      MirrorLink.module_state = lora.startTransmit(byteArr, 8);
    */

    // we're ready to send more packets,
    // enable interrupt service routine
    MirrorLink.status.enableInterrupt = (uint8_t)true;
  }
#else
  // check if the flag is set
  if(MirrorLink.status.receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint8_t)false;

    // reset flag
    MirrorLink.status.receivedFlag = (uint8_t)false;

    // you can read received data as an Arduino String
    String str;
    MirrorLink.module_state = lora.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      MirrorLink.module_state = lora.readData(byteArr, 8);
    */

    if (MirrorLink.module_state == ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1262] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1262] RSSI:\t\t"));
      Serial.print(lora.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(lora.getSNR());
      Serial.println(F(" dB"));

    } else if (MirrorLink.module_state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.module_state);

    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    MirrorLink.status.enableInterrupt = (uint8_t)true;
  }
#endif // defined(MIRRORLINK_OSREMOTE)
#endif // // defined(MIRRORLINK_DEBUGRF)

}

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)