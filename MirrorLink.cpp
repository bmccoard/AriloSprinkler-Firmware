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

#include "OpenSprinkler.h"
#include "defines.h"
#include "program.h"
#include "MirrorLink.h"
#include "server_os.h"
#include <RadioLib.h>
#include "weather.h"

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

#if !defined(MIRRORLINK_OSREMOTE)
// Intern program to store program data sent by remote
ProgramStruct mirrorlinkProg;
#endif

extern ProgramData pd;
extern OpenSprinkler os;
extern char tmp_buffer[];

// SX1262 has the following connections:
// NSS pin:   18
// DIO1 pin:  33
// DIO2 pin:  32
// BUSY pin:  26
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_DIO2, LORA_BUSY);

typedef union {
  uint16_t data;
  struct {
    uint16_t mirrorlinkState : 3;      // Operation mode of the MirrorLink system
    uint16_t receivedFlag : 1;         // Flag to indicate that a packet was received
    uint16_t transmittedFlag : 1;      // Flag to indicate that a packet was sent
    uint16_t enableInterrupt : 1;      // Disable interrupt when it's not needed
    uint16_t associated : 1;           // Shows if OS device is associated
    uint16_t flagRxTx : 1;             // Flag to indicate if module is receiving or transmitting
    uint16_t rebootRequest : 1;        // Reboot request
    uint16_t stayAlive : 1;            // Bit to indicate if the stayalive feature is active
    uint16_t free : 6;                 // Free bits
  };
} MirrorLinkStateBitfield;

struct MIRRORLINK {
  time_t sendTimer;                         // Timer in seconds to control send timing
  int16_t moduleState;                      // LORA module state
  MirrorLinkStateBitfield status;           // Bittfield including states as well as several flags
  time_t stayAliveTimer;                    // Timer in seconds to control if remote and station are still connected
  time_t stayAliveMaxPeriod;                // Maximum period in seconds configured to control if remote and station are still connected
#if defined(MIRRORLINK_OSREMOTE)
  uint8_t bufferedCommands;                 // Number of buffered commands to be sent
  uint32_t buffer[MIRRORLINK_BUFFERLENGTH]; // Buffer for queued commands to be sent to station
  uint32_t response;                        // Response from the station to the last command sent
#else
  uint32_t command;                         // Command to be executed by the station
  int32_t latitude;                         // Latitude of the remote station
  int32_t longitude;                        // Longitude of the remote station
#endif //MIRRORLINK_OSREMOTE
} MirrorLink;

void schedule_all_stations(ulong curr_time);
void schedule_test_station(byte sid, uint16_t duration);
void change_program_data(int32_t pid, byte numprograms, ProgramStruct *prog);
void delete_program_data(int32_t pid);

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
    MirrorLink.status.receivedFlag = (uint16_t)true;
  }
  else {
    MirrorLink.status.transmittedFlag = (uint16_t)true;
  }
}

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
    case ML_PROGRAMADDDEL:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 = Add (1) or delete (0)
      // bit 8 to 16 = Not used
      // bit 27 to 31 = cmd
    case ML_PROGRAMMAINSETUP:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 = enable/disable
      // bit 8 = use weather
      // bit 9 to 10 = Odd/even restriction
      // bit 11 to 12 = schedule type
      // bit 13 to 26 = Not used
      // bit 27 to 31 = cmd
    case ML_PROGRAMDAYS:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 22 = days
      // bit 23 to 26 = Not used
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
    case ML_TIMESYNC:
      // Buffer message format:
      // bit 0 to 26 = Unix Timestamp in minutes! not seconds
		  // bit 27 to 31 = cmd
    case ML_TIMEZONESYNC:
      // Buffer message format:
      // bit 0 to 7 = Time zone
      // bit 27 to 31 = cmd
    case ML_CURRENTREQUEST:
      // TODO:
    case ML_EMERGENCYSHUTDOWN:
    case ML_STATIONREBOOT:
      // Reboot:
    case ML_LATITUDE:
      // Buffer message format:
      // bit 0 to 23 = latitude
		  // bit 24 to 26 = Not used
		  // bit 27 to 31 = cmd
    case ML_LONGITUDE:
      // Buffer message format:
      // bit 0 to 23 = longitude
		  // bit 24 to 26 = Not used
		  // bit 27 to 31 = cmd
    case ML_SUNRISE:      
      // TODO:
    case ML_SUNSET:
      // TODO:
    case ML_RAINDELAYSTOPTIME:
      // TODO:
    case ML_STAYALIVE:
      // Payload format: 
      // bit 0 to 26 = Stayalive configured counter
      // bit 27 to 31 = cmd
    case ML_APC:
      // TODO:
    case ML_CHANNEL:
      // TODO:
      if (MirrorLink.bufferedCommands < MIRRORLINK_BUFFERLENGTH) {
        MirrorLink.bufferedCommands++;
        MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)] = (((uint32_t)(0x1F & (uint16_t)cmd) << 27) | payload);
      }
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

#if defined(MIRRORLINK_OSREMOTE)
void MirrorLinkPeriodicCommands(void) {
  // Send regular commands (slow)
  if ((os.now_tz() % (time_t)MIRRORLINK_REGCOMMANDS_SLOW_PERIOD) == 0) {
    
    // Send ML_TIMESYNC
		// bit 0 to 26 = Unix Timestamp in minutes! not seconds
		// bit 27 to 31 = cmd
    MirrorLinkBuffCmd((uint8_t)ML_TIMESYNC, (uint32_t)(0x7FFFFFF & (RTC.get() / 60)));
    
    // Send ML_TIMEZONESYNC
    // Payload format:
    // bit 0 to 7 = Time zone
    // bit 27 to 31 = cmd
    MirrorLinkBuffCmd((uint8_t)ML_TIMEZONESYNC, (uint32_t)(0xFF & (os.iopts[IOPT_TIMEZONE])));
    
    // Send ML_LATITUDE and ML_LONGITUDE
    os.sopt_load(SOPT_LOCATION);
    char * latitude = strtok((char *)tmp_buffer, ",");
		char * longitude = strtok(NULL, ",");
    // Encode latitude and longitude in 23 bit ints
    const float weight = 180./(1 << 23);
		int32_t fp_lat = (int) (0.5f + atof(latitude) / weight);
		int32_t fp_lon = (int) (0.5f + atof(longitude) / weight);

		// Payload format: 
		// bit 0 to 23 = latitude
		// bit 24 to 26 = Not used
		// bit 27 to 31 = cmd
		MirrorLinkBuffCmd((uint8_t)ML_LATITUDE, (uint32_t)(0xFFFFFF & fp_lat));

		// Payload format: 
		// bit 0 to 23 = longitude
		// bit 24 to 26 = Not used
		// bit 27 to 31 = cmd
		MirrorLinkBuffCmd((uint8_t)ML_LONGITUDE, (uint32_t)(0xFFFFFF & fp_lon));  
  }
  // Send regular commands (mid)
  if ((os.now_tz() % (time_t)MIRRORLINK_REGCOMMANDS_MID_PERIOD) == 0) {
    // Send ML_SUNRISE
    // Send ML_SUNSET
  }
  // Send regular commands (fast)
  if ((os.now_tz() % (time_t)MIRRORLINK_REGCOMMANDS_FAST_PERIOD) == 0) {
    // Send ML_STAYALIVE
    // bit 0 to 25 = Stayalive configured counter
    // bit 26 = Bit indicating if stayalive feature shall be used (true) or ignored (false)
    // bit 27 to 31 = cmd
    MirrorLinkBuffCmd((uint8_t)ML_STAYALIVE, (uint32_t)((((uint32_t)1) << 26) | (0x3FFFFFF & MIRRORLINK_STAYALIVE_PERIOD)));
  }
}
#endif //defined(MIRRORLINK_OSREMOTE)

// MirrorLink module initialization
void MirrorLinkInit(void) {
  MirrorLink.moduleState = ERR_NONE;
  MirrorLink.status.receivedFlag = (uint16_t)false;
  MirrorLink.status.transmittedFlag = (uint16_t)false;
  MirrorLink.status.enableInterrupt = (uint16_t)true;
  MirrorLink.status.mirrorlinkState = MIRRORLINK_INIT;
  MirrorLink.status.flagRxTx = ML_RECEIVING;
  MirrorLink.status.rebootRequest = (uint16_t)false;
  MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
  MirrorLink.status.stayAlive = (uint16_t)true;
  MirrorLink.stayAliveMaxPeriod = (time_t)MIRRORLINK_STAYALIVE_PERIOD;
  MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
#if defined(MIRRORLINK_OSREMOTE)
  MirrorLink.bufferedCommands = 0;
  for (uint8_t i = 0; i < MIRRORLINK_BUFFERLENGTH; i++) MirrorLink.buffer[i] = 0;
#else
  // Intern program to store program data sent by remote
  mirrorlinkProg.enabled = 0;
  mirrorlinkProg.use_weather = 0;
  mirrorlinkProg.oddeven = 0;
  mirrorlinkProg.type = 0;
  mirrorlinkProg.starttime_type = 0;
  mirrorlinkProg.dummy1 = 0;
  mirrorlinkProg.days[0] = 0;
  mirrorlinkProg.days[1] = 0;
  for (uint8_t i = 0; i < MAX_NUM_STARTTIMES; i++) mirrorlinkProg.starttimes[i] = 0;
  for (uint8_t i = 0; i < MAX_NUM_STATIONS; i++) mirrorlinkProg.durations[i] = 0;
  sprintf_P(mirrorlinkProg.name, "%d", 0);
#endif // defined(MIRRORLINK_OSREMOTE)

  // TODO: Change this with flash check of associated adress:
  MirrorLink.status.associated = (uint16_t)true;

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
	MirrorLink.moduleState = lora.begin(866.2, 125.0, 12, 5, 0x1424, 22, 8, (float)(1.8), true);
	if (MirrorLink.moduleState == ERR_NONE) {
    Serial.println(F("success!"));
	} else {
		Serial.print(F("failed, code "));
		Serial.println(MirrorLink.moduleState);
	}

  // Current limitation
  MirrorLink.moduleState = lora.setCurrentLimit(120.0);
	if (MirrorLink.moduleState == ERR_INVALID_CURRENT_LIMIT) {
    Serial.println(F("Current limit configure exceedes max.!"));
	}

#if defined(MIRRORLINK_MODRADIOLIB)
  // Set SX126x_REG_RX_GAIN to 0x96 -> LNA +3dB gain   SX126xWriteRegister( SX126x_REG_RX_GAIN, 0x96 );
  uint16_t modReg = 0x08AC; //SX126X_REG_RX_GAIN
  uint8_t modData[1] = { 0x96 };
  MirrorLink.moduleState = lora.writeRegister(modReg, modData, 1); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
	if (MirrorLink.moduleState != ERR_NONE) {
    Serial.println(F("LNA max gain not set successfully!"));
	}
#endif //defined(MIRRORLINK_MODRADIOLIB)

	// Serial.print(F("[SX1262] Activating LDRO ... "));
	// // Activate automatic LDRO optimization for long symbol duration SX1262 
	// MirrorLink.moduleState = lora.autoLDRO();
	// if (MirrorLink.moduleState == ERR_NONE) {
  //   Serial.println(F("success!"));
	// } else {
	// 	Serial.print(F("failed, code "));
	// 	Serial.println(MirrorLink.moduleState);
	// }

  // Serial.print(F("[SX1262] Setting TCXO ... "));
	// // Activate automatic LDRO optimization for long symbol duration SX1262 
	// MirrorLink.moduleState = lora.setTCXO(float(1.8), 5000);
	// if (MirrorLink.moduleState == ERR_NONE) {
  //   Serial.println(F("success!"));
	// } else {
	// 	Serial.print(F("failed, code "));
	// 	Serial.println(MirrorLink.moduleState);
	// }

	// eByte E22-900M uses DIO3 to supply the external TCXO
	//if (lora.setTCXO(2.4) == ERR_INVALID_TCXO_VOLTAGE)
  // if (lora.setTCXO(1.8) == ERR_INVALID_TCXO_VOLTAGE)
	// {
	// 	Serial.println(F("Selected TCXO voltage is invalid for this module!"));
	// }

  // Set PA config
  // if (lora.setPaConfig(0x04, 16) == ERR_INVALID_TCXO_VOLTAGE)
	// {
	// 	Serial.println(F("PA configuration is invalid for this module!"));
	// }

	// Set the function that will be called
	// when new packet is transmitted or received
  lora.setDio1Action(setFlag);
}

bool MirrorLinkTransmitStatus(void) {
  bool txSuccessful = false;
  if (MirrorLink.status.transmittedFlag == (uint8_t)true) {
    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint16_t)false;
    MirrorLink.status.transmittedFlag = (uint16_t)false;

    if (MirrorLink.moduleState == ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
      txSuccessful = true;
    } 
    else {
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.moduleState);
    }
    MirrorLink.status.enableInterrupt = (uint16_t)true;
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
    MirrorLink.moduleState = lora.startTransmit(byteArr, 8);
  */
  byte byteArr[4] = {(byte)(0xFF & MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)] >> 24) , (byte)(0xFF & MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)] >> 16) , (byte)(0xFF & MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)] >> 8) , (byte)(0xFF & MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)])};
  MirrorLink.moduleState = lora.startTransmit(byteArr, 4);
#else
  // TODO: Transmit answer to command
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    MirrorLink.moduleState = lora.startTransmit(byteArr, 8);
  */
  byte byteArr[4] = {(byte)(0xFF & MirrorLink.command >> 24) , (byte)(0xFF & MirrorLink.command >> 16) , (byte)(0xFF & MirrorLink.command >> 8) , (byte)(0xFF & MirrorLink.command)};
  MirrorLink.moduleState = lora.startTransmit(byteArr, 4);
  MirrorLink.command = 0;
#endif // defined(MIRRORLINK_OSREMOTE)
  MirrorLink.status.flagRxTx = ML_TRANSMITTING;
}

bool MirrorLinkReceiveStatus(void) {
  bool rxSuccessful = false;
  // check if the flag is set
  if(MirrorLink.status.receivedFlag) {

    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint16_t)false;

    // reset flag
    MirrorLink.status.receivedFlag = (uint16_t)false;

    // Read received data as byte array
    byte byteArr[4];
    MirrorLink.moduleState = lora.readData(byteArr, 4);

    if (MirrorLink.moduleState == ERR_NONE) {

#if defined(MIRRORLINK_OSREMOTE)
      MirrorLink.response = (((uint32_t)byteArr[0] << 24) | ((uint32_t)byteArr[1] << 16) | ((uint32_t)byteArr[2] << 8) | ((uint32_t)byteArr[3]));
#else
      MirrorLink.command = (((uint32_t)byteArr[0] << 24) | ((uint32_t)byteArr[1] << 16) | ((uint32_t)byteArr[2] << 8) | ((uint32_t)byteArr[3]));
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
    } else if (MirrorLink.moduleState == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.moduleState);
    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    MirrorLink.status.enableInterrupt = (uint16_t)true;
  }
  return rxSuccessful;
}

void MirrorLinkReceiveInit(void) {
  // Important! To enable receive you need to switch the SX126x antenna switch to RECEIVE 
  enableRX();

  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  MirrorLink.moduleState = lora.startReceive();
  if (MirrorLink.moduleState == ERR_NONE) {
    Serial.println(F("success!"));
    MirrorLink.status.flagRxTx = ML_RECEIVING;
  } else {
    Serial.print(F("failed, code "));
    Serial.println(MirrorLink.moduleState);
  }
}

#if !defined(MIRRORLINK_OSREMOTE)
// MirrorLink function to control the actions related to the stayalive counter
void MirrorLinkStayAliveControl(void) {
  // If stayalive control is active
  if (MirrorLink.status.stayAlive) {
    // If system currently enabled
    if (os.status.enabled) {
      // If timer reached
      if (os.now_tz() > MirrorLink.stayAliveTimer) {
        // Disable system
        os.disable();
      }
    }
    else {
      // If timer not reached
      if (os.now_tz() < MirrorLink.stayAliveTimer) {
        // Enable system
        os.enable();
      }
    }
  }
}
#endif

// MirrorLink module state machine change function
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
      MirrorLink.sendTimer = os.now_tz();
      if (  (MirrorLink.sendTimer == os.now_tz())
          &&(MirrorLink.bufferedCommands > 0)) {
        Serial.println(F("STATE: MIRRORLINK_SEND"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        MirrorLinkTransmit();
      }
      break;
#endif // defined(MIRRORLINK_OSREMOTE)
    // Send state
    case MIRRORLINK_SEND:
      // If send process if finished
      // empty the command in the buffer
      // change state to receive
#if defined(MIRRORLINK_OSREMOTE)
      if (MirrorLinkTransmitStatus() == true) {
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
      else if (MirrorLink.sendTimer <= os.now_tz()) {
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        MirrorLinkReceiveInit();
      }
#else
      if (   (MirrorLinkTransmitStatus() == true)
          || (MirrorLink.sendTimer <= os.now_tz())) {
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
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
      // change state to Buffering
      if (MirrorLinkReceiveStatus() == true) {
        // In case response shows a sync error between remote and station
        // delete all programs and reset response
        if ((MirrorLink.response >> 27) == ML_SYNCERROR) {
          delete_program_data(-1);
          Serial.println(F("Sync error with remote, reset program data!"));
        }
        
        // If response different than last command sent then report error
        if (MirrorLink.response != MirrorLink.buffer[(MirrorLink.bufferedCommands - 1)]) {
          Serial.println(F("Station response does not match command sent!"));
        }

        // Report command sent
        if (MirrorLink.bufferedCommands > 0) MirrorLink.bufferedCommands--;
        
        MirrorLink.response = 0;
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
      }

      // If timeout
      // Report error and change state as well to Buffering
      if(MirrorLink.sendTimer <= os.now_tz()) {

        // Report error
        Serial.println(F("No answer received from station!"));

        // Command is lost, do not retry
        if (MirrorLink.bufferedCommands > 0) MirrorLink.bufferedCommands--;
        
        MirrorLink.response = 0;
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
      }   
#else
      // If commands received
      // execute and change state to Send
      if (MirrorLinkReceiveStatus() == true) {
        uint32_t payload = 0;
        byte sid = 0;
        int16_t pid = 0;
        uint8_t en = 0;
        uint16_t timer = 0;
        uint8_t stTimeNum = 0;
        uint16_t duration = 0;
        uint8_t usw = 0;
        uint8_t oddeven = 0;
        uint8_t addProg = 0;
        char * latitude;
        char * longitude;
        const float weight = 180./(1 << 23);
        if (MirrorLink.command != 0) {
          // Execute command
          switch (MirrorLink.command >> 27) {
            // Initial state
            case ML_TESTSTATION:
              // Message format: 
              // bit 0 = status (1 = On, 0 = Off)
              // bit 1 to 8 = sid
              // bit 9 to 24 = time(sec)
              // bit 25 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_TESTSTATION);
              sid = (byte) (0xFF & (payload >> 1));
              en = (uint8_t) (payload & 0x1);
              timer = (uint16_t) ((0xFFFF) & (payload >> 9));     
              schedule_test_station(sid, timer);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_PROGRAMADDDEL:
              // Message format: 
        			// bit 0 to 6 = program number (max. is 40)
	          	// bit 7 = Add (1) or delete (0)
              // bit 8 to 16 = Not used
			        // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMADDDEL);
              addProg = ((payload >> 7) & 0x1);
              pid = (int16_t) (0x7F & payload);
              sprintf_P(mirrorlinkProg.name, "%d", pid);

              // If the request is to add a program
              if (addProg)
              {
                // In case the new pid does not match the max. program number
                // SYNC issue identified, remove all programs
                if (pid != pd.nprograms) {
                  // Delete all programs
                  delete_program_data(-1);
                  MirrorLink.command = (((uint32_t)ML_SYNCERROR) << 27);
                }
                // Otherwise create new program
                else {
                  change_program_data(pid, pd.nprograms, &mirrorlinkProg);
                }
              }
              // Request is to delete a program
              else
              {
                // In case the new pid to be removed is not within the available pid's range
                // SYNC issue identified, remove all programs
                if (pid >= pd.nprograms) {
                  // Delete all programs
                  delete_program_data(-1);
                  MirrorLink.command = (((uint32_t)ML_SYNCERROR) << 27);
                }
                // Otherwise delete the program
                else {
                  delete_program_data(pid);
                }
              }
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_PROGRAMMAINSETUP:
              // Message format:
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 = enable/disable
              // bit 8 = use weather
              // bit 9 to 10 = Odd/even restriction
              // bit 11 to 12 = schedule type
              // bit 13 to 20 = Number of programs in remote
              // bit 21 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMMAINSETUP);
              pid = (int16_t) (0x7F & payload);
              mirrorlinkProg.enabled = (uint8_t)(0x1 & (payload >> 7));
              mirrorlinkProg.use_weather = (uint8_t) (0x1 & (payload >> 8));
              mirrorlinkProg.oddeven = (uint8_t) (0x3 & (payload >> 9));
              sprintf_P(mirrorlinkProg.name, "%d", pid);
              change_program_data(pid, pd.nprograms, &mirrorlinkProg);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_PROGRAMDAYS:
              // Message format:
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 to 22 = days
              // bit 23 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMDAYS);
              pid = (int16_t) (0x7F & payload);
              mirrorlinkProg.days[0] = (byte) (0xFF & (payload >> 15));
              mirrorlinkProg.days[1] = (byte) (0xFF & (payload >> 7));
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;            
            case ML_PROGRAMSTARTTIME:
              // Message format:
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 to 8 = start time number (max. is 4 for each program)
              // bit 9 to 24 = start time
              // bit 25 = Starttime type
              // bit 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMSTARTTIME);
              pid = (int16_t)(payload & 0x7F);
              stTimeNum = (uint8_t)((payload >> 7) & 0x3);
              mirrorlinkProg.starttimes[stTimeNum] = (int16_t)((payload >> 9) & 0xFFFF);
              mirrorlinkProg.starttime_type = (uint8_t)((payload >> 25) & 0x1);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_PROGRAMDURATION:
              // Message format:
              // bit 0 to 6 = program number (max. is 40)
              // bit 7 to 14 = sid
              // bit 15 to 25 = time (min)
              // bit 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_PROGRAMDURATION);
              pid = (int16_t)(payload & 0x7F);
              sid = (byte)((payload >> 7) & 0xFF);
              mirrorlinkProg.durations[sid] = (uint16_t)(60 * ((payload >> 15) & 0x7FF));
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_TIMESYNC:
              // Message format:
              // bit 0 to 26 = Unix Timestamp in minutes! not seconds
		          // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_TIMESYNC);
              setTime((time_t)(60*(0x7FFFFFF & payload)));
              RTC.set((time_t)(60*(0x7FFFFFF & payload)));
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_TIMEZONESYNC:
            	// Payload format:
		          // bit 0 to 7 = Time zone
              // bit 8 to 26 = Not used
		          // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_TIMEZONESYNC);
              os.iopts[IOPT_TIMEZONE] = (byte)(0xFF & payload);
              os.iopts[IOPT_USE_NTP] = 0;
              os.iopts_save();
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_CURRENTREQUEST:
              // TODO:
              payload = MirrorLinkGetCmd((uint8_t)ML_CURRENTREQUEST);
            case ML_EMERGENCYSHUTDOWN:
              // TODO:
              payload = MirrorLinkGetCmd((uint8_t)ML_EMERGENCYSHUTDOWN);
              break;
            case ML_STATIONREBOOT:
            	// Payload format:
		          // bit 0 to 26 = Not used
		          // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_STATIONREBOOT);
              MirrorLink.status.rebootRequest = (uint16_t)true;
              os.reboot_dev(REBOOT_CAUSE_MIRRORLINK);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_LATITUDE:
              // Payload format: 
              // bit 0 to 23 = latitude
              // bit 24 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_LATITUDE);
              MirrorLink.latitude = (int32_t)(0xFFFFFF & payload);
              sprintf_P(tmp_buffer, PSTR("%f,%f"), (float) (weight * ((float)MirrorLink.latitude- 0.5f)), (float) (weight * ((float)MirrorLink.longitude- 0.5f)));
              os.sopt_save(SOPT_LOCATION, tmp_buffer);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_LONGITUDE:
              // Payload format: 
              // bit 0 to 23 = longitude
              // bit 24 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_LONGITUDE);
              MirrorLink.longitude = (int32_t)(0xFFFFFF & payload);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_SUNRISE:
              // TODO:
              payload = MirrorLinkGetCmd((uint8_t)ML_SUNRISE);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_SUNSET:
              // TODO:
              payload = MirrorLinkGetCmd((uint8_t)ML_SUNSET);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_RAINDELAYSTOPTIME:
              // TODO:
              payload = MirrorLinkGetCmd((uint8_t)ML_RAINDELAYSTOPTIME);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_STAYALIVE:
              // Payload format: 
              // bit 0 to 25 = Stayalive configured counter
              // bit 26 = Bit indicating if stayalive feature shall be used (true) or ignored (false)
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_STAYALIVE);
              MirrorLink.status.stayAlive = (uint8_t)((payload >> 26) & 0x01);
              MirrorLink.stayAliveMaxPeriod = (time_t)(payload & 0x3FFFFFF);
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_APC:
              // TODO:
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
            case ML_CHANNEL:
              // TODO:
              MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
              break;
          }
        }
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("SATE: MIRRORLINK_SEND"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
        // Delay to allow the remote to turn to rx mode
        delay(100);
      }
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
  }
}

// MirrorLink module state maction actions function, called once every second
void MirrorLinkWork(void) {
  switch (MirrorLink.status.mirrorlinkState) {
    // Initial state
    case MIRRORLINK_INIT:
      // Do nothing
#if !defined(MIRRORLINK_OSREMOTE)
      MirrorLinkStayAliveControl();
#endif
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
      MirrorLinkStayAliveControl();
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
#if defined(MIRRORLINK_OSREMOTE)
    // Buffering state
    case MIRRORLINK_BUFFERING:
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
      if (   (MirrorLink.sendTimer == (os.now_tz() + (time_t)(MIRRORLINK_RXTX_MAX_TIME - MIRRORLINK_RXTX_DEAD_TIME)))
          && (MirrorLink.status.flagRxTx == ML_RECEIVING)) {
        MirrorLinkTransmit();
      }
      MirrorLinkStayAliveControl();
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
    // Receive state
    case MIRRORLINK_RECEIVE:
#if !defined(MIRRORLINK_OSREMOTE)
      MirrorLinkStayAliveControl();
#endif
      break;
  }
}

// MirrorLink module function, called once every second
void MirrorLinkMain(void) {
  // State changes
  MirrorLinkState();

  // State actions
  MirrorLinkWork();
}

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)