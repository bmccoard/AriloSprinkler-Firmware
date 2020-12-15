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

// Define buffers: need them to be sufficiently large to cover string option reading

#if !defined(MIRRORLINK_OSREMOTE)
// Intern program to store program data sent by remote
ProgramStruct mirrorlinkProg;
#endif

// Speck key expansion buffer
SPECK_TYPE mirrorLinkSpeckKeyExp[SPECK_ROUNDS];

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
  uint32_t data;
  struct {
    uint32_t mirrorlinkState : 3;      // Operation mode of the MirrorLink system
    uint32_t networkId : 8;            // Network ID for the Link
    uint32_t receivedFlag : 1;         // Flag to indicate that a packet was received
    uint32_t transmittedFlag : 1;      // Flag to indicate that a packet was sent
    uint32_t enableInterrupt : 1;      // Disable interrupt when it's not needed
    uint32_t flagRxTx : 1;             // Flag to indicate if module is receiving or transmitting
    uint32_t rebootRequest : 1;        // Reboot request
    uint32_t stayAlive : 1;            // Bit to indicate if the stayalive feature is active
    uint32_t link : 1;                 // Shows if link is up or down
    uint32_t comStatus : 2;            // Shows the link communication status
    uint32_t powerLevel : 4;           // Power Level for the link
    uint32_t channelNumber : 4;        // Channel number for the link
    uint32_t free : 4;                 // Free bits
  };
} MirrorLinkStateBitfield;

struct MIRRORLINK {
  uint32_t key[4];                          // Encryption key for the Link
  uint32_t associationKey[2];               // Association key for the Link used to generate new key
  time_t sendTimer;                         // Timer in seconds to control send timing
  int16_t moduleState;                      // LORA module state
  MirrorLinkStateBitfield status;           // Bittfield including states as well as several flags
  time_t stayAliveTimer;                    // Timer in seconds to control if remote and station are still connected
  time_t stayAliveMaxPeriod;                // Maximum period in seconds configured to control if remote and station are still connected
  float frequency;                          // Frequency in use
  uint32_t packetsSent;                     // Number of sent packets
  uint32_t packetsReceived;                 // Number of received packets
  uint16_t packetExchCtr;                   // Counter to control sequence of packets exchanged in the link
  uint16_t associationAttempts;             // Counter to control the number of association attempts
  uint16_t snrLocal;                        // Local SNR (local reception)
  int16_t rssiLocal;                        // Local RSSI (local reception)
#if defined(MIRRORLINK_OSREMOTE)
  uint16_t snrRemote;                       // Remote SNR (remote reception)
  int16_t rssiRemote;                       // Remote RSSI (remote reception)
  uint32_t buffer[MIRRORLINK_BUFFERLENGTH]; // Buffer for queued commands to be sent to station
  uint32_t response;                        // Response from the station to the last command sent
  uint32_t txTime;                          // Time used to transmit a payload in milliseconds
  uint8_t bufferedCommands;                 // Number of buffered commands to be sent
  uint8_t indexBufferHead;                  // Index of the first element in the command buffer
  uint8_t indexBufferTail;                  // Index of the last element in the command buffer     
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

// Speck64/128 encryption based on Moritz Bitsch implementation
// Speck encryption expand function
void speck_expand(SPECK_TYPE const K[SPECK_KEY_LEN], SPECK_TYPE S[SPECK_ROUNDS])
{
  SPECK_TYPE i, b = K[0];
  SPECK_TYPE a[SPECK_KEY_LEN - 1];

  for (i = 0; i < (SPECK_KEY_LEN - 1); i++)
  {
    a[i] = K[i + 1];
  }
  S[0] = b;  
  for (i = 0; i < SPECK_ROUNDS - 1; i++) {
    R(a[i % (SPECK_KEY_LEN - 1)], b, i);
    S[i + 1] = b;
  }
}

// Speck encryption function
void speck_encrypt(SPECK_TYPE const pt[2], SPECK_TYPE ct[2], SPECK_TYPE const K[SPECK_ROUNDS])
{
  SPECK_TYPE i;
  ct[0]=pt[0]; ct[1]=pt[1];

  for(i = 0; i < SPECK_ROUNDS; i++){
    R(ct[1], ct[0], K[i]);
  }
}

// Speck decryption function
void speck_decrypt(SPECK_TYPE const ct[2], SPECK_TYPE pt[2], SPECK_TYPE const K[SPECK_ROUNDS])
{
  SPECK_TYPE i;
  pt[0]=ct[0]; pt[1]=ct[1];

  for(i = 0; i < SPECK_ROUNDS; i++){
    RR(pt[1], pt[0], K[(SPECK_ROUNDS - 1) - i]);
  }
}

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
void ICACHE_RAM_ATTR setFlag(void) {
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
  if (MirrorLink.status.mirrorlinkState != MIRRORLINK_ASSOCIATE) {
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
          MirrorLink.buffer[MirrorLink.indexBufferHead] = (((uint32_t)(0x1F & (uint16_t)cmd) << 27) | payload);
          MirrorLink.indexBufferHead = (MirrorLink.indexBufferHead + 1) % MIRRORLINK_BUFFERLENGTH;
        }
        break;
    }
  }
}
#else
uint32_t MirrorLinkGetCmd(uint8_t cmd)
{
  uint32_t payload = 0;
  if(cmd == (MirrorLink.command >> 27)) {
    payload = (MirrorLink.command & 0x7FFFFFF);
  }
  // Empty command but cmd part to send answer to remote station
  MirrorLink.command &= 0xF8000000;
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
		// bit 0 to 23 = longitude
		// bit 24 to 26 = Not used
		// bit 27 to 31 = cmd

    // Encode negative longitude
    if (fp_lon < 0) {
      fp_lon *= -1;
      fp_lon |= 0x1000000;
    }
		MirrorLinkBuffCmd((uint8_t)ML_LONGITUDE, (uint32_t)(0x1FFFFFF & fp_lon));

		// Payload format: 
		// bit 0 to 24 = latitude (bit 24 for sign)
		// bit 25 to 26 = Not used
		// bit 27 to 31 = cmd

    // Encode negative latitude
    if (fp_lat < 0) {
      fp_lat *= -1;
      fp_lat |= 0x1000000;
    }
		MirrorLinkBuffCmd((uint8_t)ML_LATITUDE, (uint32_t)(0x1FFFFFF & fp_lat));
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

// Status MirrorLink for wifi server
String MirrorLinkStatus() {
	String mirrorLinkInfo;
	// Encode in JSON message
	mirrorLinkInfo = "{\"frequency\":["; 
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += String(MirrorLink.frequency);
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"rssis\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(MirrorLink.rssiLocal);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += ",\r\n";
  mirrorLinkInfo += "\"";
  #if defined(MIRRORLINK_OSREMOTE)
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(MirrorLink.rssiRemote);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  #else
  mirrorLinkInfo += "N.A.";
  #endif // defined(MIRRORLINK_OSREMOTE)
  mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"snrs\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(((float)MirrorLink.snrLocal) / 10);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += ",\r\n";
  mirrorLinkInfo += "\"";
  #if defined(MIRRORLINK_OSREMOTE)
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(((float)MirrorLink.snrRemote) / 10);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  #else
  mirrorLinkInfo += "N.A.";
  #endif // defined(MIRRORLINK_OSREMOTE)
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"linkst\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += "UP";
  }
  else {
    mirrorLinkInfo += "DOWN";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"assocst\":[";
  mirrorLinkInfo += "\"";
  switch (MirrorLink.status.comStatus) {
    case ML_LINK_COM_ASSOCIATION:
      mirrorLinkInfo += "ASSOCIATING";
      break;
    case ML_LINK_COM_CHANGEKEY:
      mirrorLinkInfo += "CHANGING KEY";
      break;
    case ML_LINK_COM_NORMAL:
      mirrorLinkInfo += "ASSOCIATED";
      break;
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"assocatm\":[";
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += String(MirrorLink.associationAttempts);
  mirrorLinkInfo += "\"";
	mirrorLinkInfo += "]}";
	return mirrorLinkInfo;
}

// MirrorLink module initialization
void MirrorLinkInit(void) {
  MirrorLink.moduleState = ERR_NONE;
  MirrorLink.status.networkId = (uint32_t)MIRRORLINK_NETWORK_ID;
  MirrorLink.status.receivedFlag = (uint32_t)false;
  MirrorLink.status.transmittedFlag = (uint32_t)false;
  MirrorLink.status.enableInterrupt = (uint32_t)true;
  MirrorLink.status.mirrorlinkState = (uint32_t)MIRRORLINK_INIT;
  MirrorLink.status.flagRxTx = (uint32_t)ML_RECEIVING;
  MirrorLink.status.comStatus = (uint32_t)ML_LINK_COM_ASSOCIATION;
  MirrorLink.status.rebootRequest = (uint32_t)false;
  MirrorLink.status.stayAlive = (uint32_t)true;
  MirrorLink.status.powerLevel = (uint32_t)ML_TX_POWER;
  MirrorLink.status.channelNumber = (uint32_t)ML_CH_0;
  MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
  MirrorLink.stayAliveMaxPeriod = (time_t)MIRRORLINK_STAYALIVE_PERIOD;
  MirrorLink.stayAliveTimer = os.now_tz() + MirrorLink.stayAliveMaxPeriod;
  MirrorLink.status.link = ML_LINK_DOWN;
  MirrorLink.snrLocal = 0;
  MirrorLink.rssiLocal = -200;
  MirrorLink.frequency = (float)ML_FREQUENCY;
  MirrorLink.key[0] = SPECK_DEFAULT_KEY_N1;
  MirrorLink.key[1] = SPECK_DEFAULT_KEY_N2;
  MirrorLink.key[2] = SPECK_DEFAULT_KEY_N3;
  MirrorLink.key[3] = SPECK_DEFAULT_KEY_N4;
  speck_expand(MirrorLink.key, mirrorLinkSpeckKeyExp);
  MirrorLink.packetsSent = 0;
  MirrorLink.packetsReceived = 0;
  MirrorLink.associationAttempts = 0;
#if defined(MIRRORLINK_OSREMOTE)
  MirrorLink.bufferedCommands = 0;
  for (uint8_t i = 0; i < MIRRORLINK_BUFFERLENGTH; i++) MirrorLink.buffer[i] = 0;
  MirrorLink.indexBufferHead = 0;
  MirrorLink.indexBufferTail = 0;
  MirrorLink.snrRemote = 0;
  MirrorLink.rssiRemote = -200;
  MirrorLink.txTime = 0;
  MirrorLink.packetExchCtr = 0;
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
  MirrorLink.packetExchCtr = 0;
#endif // defined(MIRRORLINK_OSREMOTE)

	Serial.begin(115200);

	// Configure LORA module pins
	pinMode(LORA_SCLK, OUTPUT); // SCLK
	pinMode(LORA_MISO, INPUT);  // MISO
	pinMode(LORA_MOSI, OUTPUT); // MOSI
	pinMode(LORA_NSS, OUTPUT);  // CS
	pinMode(LORA_DIO1, INPUT);  // DIO1
	pinMode(LORA_DIO2, INPUT);  // DIO2
	pinMode(LORA_BUSY, INPUT);  // BUSY
  pinMode(LORA_RXEN, OUTPUT); // RXEN
  pinMode(LORA_TXEN, OUTPUT); // TXEN

	// SCLK GPIO 5, MISO GPIO 19, MOSI GPIO 27, CS == NSS GPIO 18
	SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_NSS);
	Serial.print(F("[SX1262] Initializing ... "));
	// initialize SX1262 
	// carrier frequency:           868.0 MHz
	// bandwidth:                   125.0 kHz
	// spreading factor:            7
	// coding rate:                 5
	// sync word:                   0x1424 (private network)
	// output power:                16 dBm (30 dBm with E22-900T30S amplification)
	// current limit:               120 mA
	// preamble length:             8 symbols
	// CRC:                         enabled
	MirrorLink.moduleState = lora.begin(MirrorLink.frequency, 125.0, 12, 5, SX126X_SYNC_WORD_PRIVATE, 16 , 8, (float)(1.8), true);
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
  // Set SX126x_REG_RX_GAIN to 0x96 -> LNA +3dB gain SX126xWriteRegister( SX126x_REG_RX_GAIN, 0x96 );
  uint16_t modReg = 0x08AC; //SX126X_REG_RX_GAIN
  uint8_t modData[1] = { 0x96 };
  MirrorLink.moduleState = lora.writeRegister(modReg, modData, 1); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
	if (MirrorLink.moduleState != ERR_NONE) {
    Serial.println(F("LNA max gain not set successfully!"));
	}
#endif //defined(MIRRORLINK_MODRADIOLIB)

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

      // Increase transmission counter
      MirrorLink.packetsSent++;
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
  SPECK_TYPE plain[2] = {0, 0};
  SPECK_TYPE buffer[2] = {0, 0};

#if defined(MIRRORLINK_OSREMOTE)
  // Check communication status
  // If not associated
  // Reset to default key
  if (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_ASSOCIATION) {
    Serial.println(F(""));
    Serial.println(F("Resetting to default key"));
    Serial.println(F(""));
    MirrorLink.key[0] = SPECK_DEFAULT_KEY_N1;
    MirrorLink.key[1] = SPECK_DEFAULT_KEY_N2;
    MirrorLink.key[2] = SPECK_DEFAULT_KEY_N3;
    MirrorLink.key[3] = SPECK_DEFAULT_KEY_N4;
    speck_expand(MirrorLink.key, mirrorLinkSpeckKeyExp);
  }
#endif // defined(MIRRORLINK_OSREMOTE)

  // If not associated or change key process
  if (   (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_ASSOCIATION)
      || (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_CHANGEKEY) ) {
    MirrorLink.associationAttempts++;
#if defined(MIRRORLINK_OSREMOTE)
    // Initialized packetExchCtr with random number
    MirrorLink.packetExchCtr = random(16383);
    // Generate first part of future random key for the link
    MirrorLink.associationKey[0] = (uint32_t)random(INT32_MAX);
    plain[1] = MirrorLink.associationKey[0];
    MirrorLink.txTime = millis();
#else
    // Increase packetExchCtr value
    MirrorLink.packetExchCtr = ((MirrorLink.packetExchCtr + 1) % 16383);
    // Generate second part of future random key for the link
    MirrorLink.associationKey[1] = (uint32_t)random(INT32_MAX);
    plain[1] = MirrorLink.associationKey[1];
#endif // defined(MIRRORLINK_OSREMOTE)
  }
  // Else if mode is normal
  else {
    // Increase packetExchCtr value
    MirrorLink.packetExchCtr = ((MirrorLink.packetExchCtr + 1) % 16383);
#if defined(MIRRORLINK_OSREMOTE)
    plain[1] = MirrorLink.buffer[MirrorLink.indexBufferTail];
    MirrorLink.txTime = millis();
#else
    // Encode RSSI to 0.2 resolution in 8 bit (value range from -255dBm to 255dBm)
    // Encode SNR to 0.2 resolution in 8 bit (value range from 0dB to 51.1dB)
    uint8_t rssi;
    if (MirrorLink.rssiLocal > 0) {
      if ((MirrorLink.rssiLocal / 2) > 127) {
        rssi = (uint8_t)127;
      }
      else {
        rssi = (uint8_t)(MirrorLink.rssiLocal / 2);
      }
    } 
    else {
      if ((MirrorLink.rssiLocal / 2) < -127) {
        rssi = ((uint8_t)127 | 0x80);
      }
      else {
        rssi = (((uint8_t)((MirrorLink.rssiLocal / 2) * -1)) | 0x80);
      }
    }
    MirrorLink.command |= (uint32_t)((rssi) & 0xFF);
    MirrorLink.command |= ((uint32_t)((MirrorLink.snrLocal / 2) & 0xFF) << 8);
    plain[1] = (((((uint32_t)(MirrorLink.snrLocal / 2) & 0xFF)) << 24) | (((uint32_t)((rssi) & 0xFF)) << 16) | ((uint32_t)(MirrorLink.command >> 16)));
    MirrorLink.command = 0;
#endif // defined(MIRRORLINK_OSREMOTE)
  }
  plain[0] = ((((uint32_t)MirrorLink.status.networkId) << 24) | ((((uint32_t)MirrorLink.status.comStatus) & 0x3) << 22) | ((((uint32_t)MirrorLink.status.powerLevel) & 0xF) << 18) | ((((uint32_t)MirrorLink.status.channelNumber) & 0xF) << 14)  | ((((uint32_t)MirrorLink.packetExchCtr) & 0x3FFF)));

  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    MirrorLink.moduleState = lora.startTransmit(byteArr, 8);
  */
  // Encrypt message
  speck_encrypt(plain, buffer, mirrorLinkSpeckKeyExp);
  byte byteArr[8] = {(byte)(0xFF & buffer[0] >> 24), (byte)(0xFF & buffer[0] >> 16), (byte)(0xFF & buffer[0] >> 8), (byte)(0xFF & buffer[0]), (byte)(0xFF & buffer[1] >> 24) , (byte)(0xFF & buffer[1] >> 16) , (byte)(0xFF & buffer[1] >> 8) , (byte)(0xFF & buffer[1])};
  MirrorLink.moduleState = lora.startTransmit(byteArr, 8);
  MirrorLink.status.flagRxTx = ML_TRANSMITTING;

#if !defined(MIRRORLINK_OSREMOTE)
  // For station, if not associated or change key process, update key and com. to normal
  if (   (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_ASSOCIATION)
      || (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_CHANGEKEY) ) {
    Serial.println(F(""));
    Serial.println(F("Changing key"));
    Serial.println(F(""));
    MirrorLink.key[3] = MirrorLink.key[1];
    MirrorLink.key[2] = MirrorLink.key[0];
    MirrorLink.key[1] = MirrorLink.associationKey[1];
    MirrorLink.key[0] = MirrorLink.associationKey[0];
    speck_expand(MirrorLink.key, mirrorLinkSpeckKeyExp);
    // Change communication mode to normal
    MirrorLink.status.comStatus = ML_LINK_COM_NORMAL;

    Serial.println(F(""));
    Serial.print(F("Entering ML_LINK_COM_NORMAL mode "));
    Serial.println(F(""));
  }
#endif // defined(MIRRORLINK_OSREMOTE)
}

bool MirrorLinkReceiveStatus(void) {
  bool rxSuccessful = false;
  // check if the flag is set
  if(MirrorLink.status.receivedFlag) {
    // Increase reception counter
    MirrorLink.packetsReceived++;

    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint16_t)false;

    // Reset flag
    MirrorLink.status.receivedFlag = (uint16_t)false;

    // Read received data as byte array
    byte byteArr[8];
    MirrorLink.moduleState = lora.readData(byteArr, 8);

    // Message decryption
    SPECK_TYPE encoded[2] = {0, 0};
    SPECK_TYPE buffer[2] = {0, 0};
    encoded[0] = (((uint32_t)(byteArr[0]) << 24) | ((uint32_t)(byteArr[1]) << 16) | ((uint32_t)(byteArr[2]) << 8) | ((uint32_t)(byteArr[3])));
    encoded[1] = (((uint32_t)(byteArr[4]) << 24) | ((uint32_t)(byteArr[5]) << 16) | ((uint32_t)(byteArr[6]) << 8) | ((uint32_t)(byteArr[7])));
    speck_decrypt(encoded, buffer, mirrorLinkSpeckKeyExp);

    // Network ID match flag
    bool networkIdMatch = (bool)(MirrorLink.status.networkId == (0xFF & (buffer[0] >> 24)));

    // Packet Exchange Control Match flag
    bool packetExchCtrMatch = (bool)((buffer[0] & 0x3FFF) == (MirrorLink.packetExchCtr + 1));
    
    Serial.println(F(""));
    Serial.println(F("packetExchCtrMatch buffer vs packetExchCtr: "));
    Serial.println(buffer[0] & 0x3FFF);
    Serial.println(MirrorLink.packetExchCtr + 1);
    Serial.println(packetExchCtrMatch);
    Serial.println(F(""));

    // If reception and decryption successful
    if (  (MirrorLink.moduleState == ERR_NONE)
        &&(networkIdMatch)
        &&(packetExchCtrMatch) ) {
      // Increase packetExchCtr value
      MirrorLink.packetExchCtr = ((MirrorLink.packetExchCtr + 1) % 16383);
      MirrorLink.status.comStatus = ((buffer[0] & 0xC00000) >> 22);
#if defined(MIRRORLINK_OSREMOTE)
      if (  (MirrorLink.status.comStatus == ML_LINK_COM_ASSOCIATION) 
          ||(MirrorLink.status.comStatus == ML_LINK_COM_CHANGEKEY)) {

        Serial.println(F(""));
        Serial.print(F("Entering ML_LINK_COM_ASSOCIATION OR CHANGEKEY mode "));
        Serial.println(MirrorLink.status.comStatus);
        Serial.println(F(""));

        // Update second part of the key and update decryption key
        
        Serial.println(F(""));
        Serial.println(F("Changing key"));
        Serial.println(F(""));

        MirrorLink.associationKey[1] = buffer[1];
        MirrorLink.key[3] = MirrorLink.key[1];
        MirrorLink.key[2] = MirrorLink.key[0];
        MirrorLink.key[1] = MirrorLink.associationKey[1];
        MirrorLink.key[0] = MirrorLink.associationKey[0];
        speck_expand(MirrorLink.key, mirrorLinkSpeckKeyExp);
        // Change communication mode to normal
        MirrorLink.status.comStatus = ML_LINK_COM_NORMAL;

        Serial.println(F(""));
        Serial.print(F("Entering ML_LINK_COM_NORMAL mode "));
        Serial.println(F(""));
#else
      if (MirrorLink.status.comStatus == ML_LINK_COM_CHANGEKEY) {
        MirrorLink.status.powerLevel = ((buffer[0] & 0x3C0000) >> 18);
        MirrorLink.status.channelNumber = ((buffer[0] & 0x3C000) >> 14);
        // Update first part of the key
        MirrorLink.associationKey[0] = buffer[1];
#endif // defined(MIRRORLINK_OSREMOTE)
      }
      else {
        Serial.println(F(""));
        Serial.print(F("We are in ML_LINK_COM_NORMAL mode "));
        Serial.println(F(""));
#if defined(MIRRORLINK_OSREMOTE)
        MirrorLink.response = (uint32_t)buffer[1] << 16;
        // Gather SNR and RSSI from station
        // Decode RSSI from 0.2 resolution in 8 bit (value range from -255dBm to 255dBm)
        // Decode SNR from 0.2 resolution in 8 bit (value range from 0dB to 51.1dB)
        MirrorLink.rssiRemote = (int16_t)((buffer[1] & 0x7F0000) >> 16);
        if (buffer[1] && 0x800000) {
          MirrorLink.rssiRemote *= -1;
        }
        MirrorLink.rssiRemote *= 2;
        MirrorLink.snrRemote = (uint16_t)(((buffer[1] >> 24) & 0xFF) * 2);

        // Diagnostics
        // If response command different than last command sent then report error
        if ((MirrorLink.response >> 27) != (MirrorLink.buffer[MirrorLink.indexBufferTail] >> 27)) {
          Serial.println(F("Station response does not match command sent!"));
        }
#else
        MirrorLink.command = (uint32_t)buffer[1];
#endif // defined(MIRRORLINK_OSREMOTE)
      }
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1262] Data:\t\t"));
#if defined(MIRRORLINK_OSREMOTE)
      Serial.println(MirrorLink.response);
#else
      Serial.println(MirrorLink.command);
#endif // defined(MIRRORLINK_OSREMOTE)
      float signal;
      // print RSSI (Received Signal Strength Indicator)
      signal = lora.getRSSI();
      Serial.print(F("[SX1262] RSSI:\t\t"));
      Serial.print(signal);
      Serial.println(F(" dBm"));
      MirrorLink.rssiLocal = (int16_t)signal;

      signal = lora.getSNR();
      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(signal);
      Serial.println(F(" dB"));
      MirrorLink.snrLocal = (uint16_t)(signal * 10);
      
      rxSuccessful = true;
    } 
    else if (MirrorLink.moduleState == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));
    }
    else if (MirrorLink.moduleState != ERR_NONE) {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(MirrorLink.moduleState);
    }
    else {
#if defined(MIRRORLINK_OSREMOTE)
      Serial.println(F(""));
      Serial.print(F("Entering ML_LINK_COM_ASSOCIATION mode "));
      Serial.println(MirrorLink.status.comStatus);
      Serial.println(F(""));
      // Change communication mode to Association
      MirrorLink.status.comStatus = ML_LINK_COM_ASSOCIATION;
#else
      Serial.println(F(""));
      Serial.println(F("Resetting to default key"));
      Serial.println(F(""));
      // Check if it is an association packet
      MirrorLink.key[0] = SPECK_DEFAULT_KEY_N1;
      MirrorLink.key[1] = SPECK_DEFAULT_KEY_N2;
      MirrorLink.key[2] = SPECK_DEFAULT_KEY_N3;
      MirrorLink.key[3] = SPECK_DEFAULT_KEY_N4;
      speck_expand(MirrorLink.key, mirrorLinkSpeckKeyExp);
      speck_decrypt(encoded, buffer, mirrorLinkSpeckKeyExp);

      // Network ID match flag
      networkIdMatch = (bool)(MirrorLink.status.networkId == (0xFF & (buffer[0] >> 24)));

      if (  (MirrorLink.moduleState == ERR_NONE)
          &&(networkIdMatch) ) {
        rxSuccessful = true;
        MirrorLink.packetExchCtr = ((buffer[0] & 0x3FFF) % 16383);
        MirrorLink.status.comStatus = ((buffer[0] & 0xC00000) >> 22);
        MirrorLink.status.powerLevel = ((buffer[0] & 0x3C0000) >> 18);
        MirrorLink.status.channelNumber = ((buffer[0] & 0x3C000) >> 14);
        MirrorLink.associationKey[0] = buffer[1];

        float signal;
        // print RSSI (Received Signal Strength Indicator)
        signal = lora.getRSSI();
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(signal);
        Serial.println(F(" dBm"));
        MirrorLink.rssiLocal = (int16_t)signal;

        signal = lora.getSNR();
        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(signal);
        Serial.println(F(" dB"));
        MirrorLink.snrLocal = (uint16_t)(signal * 10);
        
        rxSuccessful = true;
      }
      else {
        // packet was received, but Network ID mismatch
        Serial.println(F("Network ID mismatch!"));
        // Change communication mode to association
        MirrorLink.status.comStatus = ML_LINK_COM_ASSOCIATION;
      }
#endif // defined(MIRRORLINK_OSREMOTE)
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
        // MirrorLink status down
        MirrorLink.status.link = ML_LINK_DOWN;
      }
    }
    else {
      // If timer not reached
      if (os.now_tz() < MirrorLink.stayAliveTimer) {
        // Enable system
        os.enable();
        // MirrorLink status up
        MirrorLink.status.link = ML_LINK_UP;
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
      if (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_NORMAL) {
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.sendTimer = os.now_tz();
      }
      else {
        Serial.println(F("STATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
        MirrorLink.sendTimer = os.now_tz();
      }
#else
      if (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_NORMAL) {
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
      else {
        Serial.println(F("STATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
        MirrorLink.sendTimer = 0;
        MirrorLinkReceiveInit();
      }
#endif // defined(MIRRORLINK_OSREMOTE)
      break;
    // Association state 
    case MIRRORLINK_ASSOCIATE:
#if defined(MIRRORLINK_OSREMOTE)
      // Wait for associating answer
      if (   (MirrorLinkReceiveStatus() == true) 
          && (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_NORMAL) ) {
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        // Calculate transmission-free time based on duty cycle and time of last message
        MirrorLink.sendTimer = os.now_tz() + (((MirrorLink.txTime * 2) * (10000 / (MIRRORLINK_MAX_DUTY_CYCLE))) / 10000);
        MirrorLinkReceiveInit();
#else
      if (   (MirrorLink.status.comStatus == (uint32_t)ML_LINK_COM_NORMAL)
          && (MirrorLinkTransmitStatus() == true)) {
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
      if (  (MirrorLink.sendTimer <= os.now_tz())
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
        uint32_t timeMillis = millis();
        if (MirrorLink.txTime < timeMillis) MirrorLink.txTime = timeMillis - MirrorLink.txTime;
        Serial.print(F("Transmission duration: "));
        Serial.println(MirrorLink.txTime);
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_RECEIVE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
        MirrorLinkReceiveInit();
      }
      else if (MirrorLink.sendTimer <= os.now_tz()) {
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        // Calculate transmission-free time based on duty cycle and time of last message
        MirrorLink.sendTimer = os.now_tz() + (((MirrorLink.txTime * 2) * (10000 / (MIRRORLINK_MAX_DUTY_CYCLE))) / 10000);
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
        
        // Response payload format
        // bit 16 to 22 = Number of active SID's (for diagnostic check) --> TODO!
        // bit 23 to 26 = Error bits
        // bit 27 to 31 = cmd

        // In case response shows a sync error between remote and station
        // delete all programs and reset response
        if (((MirrorLink.response >> 23) & 0xF) == ML_SYNCERROR) {
          delete_program_data(-1);
          Serial.println(F("Sync error with remote, reset program data!"));
        }

        // Report command sent
        if (MirrorLink.bufferedCommands > 0) {
          MirrorLink.bufferedCommands--;
          MirrorLink.indexBufferTail = (MirrorLink.indexBufferTail + 1) % MIRRORLINK_BUFFERLENGTH;
        }
        
        MirrorLink.response = 0;
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_BUFFERING"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        // Calculate transmission-free time based on duty cycle and time of last message
        MirrorLink.sendTimer = os.now_tz() + (((MirrorLink.txTime * 2) * (10000 / (MIRRORLINK_MAX_DUTY_CYCLE))) / 10000);

        // Update Link status
        MirrorLink.status.link = ML_LINK_UP;
      }

      // If timeout
      // Report error and change state as well to association
      if(MirrorLink.sendTimer <= os.now_tz()) {

        // Report error
        Serial.println(F("No answer received from station!"));

        // Command is lost, do not retry
        if (MirrorLink.bufferedCommands > 0) {
          MirrorLink.bufferedCommands--;
          (MirrorLink.indexBufferTail++) % MIRRORLINK_BUFFERLENGTH;
        }
        
        MirrorLink.response = 0;
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
        Serial.println(F("STATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
        // Calculate transmission-free time based on duty cycle and time of last message
        MirrorLink.sendTimer = os.now_tz() + (((MirrorLink.txTime * 2) * (10000 / (MIRRORLINK_MAX_DUTY_CYCLE))) / 10000);

        // Update Link status
        MirrorLink.status.link = ML_LINK_DOWN;
        MirrorLink.snrRemote = 0;
        MirrorLink.rssiRemote = -200;

        // Start association process
        MirrorLink.status.comStatus = (uint32_t)ML_LINK_COM_ASSOCIATION;
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

              // If the request is to add (or modify) a program
              if (addProg)
              {
                // In case the new pid does not match the max. program number
                // SYNC issue identified, remove all programs
                if (pid != pd.nprograms) {
                  // Delete all programs
                  delete_program_data(-1);
                  MirrorLink.command |= (((uint32_t)ML_SYNCERROR) << 23);
                }
                // Otherwise create new program or modify existing one
                else {
                  // Reset MirrorLinkProg
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
                  change_program_data(pid, pd.nprograms, &mirrorlinkProg);
                }
              }
              // Request is to delete a program
              else
              {
                Serial.print(F("Remove program/s: "));
                Serial.println(pid);
                Serial.print(F("Number of programs: "));
                Serial.println(pd.nprograms);
                // In case the new pid to be removed is not within the available pid's range
                // SYNC issue identified, remove all programs
                if (pid >= pd.nprograms) {
                  // Delete all programs
                  delete_program_data(-1);
                  MirrorLink.command |= (((uint32_t)ML_SYNCERROR) << 23);
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
              // Reset MirrorLinkProg
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
              // bit 0 to 24 = latitude (bit 24 for sign)
              // bit 24 to 26 = Not used
              // bit 27 to 31 = cmd
              payload = MirrorLinkGetCmd((uint8_t)ML_LATITUDE);
              MirrorLink.latitude = (int32_t)(0xFFFFFF & payload);
              if (payload & 0x1000000) MirrorLink.latitude *= -1;
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
              if (payload & 0x1000000) MirrorLink.longitude *= -1;
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
      else if (MirrorLink.status.comStatus == ML_LINK_COM_ASSOCIATION) {
        Serial.println(F("SATE: MIRRORLINK_ASSOCIATE"));
        MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
        MirrorLink.sendTimer = 0;
        MirrorLinkReceiveInit();
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
      // Association algorithm
#if defined(MIRRORLINK_OSREMOTE)
      // If timeout send association request
      if (MirrorLink.sendTimer <= os.now_tz()) {
        // Send association request
        MirrorLinkTransmit();
        MirrorLink.sendTimer = os.now_tz() + (time_t)MIRRORLINK_RXTX_MAX_TIME;
      }
      // If association request sent
      else if (MirrorLinkTransmitStatus() == true) {
        uint32_t timeMillis = millis();
        if (MirrorLink.txTime < timeMillis) MirrorLink.txTime = timeMillis - MirrorLink.txTime;
        // Wait for answer to association command
        MirrorLinkReceiveInit();
      }
#else
      // Wait for associating request
      if (MirrorLinkReceiveStatus() == true) {
        // Start transmission timer
        MirrorLink.sendTimer = (os.now_tz() + (time_t)(MIRRORLINK_RXTX_DEAD_TIME)); 
      }
      // If transmission timer is due
      else if (   (MirrorLink.sendTimer == os.now_tz())
               && (MirrorLink.status.flagRxTx == ML_RECEIVING)) {
        // If association request sent
        MirrorLinkTransmit();
      }
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
      if (   (MirrorLink.sendTimer == (os.now_tz() + (time_t)MIRRORLINK_RXTX_DEAD_TIME))
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