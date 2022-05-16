/*
  MirrorLink.cpp - LORA long range MirrorLink driver for OpenSprinkler

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
#if defined(DISABLE_ESP32_BROWNOUT)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif //defined(DISABLE_ESP32_BROWNOUT)

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

// Frequency hopping array (Default values acc. to German Bundesagenturnetz Vfg 12/2020)
float MirrorLinkFreqs[ML_CH_MAX] = {
#if defined(ML_LOCALTEST)
  866.235,    // ML_CH_0 -> 866.235MHz to 866.360MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
#else
  865.635,    // ML_CH_0 -> 865.635MHz to 865.760MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
#endif
  866.235,    // ML_CH_1 -> 866.235MHz to 866.360MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
  866.835,    // ML_CH_2 -> 866.835MHz to 866.960MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
  867.435,    // ML_CH_3 -> 867.435MHz to 867.560MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
  869.4,      // ML_CH_4 -> 869.4MHz to 869.525MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
  869.525,    // ML_CH_5 -> 869.525MHz to 869.65MHz, 500mW (27dBm) with ATPC, 10% Duty Cycle
};

// Intern program to store program data sent by remote
ProgramStruct mirrorlinkProg;

// Speck key expansion buffer
uint32_t MirrorLinkSpeckKeyExp[MIRRORLINK_SPECK_ROUNDS];

// Extern structures/buffer
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
    uint32_t mirrorLinkStationType : 1;     // MirrorLink station type (remote/station)
    uint32_t mirrorlinkState : 3;           // Operation mode of the MirrorLink system
    uint32_t networkId : 8;                 // Network ID for the Link
    uint32_t receivedFlag : 1;              // Flag to indicate that a packet was received
    uint32_t transmittedFlag : 1;           // Flag to indicate that a packet was sent
    uint32_t enableInterrupt : 1;           // Disable interrupt when it's not needed
    uint32_t flagRxTx : 1;                  // Flag to indicate if module is receiving or transmitting
    uint32_t rebootRequest : 1;             // Reboot request
    uint32_t stayAlive : 1;                 // Bit to indicate if the stayalive feature is active
    uint32_t link : 1;                      // Shows if link is up or down
    uint32_t comStationState : 2;           // Command sent from remote to change station state
    uint32_t powerCmd : 4;                  // Power level command to be sent by remote and executed by station
    uint32_t channelNumber : 4;             // Channel number for the link
    uint32_t freqHopState : 1;              // State of Frequency Hopping (1 = enable, 0 = disable)
    uint32_t atpcState : 1;                 // State of the Adaptive Transmissison Power Control (1 = enable, 0 = disable)
    uint32_t free : 1;                      // Free bits
  };
} MirrorLinkStateBitfield;

struct MIRRORLINK {
  uint32_t key[MIRRORLINK_SPECK_KEY_LEN];                        // Encryption key for the Link
  uint32_t nonceCtr[MIRRORLINK_SPECK_TEXT_LEN];                  // Nonce for the cipher plus counter
  uint32_t nonce[MIRRORLINK_SPECK_TEXT_LEN];                     // Nonce for the cipher
  uint32_t sendTimer;                                            // Timer in milliseconds to control send timing
  MirrorLinkStateBitfield status;                                // Bittfield including states as well as several flags
  uint32_t stayAliveTimer;                                       // Timer in milliseconds to control if remote and station are still connected
  uint32_t stayAliveMaxPeriod;                                   // Maximum period in milliseconds configured to control if remote and station are still connected
  uint32_t nonceUpdateTimer;                                     // Timer to control the nonce update process
  uint32_t packetsSent;                                          // Number of sent packets
  uint32_t packetsReceived;                                      // Number of received packets
  uint32_t txTime;                                               // Time used to transmit a payload in milliseconds
  uint32_t command[ML_CMD_MAX];                                  // Array containing 46 bit (14+32) bits of the command to be executed by the station
  int32_t latitude;                                              // Latitude of the remote station
  int32_t longitude;                                             // Longitude of the remote station
  uint32_t payloadBuffer[ML_CMD_MAX][MIRRORLINK_BUFFERLENGTH];   // Buffer for queued payloads (46 bit each) to be sent to station
  uint32_t bannedChannelTimer[ML_CH_MAX];                        // Timer controlling banned channels during a specific period of time
  int16_t moduleState;                                           // LORA module state
  uint16_t associationAttempts;                                  // Counter to control the number of association attempts
  uint16_t packetsLost[ML_CH_MAX];                               // Counter with the number of lost packets per channel within a defined timeframe
  int16_t snrLocal[ML_CH_MAX];                                   // Local SNR (local reception)
  int16_t rssiLocal[ML_CH_MAX];;                                 // Local RSSI (local reception)
  uint16_t dutyCycle;                                            // Maximum duty cycle in tenths of % (1 = 0.1)
  int16_t snrRemote[ML_CH_MAX];;                                 // Remote SNR (remote reception)
  int16_t rssiRemote[ML_CH_MAX];                                 // Remote RSSI (remote reception)
  uint8_t responseCommand;                                       // Response command from the station to the last command sent
  uint8_t bufferedCommands;                                      // Number of buffered commands to be sent
  uint8_t indexBufferHead;                                       // Index of the first element in the command buffer
  uint8_t indexBufferTail;                                       // Index of the last element in the command buffer   
  int8_t powerLevel;                                             // Transmission power
  int8_t powerMax;                                               // Maximum allowed transmission power
  uint8_t boardSelected;                                         // Selected board to show status in the MirrorLink web page
  uint8_t boardStatusBits[MAX_NUM_BOARDS];                       // Status bits of the boards on the station acc. to last sync
  uint8_t cmdErrors;                                             // Command errors errors
  uint8_t diagBits;                                              // Diagnostic bits
  uint8_t nextChannel;                                           // Next channel to be used after a packet/response has been exchanged
  uint8_t assocRetryCtr;                                         // Association retry counter
  uint8_t lastCmd;                                               // Last command executed in a sequence
  uint8_t maxProgrNum;                                           // Maximum program number in the remote
} MirrorLink;

void schedule_all_stations(ulong curr_time);
void turn_off_station(byte sid, ulong curr_time);
void schedule_test_station(byte sid, uint16_t duration);
void change_program_data(int32_t pid, byte numprograms, ProgramStruct *prog);
void delete_program_data(int32_t pid);

// Speck64/128 encryption based on NSA implementation guide plus CTR cipher mode
// Speck encryption key expand function
void MirrorLinkSpeckKeySchedule(uint32_t const K[MIRRORLINK_SPECK_KEY_LEN], uint32_t rk[MIRRORLINK_SPECK_ROUNDS])
{
  uint32_t i, b = K[0];
  uint32_t a[MIRRORLINK_SPECK_KEY_LEN - 1];

  for (i = 0; i < (MIRRORLINK_SPECK_KEY_LEN - 1); i++)
  {
    a[i] = K[i + 1];
  }
  rk[0] = b;  
  for (i = 0; i < MIRRORLINK_SPECK_ROUNDS - 1; i++) {
    ER(a[i % (MIRRORLINK_SPECK_KEY_LEN - 1)], b, i);
    rk[i + 1] = b;
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

uint8_t MirrorLinkGetStationType(void) {
  return ((uint8_t)MirrorLink.status.mirrorLinkStationType);
}

bool MirrorLinkSetNetworkId(uint8_t networkid) {
  bool accepted = false;
  if ((networkid <= UINT8_MAX) && (networkid >= 0)) {
    MirrorLink.status.networkId = networkid;
    os.iopts[IOPT_ML_NETWORKID] = networkid;
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options NetworkID"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetStationType(uint8_t type) {
  bool accepted = false;
  if (type == 1) {
    MirrorLink.status.mirrorLinkStationType = ML_REMOTE;
    os.iopts[IOPT_ML_STATIONTYPE] = ML_REMOTE;
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options Station Type"));
    os.iopts_save();
  }
  else if (type == 0) {
    MirrorLink.status.mirrorLinkStationType = ML_STATION;
    os.iopts[IOPT_ML_STATIONTYPE] = ML_STATION;
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options Station Type"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetKeys(uint32_t ask1, uint32_t ask2, uint32_t ask3, uint32_t ask4) {
  bool accepted = false;
  if (  ((ask1 <= UINT32_MAX) && (ask1 >= 0))
      &&((ask2 <= UINT32_MAX) && (ask2 >= 0))
      &&((ask3 <= UINT32_MAX) && (ask3 >= 0))
      &&((ask4 <= UINT32_MAX) && (ask4 >= 0)) ) {
    MirrorLink.key[0] = ask1;
    MirrorLink.key[1] = ask2;
    MirrorLink.key[2] = ask2;
    MirrorLink.key[3] = ask3;
    os.iopts[IOPT_ML_ASSOC_KEY1BYTE] = ((ask1 >> 24) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY2BYTE] = ((ask1 >> 16) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY3BYTE] = ((ask1 >> 8) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY4BYTE] = (ask1 & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY5BYTE] = ((ask2 >> 24) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY6BYTE] = ((ask2 >> 16) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY7BYTE] = ((ask2 >> 8) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY8BYTE] = (ask2 & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY9BYTE] = ((ask3 >> 24) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY10BYTE] = ((ask3 >> 16) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY11BYTE] = ((ask3 >> 8) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY12BYTE] = (ask3 & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY13BYTE] = ((ask4 >> 24) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY14BYTE] = ((ask4 >> 16) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY15BYTE] = ((ask4 >> 8) & 0xFF);
    os.iopts[IOPT_ML_ASSOC_KEY16BYTE] = (ask4 & 0xFF);
    MLDEBUG_PRINTLN(F("Saving options Keys"));
    os.iopts_save();
    MirrorLinkSpeckKeySchedule(MirrorLink.key, MirrorLinkSpeckKeyExp);
    accepted = true;
    }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetChannel(uint8_t channel) {
  bool accepted = false;
  if ((channel <= 15) && (channel >= 0)) {
    os.iopts[IOPT_ML_DEFCHANNEL] &= 0xF0;
    os.iopts[IOPT_ML_DEFCHANNEL] |= channel;
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options Channel"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetFrequencyHoppingStatus(uint8_t freqhopstatus) {
  bool accepted = false;
  if (freqhopstatus == 1) {
    os.iopts[IOPT_ML_DEFCHANNEL] |= 0x80;
    accepted = true;
    MirrorLink.status.freqHopState = (uint32_t)1;
    MLDEBUG_PRINTLN(F("Saving options Frequency Hopping Status"));
    os.iopts_save();
  }
  else if (freqhopstatus == 0) {
    os.iopts[IOPT_ML_DEFCHANNEL] &= 0x7F;
    accepted = true;
    MirrorLink.status.freqHopState = (uint32_t)0;
    MLDEBUG_PRINTLN(F("Saving options Frequency Hopping Status"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetMaxPower(int8_t maxpower) {
  bool accepted = false;
  if ((maxpower <= 30) && (maxpower >= -20)) {
    MirrorLink.powerMax = maxpower;
    os.iopts[IOPT_ML_MAXPOWER] = 0;
    if (maxpower < 0) {
      maxpower *= -1;
      os.iopts[IOPT_ML_MAXPOWER] = (0x40 | (uint8_t)maxpower);
    }
    else {
      os.iopts[IOPT_ML_MAXPOWER] = (uint8_t)maxpower;
    }
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options Power"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetATPCStatus(uint8_t atpcstatus) {
  bool accepted = false;
  if (atpcstatus == 1) {
    os.iopts[IOPT_ML_MAXPOWER] |= 0x80;
    accepted = true;
    MirrorLink.status.atpcState = (uint32_t)1;
    MLDEBUG_PRINTLN(F("Saving options Adaptive Transmission Power Control Status"));
    os.iopts_save();
  }
  else if (atpcstatus == 0) {
    os.iopts[IOPT_ML_MAXPOWER] &= 0x7F;
    accepted = true;
    MirrorLink.status.atpcState = (uint32_t)0;
    MLDEBUG_PRINTLN(F("Saving options Adaptive Transmission Power Control Status"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetDutyCycle(float dutycycle) {
  bool accepted = false;
  if ((dutycycle <= 100) && (dutycycle >= 0)) {
    MirrorLink.dutyCycle = uint16_t(dutycycle * 10);
    os.iopts[IOPT_ML_DUTYCYCLE1] = ((MirrorLink.dutyCycle >> 8) & 0xFF);
    os.iopts[IOPT_ML_DUTYCYCLE2] = (MirrorLink.dutyCycle & 0xFF);
    accepted = true;
    MLDEBUG_PRINTLN(F("Saving options DutyCycle"));
    os.iopts_save();
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkSetBoardSelect(uint8_t boardNumber) {
  bool accepted = false;
  if ((boardNumber > 0) && (boardNumber <= os.nboards)) {
    MirrorLink.boardSelected = (boardNumber - 1);
    MLDEBUG_PRINT(F("Board Selected: "));
    MLDEBUG_PRINTLN(MirrorLink.boardSelected);
    accepted = true;
  }
  else {
    accepted = false;
  }
  return accepted;
}

bool MirrorLinkGetAssociationStatus(void) {
  bool associated = false;
  if (MirrorLink.status.comStationState != ML_LINK_COM_ASSOCIATION) {
    associated = true;
  }
  return associated;
}

uint8_t MirrorLinkGetChannel(void) {
  return MirrorLink.status.channelNumber;
}

int8_t MirrorLinkGetPower(void) {
  return MirrorLink.powerLevel;
}

void MirrorLinkBuffCmd(uint8_t cmd, uint32_t payload) {
  switch (cmd) {
    // Initial state
    case ML_TESTSTATION:
      // Buffer message format: 
      // bit 0 = status (1 = On, 0 = Off)
      // bit 1 to 8 = sid
      // bit 9 to 24 = time(sec)
      // bit 25 to 31 = Not used
    case ML_PROGRAMADDDEL:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 = Add (1) or delete (0)
      // bit 8 to 31 = Not used
    case ML_PROGRAMMAINSETUP:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 = enable/disable
      // bit 8 = use weather
      // bit 9 to 10 = Odd/even restriction
      // bit 11 to 12 = schedule type
      // bit 13 to 31 = Not used
    case ML_PROGRAMDAYS:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 22 = days
      // bit 23 to 31 = Not used
    case ML_PROGRAMSTARTTIME:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 8 = start time number (max. is 4 for each program)
      // bit 9 to 24 = start time
      // bit 25 = Starttime type
      // bit 26 to 31 = Not used
    case ML_PROGRAMDURATION:
      // Buffer message format:
      // bit 0 to 6 = program number (max. is 40)
      // bit 7 to 14 = sid
      // bit 15 to 25 = time (min)
      // bit 26 to 31 = Not used
    case ML_TIMESYNC:
      // Buffer message format:
      // bit 0 to 26 = Unix Timestamp in minutes! not seconds
      // bit 27 to 31 = Not used
    case ML_TIMEZONESYNC:
      // Buffer message format:
      // bit 0 to 7 = Time zone
      // bit 27 to 31 = Not used
    case ML_CURRENTREQUEST:
      // TODO:
    case ML_EMERGENCYSHUTDOWN:
    case ML_STATIONREBOOT:
      // Reboot:
    case ML_LATITUDE:
      // Buffer message format:
      // bit 0 to 23 = latitude
      // bit 24 to 31 = Not used
    case ML_LONGITUDE:
      // Buffer message format:
      // bit 0 to 23 = longitude
      // bit 24 to 31 = Not used
    case ML_SUNRISE:      
      // Buffer message format:
      // bit 0 to 15 = sunrise time
      // bit 16 to 31 = Not used
    case ML_SUNSET:
      // Buffer message format:
      // bit 0 to 15 = sunset time
      // bit 16 to 31 = Not used
    case ML_RAINDELAYSTOPTIME:
      // TODO:
    case ML_STAYALIVE:
      // Payload format: 
      // bit 0 to 26 = Stayalive configured counter
      // bit 27 to 31 = Not used
      if (MirrorLink.bufferedCommands < MIRRORLINK_BUFFERLENGTH) {
        MirrorLink.bufferedCommands++;
        MirrorLink.payloadBuffer[ML_CMD_1][MirrorLink.indexBufferHead] = ((uint32_t)(0x3F & (uint16_t)cmd) << CMD_REMOTE_POS);
        MirrorLink.payloadBuffer[ML_CMD_2][MirrorLink.indexBufferHead] = payload;
        MirrorLink.indexBufferHead = (MirrorLink.indexBufferHead + 1) % MIRRORLINK_BUFFERLENGTH;
      }
      break;
  }
}

void MirrorLinkGetCmd(uint8_t cmd, uint32_t *payload)
{
  if(cmd == ((MirrorLink.command[ML_CMD_1] & CMD_REMOTE_MASK) >> CMD_REMOTE_POS)) {
    payload[ML_CMD_1] = (MirrorLink.command[ML_CMD_1] & PAYLOAD1_REMOTE_MASK);
    payload[ML_CMD_2] = MirrorLink.command[ML_CMD_2];
  }
  // Empty command but cmd part to send answer to remote station
  MirrorLink.command[ML_CMD_2] = 0;
  MirrorLink.command[ML_CMD_2] |= (((uint32_t)(MirrorLink.command[ML_CMD_1] >> CMD_REMOTE_POS)) << CMD_STATION_POS);
  MirrorLink.command[ML_CMD_1] &= HEADER_MASK;
}

void MirrorLinkPeriodicCommands(void) {
  // Send regular commands (slow)
  if (((millis() / 1000) % (time_t)MIRRORLINK_REGCOMMANDS_SLOW_PERIOD) == 0) {
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

    // Send ML_STAYALIVE config
    // bit 0 to 25 = Stayalive configured counter
    // bit 26 = Bit indicating if stayalive feature shall be used (true) or ignored (false)
    // bit 27 to 31 = cmd
    MirrorLinkBuffCmd((uint8_t)ML_STAYALIVE, (uint32_t)((((uint32_t)1) << 26) | (0x3FFFFFF & MIRRORLINK_STAYALIVE_PERIOD)));
  }
  // Send regular commands (mid)
  if (((millis() / 1000) % (time_t)MIRRORLINK_REGCOMMANDS_MID_PERIOD) == 0) {
    // Send ML_SUNRISE
    MirrorLinkBuffCmd((uint8_t)ML_SUNRISE, (uint16_t)(0xFFFF & os.nvdata.sunrise_time));
    // Send ML_SUNSET
    MirrorLinkBuffCmd((uint8_t)ML_SUNSET, (uint16_t)(0xFFFF & os.nvdata.sunset_time));
  }
  // Send regular commands (fast)
  if (((millis() / 1000) % (time_t)MIRRORLINK_REGCOMMANDS_FAST_PERIOD) == 0) {
    // Send ML_TIMESYNC just in case no messages in the buffer (to avoid inducing time delay between REMOTE and STATION)
    if (MirrorLink.bufferedCommands == 0) {
      // Send ML_TIMESYNC
      // bit 0 to 26 = Unix Timestamp in minutes! not seconds
      // bit 27 to 31 = cmd
      MirrorLinkBuffCmd((uint8_t)ML_TIMESYNC, (uint32_t)(0x7FFFFFF & (RTC.get() / 60)));
    }
  }
}

// General Status MirrorLink for wifi server
String MirrorLinkStatusGeneral() {
	String mirrorLinkInfo;
	// Encode in JSON message
  mirrorLinkInfo = "{\"mode\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
    mirrorLinkInfo += "REMOTE";
  }
  else {
    mirrorLinkInfo += "STATION";
  }
  mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"networkid\":["; 
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += String(MirrorLink.status.networkId);
	mirrorLinkInfo += "\"";
  mirrorLinkInfo += "]}";
	return mirrorLinkInfo;
}

// Radio Status MirrorLink for wifi server
String MirrorLinkStatusRadio() {
	String mirrorLinkInfo;
	// Encode in JSON message
	mirrorLinkInfo = "{\"frequency\":["; 
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += String(MirrorLinkFreqs[MirrorLink.status.channelNumber]);
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"dutycycle\":["; 
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += String((MirrorLink.dutyCycle) / 10);
	mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"powerlevel\":["; 
	mirrorLinkInfo += "\"";
  mirrorLinkInfo += String(MirrorLink.powerLevel);
  mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"rssis\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(MirrorLink.rssiLocal[MirrorLink.status.channelNumber]);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += ",\r\n";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
    if (MirrorLink.status.link == ML_LINK_UP) {
      mirrorLinkInfo += String(MirrorLink.rssiRemote[MirrorLink.status.channelNumber]);
    }
    else {
      mirrorLinkInfo += "N.A.";
    }
  }
  else {
  mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
	mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"snrs\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.link == ML_LINK_UP) {
    mirrorLinkInfo += String(((float)MirrorLink.snrLocal[MirrorLink.status.channelNumber]) / 10);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += ",\r\n";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
    if (MirrorLink.status.link == ML_LINK_UP) {
      mirrorLinkInfo += String(((float)MirrorLink.snrRemote[MirrorLink.status.channelNumber]) / 10);
    }
    else {
      mirrorLinkInfo += "N.A.";
    }
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"assocst\":[";
  mirrorLinkInfo += "\"";
  switch (MirrorLink.status.comStationState) {
    case ML_LINK_COM_ASSOCIATION:
      mirrorLinkInfo += "ASSOCIATING";
      break;
    case ML_LINK_COM_NONCEUPDATE:
      mirrorLinkInfo += "NONCE UPDATE";
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

// Packets Status MirrorLink for wifi server
String MirrorLinkStatusPackets() {
	String mirrorLinkInfo;
	// Encode in JSON message
  mirrorLinkInfo = "{\"buffpackets\":["; 
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
    mirrorLinkInfo += String(MirrorLink.bufferedCommands);
  }
  else {
    mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"packetstx\":[";
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += String(MirrorLink.packetsSent);
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"packetsrx\":[";
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += String(MirrorLink.packetsReceived);
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"encryption\":[";
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "Speck 64/128 CTR";
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"packettime\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.flagRxTx == ML_RECEIVING) {
    mirrorLinkInfo += String(MirrorLink.txTime);
  }
  else {
    mirrorLinkInfo += "Calculating...";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"notxtime\":[";
  mirrorLinkInfo += "\"";
  if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
    if (MirrorLink.status.flagRxTx == ML_RECEIVING) {
      if (MirrorLink.sendTimer >= millis()) {
        mirrorLinkInfo += String((MirrorLink.sendTimer - millis()) / 1000);
      }
      else {
        mirrorLinkInfo += String((UINT32_MAX - MirrorLink.sendTimer + millis()) / 1000);
      }
    }
    else {
      mirrorLinkInfo += "Calculating...";
    }
  }
  else {
      mirrorLinkInfo += "N.A.";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "]}";
	return mirrorLinkInfo;
}

// Board Status MirrorLink for wifi server
String MirrorLinkStatusBoards() {
	String mirrorLinkInfo;
	// Encode in JSON message
  mirrorLinkInfo = "{\"boardnumber\":["; 
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += String((MirrorLink.boardSelected + 1));
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput1\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x1)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput2\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x2)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput3\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x4)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput4\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x8)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput5\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x10)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput6\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x20)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput7\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x40)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "],";
  mirrorLinkInfo += "\"boardoutput8\":[";
  mirrorLinkInfo += "\"";
  if ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & 0x80)) {
    mirrorLinkInfo += "ON";
  }
  else {
    mirrorLinkInfo += "OFF";
  }
  mirrorLinkInfo += "\"";
  mirrorLinkInfo += "]}";
	return mirrorLinkInfo;
}

// Return next channel to be selected
uint8_t MirrorLinkSelectChannel(void) {
  uint8_t selChannel = 0;
  uint8_t numBannedChannels = 0;
//  bool channelFree = false;
  // start scanning current channel
  MLDEBUG_PRINTLN(F("Selecting channel"));
  // Send next channel to station
  if (MirrorLink.status.freqHopState == 1) {
#if defined(MIRRORLINK_ENABLE_BLACKLIST)
    // Update banned channel list and its duration
    for (uint8_t i = 0; i < ML_CH_MAX; i++) {
      // Update banned list
      if (MirrorLink.packetsLost[i] > MIRRORLINK_CHANNEL_BLACKLIST_NUM) {
        // Ban channel
        MirrorLink.bannedChannelTimer[i] = (millis() / 1000) + MIRRORLINK_CHANNEL_BLACKLIST_TIME;
        // Reset ban counter
        MirrorLink.packetsLost[i] = 0;
      }
      // Increase banned channel counter if needed
      if (MirrorLink.bannedChannelTimer[i] > (millis() / 1000)) numBannedChannels++;
    }
    MLDEBUG_PRINT(F("Number of banned channels: "));
    MLDEBUG_PRINTLN(numBannedChannels);
    // If all channels but one are banned, free all channels
    if (numBannedChannels == (ML_CH_MAX - 1)) {
      for (uint8_t i = 0; i < ML_CH_MAX; i++) {
        // Reset ban channel timer
        MirrorLink.bannedChannelTimer[i] = 0;
        // Reset ban counter
        MirrorLink.packetsLost[i] = 0;
      }
    }
    // Assure default channel is never banned
    MirrorLink.bannedChannelTimer[((os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF)] = 0;
#endif
    // Look for a channel a maximum of ML_CH_MAX times otherwise reset to default channel
//    for (uint8_t i = 0; i < ML_CH_MAX; i++) {
      selChannel = random(0, ML_CH_MAX);
#if defined(MIRRORLINK_ENABLE_BLACKLIST)
      while (MirrorLink.bannedChannelTimer[selChannel] > (millis() / 1000)) selChannel = ((selChannel + 1) % ML_CH_MAX);
      // If ban is no longer valid
//      if (MirrorLink.bannedChannelTimer[selChannel] <= (millis() / 1000)) {
        MLDEBUG_PRINT(F("Channel: "));
        MLDEBUG_PRINT(selChannel);
        // Channel is free
        MLDEBUG_PRINTLN(F(" -> Channel is valid!"));
//        channelFree = true;
//        break;
//      }
      // else {
      //   // no preamble was detected, channel is free
      //   MLDEBUG_PRINT(F("Channel: "));
      //   MLDEBUG_PRINT(selChannel);
      //   MLDEBUG_PRINTLN(F(" -> Channel is banned!"));
      //   channelFree = false;
      //   break;
      // }
//#else
//    channelFree = true;
#endif
//    }
    // In case no free channel found then switch to default one
//    if (channelFree == false) selChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
    // Print list of channels and packetlost counter status
    MLDEBUG_PRINTLN(F("Status Packet Lost per Channel: "));
    for (uint8_t i = 0; i < ML_CH_MAX; i++) {
      MLDEBUG_PRINT(F("Packet Lost Channel "));
      MLDEBUG_PRINT(i);
      MLDEBUG_PRINT(F(": "));
      MLDEBUG_PRINTLN(MirrorLink.packetsLost[i]);
#if defined(MIRRORLINK_ENABLE_BLACKLIST)
      MLDEBUG_PRINT(F("Banned Time Channel "));
      MLDEBUG_PRINT(i);
      MLDEBUG_PRINT(F(": "));
      if (MirrorLink.bannedChannelTimer[i] >= (millis() / 1000)) {
        MLDEBUG_PRINTLN((MirrorLink.bannedChannelTimer[i] - (millis() / 1000)));
      }
      else {
        MLDEBUG_PRINTLN(0);
      }
#endif
    }
  }
  else {
//    channelFree = true;
    selChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
    MLDEBUG_PRINT(F("Channel: "));
    MLDEBUG_PRINTLN(selChannel);
  }
  return selChannel;
}

// Calculate power level
int8_t MirrorLinkPowerLevel(void) {
  int16_t powerLevel;
  if (   (MirrorLink.status.link == ML_LINK_DOWN)
      || (MirrorLink.status.atpcState == 0)) {
    powerLevel = MirrorLink.powerMax;
    MLDEBUG_PRINTLN(F("ML_LINK_DOWN"));
  }
  else {
    if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
      // Calculate power level based on local max, snrremote and rssiremote
      // Power level = last Power Level + MIRRORLINK_MIN_POWER_BUDGET - (RSSI + SNR)
      powerLevel = MirrorLink.powerLevel + MIRRORLINK_MIN_POWER_BUDGET - ((int16_t)MirrorLink.rssiRemote[MirrorLink.status.channelNumber] + (int16_t)(MirrorLink.snrRemote[MirrorLink.status.channelNumber] / 10));
    }
    else {
      // Calculate power level based on local max and powerCmd received from remote
      uint8_t powerCmd = MirrorLink.status.powerCmd;
      powerLevel = (10 * ((powerCmd & 0x4) >> 2)) + (5 * ((powerCmd & 0x2) >> 1)) + (powerCmd & 0x1);
      if (powerCmd & 0x8) powerLevel *= -1;
      powerLevel += MirrorLink.powerLevel;
    }

    // Max./Min. absolute limits
    if (powerLevel < MIRRORLINK_MIN_POWER) {
      powerLevel = MIRRORLINK_MIN_POWER;
    }
    else if (powerLevel > MIRRORLINK_MAX_POWER) {
      powerLevel = MIRRORLINK_MAX_POWER;
    }

    //Max. relative limit
    if (powerLevel > MirrorLink.powerMax) {
      powerLevel = MirrorLink.powerMax;
    }
  }

  MirrorLink.powerLevel = powerLevel;

  return (int8_t)powerLevel;
}

// Compute and encode power command to be sent from remote to station, max -16dB to +16dB relative change 
// in different steps
uint8_t MirrorLinkPowerCmdEncode(void) {
  uint8_t powerCmd = 0;
  int16_t stationPower = 0;

  // Calculate required station power
  stationPower = (int16_t)MIRRORLINK_MIN_POWER_BUDGET - ((int16_t)MirrorLink.rssiLocal[MirrorLink.status.channelNumber] + (int16_t)(MirrorLink.snrLocal[MirrorLink.status.channelNumber] / 10));

  // If link down use max power
  if (MirrorLink.status.link == ML_LINK_DOWN) stationPower = MirrorLink.powerMax;

  // Total 4 bit, MSB is the sign, bit 3 represents 10dB, bit 2 represents 5dB, bit 1 represents 1dB
  
  // Calculate sign
  if (stationPower < 0) {
    powerCmd |= 0x8;
    stationPower *= -1;
  }

  // Calculate 10dB step
  if ((stationPower / 10) >= 1) {
    powerCmd |= 0x4;
    stationPower -= 10;
  }

  // Calculate 5dB step
  if ((stationPower / 5) >= 1) {
    powerCmd |= 0x2;
    stationPower -= 5;
  }

  // Calculate 1dB step
  if ((stationPower / 1) >= 1) {
    powerCmd |= 0x1;
    stationPower -= 1;
  }

  stationPower = (10 * ((powerCmd & 0x4) >> 2)) + (5 * ((powerCmd & 0x2) >> 1)) + (powerCmd & 0x1);
  if (powerCmd & 0x8) stationPower *= -1;

  return powerCmd;
}

void MirrorLinkSetToDefaultKey(uint32_t *decryptionKey) {
  MLDEBUG_PRINTLN(F("Resetting to default key"));
  decryptionKey[0] = (((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY1BYTE] << 24) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY2BYTE] << 16) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY3BYTE] << 8) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY4BYTE]));
  decryptionKey[1] = (((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY5BYTE] << 24) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY6BYTE] << 16) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY7BYTE] << 8) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY8BYTE]));
  decryptionKey[2] = (((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY9BYTE] << 24) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY10BYTE] << 16) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY11BYTE] << 8) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY12BYTE]));
  decryptionKey[3] = (((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY13BYTE] << 24) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY14BYTE] << 16) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY15BYTE] << 8) | ((uint32_t)os.iopts[IOPT_ML_ASSOC_KEY16BYTE]));
  MirrorLinkSpeckKeySchedule(decryptionKey, MirrorLinkSpeckKeyExp);
}

void MirrorLinkIncrementNonceCtr(uint32_t *nonceCtr) {
  MLDEBUG_PRINTLN(F("Incrementing nonceCtr"));
  if (nonceCtr[0] == UINT32_MAX) {
    nonceCtr[0] = 0;
    if (nonceCtr[1] == UINT32_MAX) {
      nonceCtr[1] = 0;
    }
    else {
      nonceCtr[1]++;
    }
  }
  else {
    nonceCtr[0]++;
  }
}

// MirrorLink module initialization
void MirrorLinkInit(void) {
  MirrorLink.moduleState = ERR_NONE;
  MirrorLink.status.mirrorLinkStationType = (uint32_t)(os.iopts[IOPT_ML_STATIONTYPE]);
  MirrorLink.status.networkId = (uint32_t)(os.iopts[IOPT_ML_NETWORKID]);
  MirrorLink.status.receivedFlag = (uint32_t)false;
  MirrorLink.status.transmittedFlag = (uint32_t)false;
  MirrorLink.status.enableInterrupt = (uint32_t)true;
  MirrorLink.status.mirrorlinkState = (uint32_t)MIRRORLINK_INIT;
  MirrorLink.status.flagRxTx = (uint32_t)ML_RECEIVING;
  MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_ASSOCIATION;
  MirrorLink.status.rebootRequest = (uint32_t)false;
  MirrorLink.status.stayAlive = (uint32_t)true;
  MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
  MirrorLink.status.link = ML_LINK_DOWN;
  MirrorLink.status.powerCmd = (uint32_t)0;
  MirrorLink.status.freqHopState = ((uint32_t)((os.iopts[IOPT_ML_DEFCHANNEL]) & 0x80) >> 7);
  MirrorLink.status.atpcState = ((uint32_t)((os.iopts[IOPT_ML_MAXPOWER]) & 0x80) >> 7);
  for (uint8_t i = 0; i < MIRRORLINK_SPECK_TEXT_LEN; i++) {
    MirrorLink.nonce[i] = 0;
    MirrorLink.nonceCtr[i] = 0;
  }
  MirrorLink.assocRetryCtr = 0;
  MirrorLink.lastCmd = ML_NO_CMD;
  MirrorLink.maxProgrNum = pd.nprograms;
  MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME* 1000);
  MirrorLink.stayAliveMaxPeriod = ((uint32_t)MIRRORLINK_STAYALIVE_PERIOD * 1000);
  MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
  MirrorLink.nonceUpdateTimer = millis() + random(((uint32_t)MIRRORLINK_KEYCHANGE_MIN_TIME * 1000), ((uint32_t)MIRRORLINK_KEYCHANGE_MAX_TIME * 1000));
  MirrorLinkSetToDefaultKey(MirrorLink.key);
  MirrorLink.packetsSent = 0;
  MirrorLink.packetsReceived = 0;
  for (uint8_t i = 0; i < ML_CH_MAX; i++) {
    MirrorLink.snrLocal[i] = -200;
    MirrorLink.rssiLocal[i] = -200;
    MirrorLink.snrRemote[i] = -200;
    MirrorLink.rssiRemote[i] = -200;
    MirrorLink.packetsLost[i] = 0;
    MirrorLink.bannedChannelTimer[i] = 0;
  }
  MirrorLink.nextChannel = 0;
  MirrorLink.associationAttempts = 0;
#if defined(ML_LOCALTEST)
  MirrorLink.dutyCycle = (uint16_t)(900);
#else
  MirrorLink.dutyCycle = (uint16_t)(((uint32_t)os.iopts[IOPT_ML_DUTYCYCLE1] << 8) | ((uint32_t)os.iopts[IOPT_ML_DUTYCYCLE2]));
#endif
  MirrorLink.bufferedCommands = 0;
  for (uint8_t i = 0; i < ML_CMD_MAX; i++) {
    for (uint8_t j = 0; j < MIRRORLINK_BUFFERLENGTH; j++) {
      MirrorLink.payloadBuffer[i][j] = 0;
    }
  }
  for (uint8_t i = 0; i < ML_CMD_MAX; i++) MirrorLink.command[i] = 0;
  MirrorLink.indexBufferHead = 0;
  MirrorLink.indexBufferTail = 0;
  MirrorLink.txTime = 0;
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
  if (os.iopts[IOPT_ML_MAXPOWER] & 0x40) {
    MirrorLink.powerMax = (int8_t)(os.iopts[IOPT_ML_MAXPOWER] & 0x3F) * ((int8_t)-1);
  }
  else {
    MirrorLink.powerMax = (int8_t)(os.iopts[IOPT_ML_MAXPOWER] & 0x3F);
  }
  MirrorLink.powerLevel = MirrorLink.powerMax;
  MirrorLink.boardSelected = 0;
  MirrorLink.cmdErrors = ML_NO_ERROR;
  MirrorLink.diagBits = 0;
  for (uint8_t i; i < MAX_NUM_BOARDS; i++) MirrorLink.boardStatusBits[i] = 0;

	MLDEBUG_BEGIN(115200);

  // Brownout disabled
  #if defined(DISABLE_ESP32_BROWNOUT)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  
  #endif //defined(DISABLE_ESP32_BROWNOUT)

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
  MLDEBUG_PRINT(F("[SX1262] Initializing ... "));
  // initialize SX1262 
  // carrier frequency:           868.0 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            12
  // coding rate:                 5
  // sync word:                   0x1424 (private network)
  // output power:                16 dBm (30 dBm with E22-900T30S amplification)
  // current limit:               120 mA
  // preamble length:             8 symbols
  // CRC:                         enabled
  MirrorLink.moduleState = lora.begin(MirrorLinkFreqs[MirrorLink.status.channelNumber], 125.0, 12, 5, SX126X_SYNC_WORD_PRIVATE, 16 , 8, (float)(1.8), true);
  if (MirrorLink.moduleState == ERR_NONE) {
    MLDEBUG_PRINTLN(F("success!"));
  } else {
    MLDEBUG_PRINT(F("failed, code "));
    MLDEBUG_PRINTLN(MirrorLink.moduleState);
  }

  // Set automatic LRDO
  lora.autoLDRO();

  // Current limitation
  MirrorLink.moduleState = lora.setCurrentLimit(120.0);
  if (MirrorLink.moduleState == ERR_INVALID_CURRENT_LIMIT) {
    MLDEBUG_PRINTLN(F("Current limit configure exceedes max.!"));
  }

#if defined(RADIOLIB_GODMODE)
  // Set SX126x_REG_RX_GAIN to 0x96 -> LNA +3dB gain SX126xWriteRegister( SX126x_REG_RX_GAIN, 0x96 );
  uint16_t modReg = 0x08AC; //SX126X_REG_RX_GAIN
  uint8_t modData[1] = { 0x96 };
  MirrorLink.moduleState = lora.writeRegister(modReg, modData, 1); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity
  if (MirrorLink.moduleState != ERR_NONE) {
    MLDEBUG_PRINTLN(F("LNA max gain not set successfully!"));
  }
#endif //defined(RADIOLIB_GODMODE)

  // Set the function that will be called
  // when new packet is transmitted or received
  lora.setDio1Action(setFlag);

  // Set of initial messages
  
  // Send ML_TIMESYNC
  // bit 0 to 26 = Unix Timestamp in minutes! not seconds
  // bit 27 to 31 = Not used
  MirrorLinkBuffCmd((uint8_t)ML_TIMESYNC, (uint32_t)(0x7FFFFFF & (RTC.get() / 60)));

  // Send ML_TIMEZONESYNC
  // Payload format:
  // bit 0 to 7 = Time zone
  // bit 27 to 31 = Not used
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
  // bit 24 to 31 = Not used

  // Encode negative longitude
  if (fp_lon < 0) {
    fp_lon *= -1;
    fp_lon |= 0x1000000;
  }
  MirrorLinkBuffCmd((uint8_t)ML_LONGITUDE, (uint32_t)(0x1FFFFFF & fp_lon));

  // Payload format: 
  // bit 0 to 24 = latitude (bit 24 for sign)
  // bit 25 to 31 = Not used

  // Encode negative latitude
  if (fp_lat < 0) {
    fp_lat *= -1;
    fp_lat |= 0x1000000;
  }
  MirrorLinkBuffCmd((uint8_t)ML_LATITUDE, (uint32_t)(0x1FFFFFF & fp_lat));

  // Send ML_STAYALIVE config
  // bit 0 to 25 = Stayalive configured counter
  // bit 26 = Bit indicating if stayalive feature shall be used (true) or ignored (false)
  // bit 27 to 31 = cmd
  MirrorLinkBuffCmd((uint8_t)ML_STAYALIVE, (uint32_t)((((uint32_t)1) << 26) | (0x3FFFFFF & MIRRORLINK_STAYALIVE_PERIOD)));
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
      MLDEBUG_PRINTLN(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
      txSuccessful = true;

      // Calculate transmission time
      uint32_t timeMillis = millis();
      if (MirrorLink.txTime < timeMillis) MirrorLink.txTime = timeMillis - MirrorLink.txTime;

      // Increase transmission counter
      MirrorLink.packetsSent++;
    } 
    else {
      MLDEBUG_PRINT(F("failed, code "));
      MLDEBUG_PRINTLN(MirrorLink.moduleState);
    }
    MirrorLink.status.enableInterrupt = (uint16_t)true;
  }
  return txSuccessful;
}

bool MirrorLinkTransmit(byte *txArray) {
  bool txStartSuccessful = false;
  enableTX();
  MLDEBUG_PRINTLN(F("[SX1262] Starting to transmit ... "));
  MirrorLink.powerLevel = MirrorLinkPowerLevel();
  MirrorLink.moduleState = lora.setFrequency(MirrorLinkFreqs[MirrorLink.status.channelNumber]);
  MirrorLink.moduleState = lora.setOutputPower(MirrorLink.powerLevel);
  MirrorLink.moduleState = lora.startTransmit(txArray, 8);
  if (MirrorLink.moduleState == ERR_NONE) {
    txStartSuccessful = true;
    MirrorLink.txTime = millis();
  }
  MirrorLink.status.flagRxTx = ML_TRANSMITTING;

  MLDEBUG_PRINT(F("Channel to be sent:\t\t"));
  MLDEBUG_PRINTLN(MirrorLink.status.channelNumber);
  MLDEBUG_PRINT(F("PowerCMD to be sent:\t\t"));
  #if defined(ENABLE_DEBUG_MIRRORLINK) /** Serial debug functions */
  // Calculate power level based on local max and powerCmd received from remote
  uint8_t powerCmd = MirrorLink.status.powerCmd;
  int16_t powerLevel = (10 * ((powerCmd & 0x4) >> 2)) + (5 * ((powerCmd & 0x2) >> 1)) + (powerCmd & 0x1);
  if (powerCmd & 0x8) powerLevel *= -1;
  powerLevel += MirrorLink.powerLevel;
  #endif
  MLDEBUG_PRINTLN(powerLevel);

  return txStartSuccessful;
}

void MirrorLinkReceiveInit(void) {
  // Important! To enable receive you need to switch the SX126x antenna switch to RECEIVE 
  enableRX();
  // Set reception frequency
  MirrorLink.moduleState = lora.setFrequency(MirrorLinkFreqs[MirrorLink.status.channelNumber]);
  // start listening for LoRa packets
  MLDEBUG_PRINT(F("[SX1262] Starting to listen ... "));
  MirrorLink.moduleState = lora.startReceive();
  if (MirrorLink.moduleState == ERR_NONE) {
    MLDEBUG_PRINTLN(F("success!"));
    MirrorLink.status.flagRxTx = ML_RECEIVING;
  } else {
    MLDEBUG_PRINT(F("failed, code "));
    MLDEBUG_PRINTLN(MirrorLink.moduleState);
  }
}

bool MirrorLinkReceive(byte *rxArray) {
  bool rxSuccessful = false;
  if(MirrorLink.status.receivedFlag) {
    // Increase reception counter
    MirrorLink.packetsReceived++;

    // disable the interrupt service routine while
    // processing the data
    MirrorLink.status.enableInterrupt = (uint16_t)false;

    // Reset flag
    MirrorLink.status.receivedFlag = (uint16_t)false;

    // Read received data as byte array
    MirrorLink.moduleState = lora.readData(rxArray, MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH);

    // If reception successful
    if (MirrorLink.moduleState == ERR_NONE) {
      rxSuccessful = true;
      // packet was successfully received
      MLDEBUG_PRINTLN(F("[SX1262] Received packet!"));

      float signal;
      // print RSSI (Received Signal Strength Indicator)
      signal = lora.getRSSI();
      MLDEBUG_PRINT(F("[SX1262] RSSI:\t\t"));
      MLDEBUG_PRINT(signal);
      MLDEBUG_PRINTLN(F(" dBm"));
      MirrorLink.rssiLocal[MirrorLink.status.channelNumber] = (int16_t)signal;

      signal = lora.getSNR();
      // print SNR (Signal-to-Noise Ratio)
      MLDEBUG_PRINT(F("[SX1262] SNR:\t\t"));
      MLDEBUG_PRINT(signal);
      MLDEBUG_PRINTLN(F(" dB"));
      MirrorLink.snrLocal[MirrorLink.status.channelNumber] = (int16_t)(signal * 10);
    }
    else if (MirrorLink.moduleState == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      MLDEBUG_PRINTLN(F("CRC error!"));
    }
    else if (MirrorLink.moduleState != ERR_NONE) {
      // some other error occurred
      MLDEBUG_PRINT(F("failed, code "));
      MLDEBUG_PRINTLN(MirrorLink.moduleState);
    }

    // Restart packet reception
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    MirrorLink.status.enableInterrupt = (uint16_t)true;
  }

  return rxSuccessful;
}

void MirrorLinkEncodeMessage(byte *txArray, uint32_t *encryptedBuffer) {
  txArray[0] = (byte)(0xFF & encryptedBuffer[0] >> 24);
  txArray[1] = (byte)(0xFF & encryptedBuffer[0] >> 16);
  txArray[2] = (byte)(0xFF & encryptedBuffer[0] >> 8);
  txArray[3] = (byte)(0xFF & encryptedBuffer[0]);
  txArray[4] = (byte)(0xFF & encryptedBuffer[1] >> 24);
  txArray[5] = (byte)(0xFF & encryptedBuffer[1] >> 16);
  txArray[6] = (byte)(0xFF & encryptedBuffer[1] >> 8);
  txArray[7] = (byte)(0xFF & encryptedBuffer[1]);
}

void MirrorLinkDecodeMessage(byte *rxArray, uint32_t *encryptedBuffer) {
  encryptedBuffer[0] = (((uint32_t)(rxArray[0]) << 24) | ((uint32_t)(rxArray[1]) << 16) | ((uint32_t)(rxArray[2]) << 8) | ((uint32_t)(rxArray[3])));
  encryptedBuffer[1] = (((uint32_t)(rxArray[4]) << 24) | ((uint32_t)(rxArray[5]) << 16) | ((uint32_t)(rxArray[6]) << 8) | ((uint32_t)(rxArray[7])));
}

// Speck encryption function
void MirrorLinkSpeckEncrypt(uint32_t const pt[2], uint32_t ct[2], uint32_t const rk[MIRRORLINK_SPECK_ROUNDS])
{
  uint32_t i;
  ct[0]=pt[0]; ct[1]=pt[1];

  for(i = 0; i < MIRRORLINK_SPECK_ROUNDS; i++){
    ER(ct[1], ct[0], rk[i]);
  }
}

// Speck decryption function
void MirrorLinkSpeckDecrypt(uint32_t const ct[2], uint32_t pt[2], uint32_t const rk[MIRRORLINK_SPECK_ROUNDS])
{
  uint32_t i;
  pt[0]=ct[0]; pt[1]=ct[1];

  for(i = 0; i < MIRRORLINK_SPECK_ROUNDS; i++){
    DR(pt[1], pt[0], rk[(MIRRORLINK_SPECK_ROUNDS - 1) - i]);
  }
}

// Speck CTR encryption function
void MirrorLinkSpeckCtrEncrypt(uint32_t *encryptedBuffer, uint32_t *plainBuffer, uint32_t *nonceCtr, uint32_t *encryptionKeyExt) {
  // Message encryption
  MirrorLinkSpeckEncrypt(nonceCtr, encryptedBuffer, encryptionKeyExt);
  encryptedBuffer[0] ^= plainBuffer[0];
  encryptedBuffer[1] ^= plainBuffer[1];
}

// Speck CTR decryption function
void MirrorLinkSpeckCtrDecrypt(uint32_t *encryptedBuffer, uint32_t *decryptedBuffer, uint32_t *nonceCtr, uint32_t *decryptionKeyExt) {
  // Message decryption
  MirrorLinkSpeckEncrypt(nonceCtr, decryptedBuffer, decryptionKeyExt);
  decryptedBuffer[0] ^= encryptedBuffer[0];
  decryptedBuffer[1] ^= encryptedBuffer[1];
}

bool MirrorLinkCheckDecryptedMessage(uint32_t *decryptedBuffer) {
  bool decryptedSuccessful = false;
  bool networkIdMatch = false;

  // Network ID match flag
  networkIdMatch = (bool)(MirrorLink.status.networkId == (0xFF & (decryptedBuffer[0] >> 24)));

  // If reception and decryption successful
  if (  (MirrorLink.moduleState == ERR_NONE)
      &&(networkIdMatch) ) {
    decryptedSuccessful = true;
  }
  else {
    if (MirrorLink.moduleState != ERR_NONE) {
      // packet was received, but there was an issue in reception
      MLDEBUG_PRINTLN(F("Reception error!"));
    }
    else {
      // packet was received, but Network ID mismatch
      MLDEBUG_PRINTLN(F("Network ID mismatch!"));
      MLDEBUG_PRINTLN(MirrorLink.status.networkId);
      MLDEBUG_PRINTLN(0xFF & (decryptedBuffer[0] >> 24));
    }
  }

  return decryptedSuccessful;
}

// MirrorLink function to control the actions related to the stayalive counter
void MirrorLinkStayAliveControl(void) {
  // If stayalive control is active
  if (MirrorLink.status.stayAlive) {
    // If system currently enabled
    if (os.status.enabled) {
      // If timer reached
      if (millis() >= MirrorLink.stayAliveTimer) {
        // Disable system
        os.disable();
        // Set default channel
        MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
        // MirrorLink status down
        MirrorLink.status.link = ML_LINK_DOWN;
      }
    }
    else {
      // If timer not reached
      if (millis() < MirrorLink.stayAliveTimer) {
        // Enable system
        os.enable();
        // MirrorLink status up
        MirrorLink.status.link = ML_LINK_UP;
      }
    }
  }
}

// MirrorLink module state machine change function
void MirrorLinkState(void) {
  switch (MirrorLink.status.mirrorlinkState) {
    // Initial state
    case MIRRORLINK_INIT:
      MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_INIT"));
      if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
        if (MirrorLink.status.comStationState == (uint32_t)ML_LINK_COM_NORMAL) {
          MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_BUFFERING"));
          MirrorLink.sendTimer = millis();
        }
        else {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
          MirrorLink.sendTimer = millis();
        }
      }
      else {
        if (MirrorLink.status.comStationState == (uint32_t)ML_LINK_COM_NORMAL) {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_RECEIVE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
          MirrorLinkReceiveInit();
        }
        else {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
          MirrorLink.sendTimer = 0;
          MirrorLinkReceiveInit();
        }
      }
      break;
    // Association state
    case MIRRORLINK_ASSOCIATE:
    // Nonce update state 
    case MIRRORLINK_NONCEUPDATE:
      // Remote
      if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
        byte rxArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
        uint32_t decryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
        uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
        // Wait for answer
        if (MirrorLinkReceive(rxArray) == true) {
          bool packetOk = false;
          // Decode message
          MirrorLinkDecodeMessage(rxArray, encryptedBuffer);
          // If association packet, use normal block decryption with XOR of 0's
          if (MirrorLink.status.mirrorlinkState == MIRRORLINK_ASSOCIATE) {
            // Decrypt message
            MirrorLinkSpeckDecrypt(encryptedBuffer, decryptedBuffer, MirrorLinkSpeckKeyExp);
          }
          // Use Speck CTR streaming decryption
          else {
            // Increment nounceCtr
            MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
            // Decrypt message
            MirrorLinkSpeckCtrDecrypt(encryptedBuffer, decryptedBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          }
          packetOk = MirrorLinkCheckDecryptedMessage(decryptedBuffer);
          if (packetOk == true) {
            // Update second part of the key and update decryption key
            MirrorLink.nonce[1] = decryptedBuffer[1];
            // Update NonceCtr
            for (uint8_t i = 0; i < MIRRORLINK_SPECK_TEXT_LEN; i++) {
              MirrorLink.nonceCtr[i] = MirrorLink.nonce[i];
            }
            MirrorLink.assocRetryCtr = 0;
            MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_BUFFERING"));
            MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
            // Calculate transmission-free time based on duty cycle and time of last message
            MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
            MLDEBUG_PRINT(F("SendTimer: "));
            MLDEBUG_PRINTLN(MirrorLink.sendTimer);
            // Update Link status
            MirrorLink.status.comStationState = ML_LINK_COM_NORMAL;
            MirrorLink.status.link = ML_LINK_UP;
            // Change to next channel
            MirrorLink.status.channelNumber = MirrorLink.nextChannel;
          }
          else {
            MLDEBUG_PRINTLN(F("Packet decryption failed!"));
            MirrorLinkReceiveInit();
          }
        }
        // If timeout and nonce update or association has been attempted
        else if (  (MirrorLink.sendTimer <= millis())
                 && (MirrorLink.status.mirrorlinkState == MIRRORLINK_NONCEUPDATE)
                 && (MirrorLink.assocRetryCtr > 0) ) {
          // Report error
          MLDEBUG_PRINTLN(F("No answer received from station!"));
          MirrorLink.assocRetryCtr = 0;
          
          // Increase packet loss counter for current channel (the one the Station shall have used to transmit)
          // and old channel (the one the remote has used). As it is not known if issue was on remote or on station
          MirrorLink.packetsLost[MirrorLink.status.channelNumber]++;

          // Make sure we send an association command to station
          MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_ASSOCIATION;
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
          // Try with new channel (maybe the STATION received the message propertly but the answer did not arrive back to the REMOTE)
          MirrorLink.status.channelNumber = MirrorLink.nextChannel; //((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
          // Calculate transmission-free time based on duty cycle and time of last message
          MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          // Update Link status
          MirrorLink.status.link = ML_LINK_DOWN;
        }
        // If stayalive timer timeout the reset the channel to the default one
        else if (MirrorLink.stayAliveTimer <= millis()) {
          // Disable system
          //os.disable();
          // Set default channel
          MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
          // MirrorLink status down
          MirrorLink.status.link = ML_LINK_DOWN;
        }
      }
      // Station
      else {
        // If answer to association/nonce update sent correctly
        // Change state to receive
        if (MirrorLinkTransmitStatus() == true) {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_RECEIVE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
          MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_NORMAL;
          // Update channel number
          MirrorLink.status.channelNumber = MirrorLink.nextChannel;
          MirrorLinkReceiveInit();
          MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
          // Update Link status
          MirrorLink.status.link = ML_LINK_UP;
        }
      }
      break;
    // Buffering state
    case MIRRORLINK_BUFFERING:
      // If timer to be able to use the channel again is overdue
      if (MirrorLink.sendTimer <= millis()) {
        // If Key Renewal timer to be able to use the channel again is overdue
        byte txArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
        uint32_t plainBuffer[MIRRORLINK_SPECK_TEXT_LEN];
        uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
        uint8_t nextChannel = 0;
        // If nonce update is due
        if (MirrorLink.nonceUpdateTimer <= millis()) {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_NONCEUPDATE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_NONCEUPDATE;
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_DEAD_TIME * 1000);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          // Process header
          // Trigger a change of keys
          MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_NONCEUPDATE;
          // Reset key renewal timer
          MirrorLink.nonceUpdateTimer = millis() + (uint32_t)random(((uint32_t)MIRRORLINK_KEYCHANGE_MIN_TIME * 1000), ((uint32_t)MIRRORLINK_KEYCHANGE_MAX_TIME) * 1000);
          MirrorLink.status.powerCmd = MirrorLinkPowerCmdEncode();
          // Send next channel to station
          nextChannel = MirrorLinkSelectChannel();
          // if (MirrorLink.status.freqHopState == 1) {
          //   nextChannel = random(0, ML_CH_MAX);
          // }
          // else {
          //   nextChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
          // }
          plainBuffer[0] = ((((uint32_t)MirrorLink.status.networkId) << NETWORKID_POS) | ((((uint32_t)MirrorLink.status.comStationState) & 0x3) << STATE_POS) | ((((uint32_t)MirrorLink.status.powerCmd) & 0xF) << POWERCMD_POS) | ((((uint32_t)nextChannel) & 0xF) << CHNUMBER_POS) | ((((uint32_t)MirrorLink.boardSelected) & BOARDSEL_MASK) << BOARDSEL_REMOTE_POS));
          
          // Process payload
          // Generate first part of future nonce for the link
          MirrorLink.nonce[0] = (uint32_t)random(INT32_MAX);
          plainBuffer[1] = MirrorLink.nonce[0];

          // Increment nounceCtr
          MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
          // Encrypt plain message
          MirrorLinkSpeckCtrEncrypt(encryptedBuffer, plainBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          // Encode message
          MirrorLinkEncodeMessage(txArray, encryptedBuffer);

          // Send encrypted message
          MirrorLinkTransmit(txArray);

          // Update channel number
          MirrorLink.nextChannel = nextChannel;
        }
        // If buffer not empty
        // change state to send
        else if (MirrorLink.bufferedCommands > 0) {
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_SEND"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());

          // Process header
          MirrorLink.status.powerCmd = MirrorLinkPowerCmdEncode();
          // Send next channel to station
          nextChannel = MirrorLinkSelectChannel();
          // if (MirrorLink.status.freqHopState == 1) {
          //   nextChannel = random(0, ML_CH_MAX);
          // }
          // else {
          //   nextChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
          // }
          plainBuffer[0] = ((((uint32_t)MirrorLink.status.networkId) << 24) | ((((uint32_t)MirrorLink.status.comStationState) & 0x3) << 22) | ((((uint32_t)MirrorLink.status.powerCmd) & 0xF) << 18) | ((((uint32_t)nextChannel) & 0xF) << 14) | ((((uint32_t)MirrorLink.payloadBuffer[ML_CMD_1][MirrorLink.indexBufferTail]) & 0x3FFF)) | ((((uint32_t)MirrorLink.boardSelected) & BOARDSEL_MASK) << BOARDSEL_REMOTE_POS));

          // Process payload -> Nothing to do
          plainBuffer[1] = MirrorLink.payloadBuffer[ML_CMD_2][MirrorLink.indexBufferTail];

          // Increment nounceCtr
          MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
          // Encrypt plain message
          MirrorLinkSpeckCtrEncrypt(encryptedBuffer, plainBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          // Encode message
          MirrorLinkEncodeMessage(txArray, encryptedBuffer);

          // Send encrypted message
          MirrorLinkTransmit(txArray);

          MirrorLink.nextChannel = nextChannel;
        }
      }
      break;
    // Send state
    case MIRRORLINK_SEND:
      // If send process if finished
      // empty the command in the buffer
      // change state to receive
      if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
        if (MirrorLinkTransmitStatus() == true) {
          MLDEBUG_PRINT(F("Transmission duration: "));
          MLDEBUG_PRINTLN(MirrorLink.txTime);
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_RECEIVE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
          MirrorLinkReceiveInit();
        }
        else if (MirrorLink.sendTimer <= millis()) {
          // Calculate transmission-free time based on duty cycle and time of last message
          MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_BUFFERING"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
        }
      }
      // Station
      else {
        if (   (MirrorLinkTransmitStatus() == true)
            || (MirrorLink.sendTimer <= millis())) {
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_RECEIVE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_RECEIVE;
          // Update channel for next reception
          MirrorLink.status.channelNumber = MirrorLink.nextChannel;
          MirrorLinkReceiveInit();
        }
      }
      break; 
    // Receive state
    case MIRRORLINK_RECEIVE:
      // Remote
      if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
        byte rxArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
        // If confirmation of buffer commands received
        // change state to Buffering
        if (MirrorLinkReceive(rxArray) == true) {
          uint32_t decryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          bool packetOk = false;
          // Decode message
          MirrorLinkDecodeMessage(rxArray, encryptedBuffer);
          // Increment nounceCtr
          MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
          // Decrypt message
          MirrorLinkSpeckCtrDecrypt(encryptedBuffer, decryptedBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          packetOk = MirrorLinkCheckDecryptedMessage(decryptedBuffer);
          if (packetOk == true) {
            // Process header
            MirrorLink.responseCommand = (uint32_t)(decryptedBuffer[1] >> CMD_STATION_POS) & 0x3F;
            MirrorLink.cmdErrors = (uint32_t)(decryptedBuffer[1] >> APPERROR_STATION_POS) & 0xFF;
            MirrorLink.boardStatusBits[MirrorLink.boardSelected] = ((uint32_t)(decryptedBuffer[1] >> BOARDSTATUS_STATION_POS) & BOARDSTATUS_MASK);
            // Gather SNR and RSSI from station
            // Decode RSSI from 0.2 resolution in 8 bit (value range from -255dBm to 255dBm)
            MirrorLink.rssiRemote[MirrorLink.status.channelNumber] = (int16_t)(((decryptedBuffer[1] & RSSI_STATION_P2_MASK) >> RSSI_STATION_P2_POS) | ((decryptedBuffer[0] & RSSI_STATION_P1_MASK) << RSSI_STATION_P1_POS));
            if (decryptedBuffer[0] & RSSI_SIGN_STATION_P1_MASK) {
              MirrorLink.rssiRemote[MirrorLink.status.channelNumber] *= -1;
            }
            MirrorLink.rssiRemote[MirrorLink.status.channelNumber] *= 2;

            // Decode SNR from 0.2 resolution in 8 bit (value range from -25.55dB to 25.55dB)
            MirrorLink.snrRemote[MirrorLink.status.channelNumber] = (int16_t)((decryptedBuffer[0] & SNR_STATION_MASK) >> SNR_STATION_POS);

            if (decryptedBuffer[0] & SNR_SIGN_STATION_MASK) {
              MirrorLink.snrRemote[MirrorLink.status.channelNumber] *= -1;
            }
            MirrorLink.snrRemote[MirrorLink.status.channelNumber] *= 2;

            // Diagnostics
            // If response command different than last command sent then report error
            if (MirrorLink.responseCommand != ((MirrorLink.payloadBuffer[ML_CMD_1][MirrorLink.indexBufferTail] >> CMD_REMOTE_POS) & 0x3F)) {
              MLDEBUG_PRINTLN(F("Station response does not match command sent!"));
              MLDEBUG_PRINTLN(MirrorLink.responseCommand);
              MLDEBUG_PRINTLN((MirrorLink.payloadBuffer[ML_CMD_1][MirrorLink.indexBufferTail] >> 8) & 0x3F);
            }

            // Process payload
            // Response payload format

            // In case response shows a sync error between remote and station
            // delete all programs and reset response
            if ((MirrorLink.cmdErrors) == ML_SYNCERROR) {
              delete_program_data(-1);
              MLDEBUG_PRINTLN(F("Sync error with remote, reset program data!"));
            }

            // Station status bit diagnostic
            MLDEBUG_PRINT(F("STATION Board: "));
            MLDEBUG_PRINTLN(MirrorLink.boardSelected);
            MLDEBUG_PRINT(F("STATION status bits: "));
            MLDEBUG_PRINBIN(MirrorLink.boardStatusBits[MirrorLink.boardSelected]);
            MLDEBUG_PRINTLN(F(""));
            MirrorLink.boardSelected = (MirrorLink.boardSelected + 1) % os.nboards;

            // Report command sent
            if (MirrorLink.bufferedCommands > 0) {
              MirrorLink.bufferedCommands--;
              MirrorLink.indexBufferTail = (MirrorLink.indexBufferTail + 1) % MIRRORLINK_BUFFERLENGTH;
            }
            
            MirrorLink.responseCommand = 0;
            //MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
            MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_BUFFERING"));
            MirrorLink.status.mirrorlinkState = MIRRORLINK_BUFFERING;
            // Calculate transmission-free time based on duty cycle and time of last message
            MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);

            // Update Link status and stayAliveTimer
            MirrorLink.status.link = ML_LINK_UP;
            MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;

            // Update channel for next packet to be sent
            MirrorLink.status.channelNumber = MirrorLink.nextChannel;
          }
          // Else if command received but not propertly decrypted go to association mode
          else {
            // Report error
            MLDEBUG_PRINTLN(F("Packet decryption failed!"));
            MirrorLinkReceiveInit();

            // Command is lost, do not retry -> Changed, it will be retried until successfully sent
            // if (MirrorLink.bufferedCommands > 0) {
            //   MirrorLink.bufferedCommands--;
            //   (MirrorLink.indexBufferTail++) % MIRRORLINK_BUFFERLENGTH;
            // }

            MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
            MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
            MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
            // Use again the next channel planned
            MirrorLink.status.channelNumber = MirrorLink.nextChannel;
            //MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
            // Calculate transmission-free time based on duty cycle and time of last message
            MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);

            // Update Link status
            MirrorLink.status.link = ML_LINK_DOWN;
            MirrorLink.snrRemote[MirrorLink.status.channelNumber] = -200;
            MirrorLink.rssiRemote[MirrorLink.status.channelNumber] = -200;

            // Request change state to association for station
            MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_ASSOCIATION;
          }
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
        }

        // If timeout
        // Report error and change state as well to association
        if(MirrorLink.sendTimer <= millis()) {

          // Report error
          MLDEBUG_PRINTLN(F("No answer received from station!"));

          // Increase packet loss counter for current channel (the one the Station shall have used to transmit)
          // and old channel (the one the remote has used). As it is not known if issue was on remote or on station
          MirrorLink.packetsLost[MirrorLink.status.channelNumber]++;

          // Command is lost, do not retry
          //if (MirrorLink.bufferedCommands > 0) {
          //  MirrorLink.bufferedCommands--;
          //  MirrorLink.indexBufferTail = ((MirrorLink.indexBufferTail + 1) % MIRRORLINK_BUFFERLENGTH);
          //}
          
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
          MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
          MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
          MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
          // Calculate transmission-free time based on duty cycle and time of last message
          MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());

          // Update Link status
          MirrorLink.status.link = ML_LINK_DOWN;
          MirrorLink.snrRemote[MirrorLink.status.channelNumber] = -200;
          MirrorLink.rssiRemote[MirrorLink.status.channelNumber] = -200;

          // Request change state to association for station
          MirrorLink.status.comStationState = (uint32_t)ML_LINK_COM_ASSOCIATION;
        }
      }
      // Station
      else {
        byte rxArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
        // If commands received
        // execute and change state to Send
        if (MirrorLinkReceive(rxArray) == true) {
          uint32_t decryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          bool packetOk = false;
          // Decode message
          MirrorLinkDecodeMessage(rxArray, encryptedBuffer);
          // Increment nounceCtr
          MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
          // Decrypt message
          MirrorLinkSpeckCtrDecrypt(encryptedBuffer, decryptedBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          packetOk = MirrorLinkCheckDecryptedMessage(decryptedBuffer);
          if (packetOk == true) {
            // Process header
            MirrorLink.status.comStationState = ((decryptedBuffer[ML_CMD_1] & STATE_MASK) >> STATE_POS);
            MirrorLink.status.powerCmd = ((decryptedBuffer[ML_CMD_1] & POWERCMD_MASK) >> POWERCMD_POS);
            MirrorLink.nextChannel = ((decryptedBuffer[ML_CMD_1] & CHNUMBER_MASK) >> CHNUMBER_POS);
            MirrorLink.command[ML_CMD_1] = ((uint32_t)decryptedBuffer[ML_CMD_1] & (CMD_REMOTE_MASK | PAYLOAD1_REMOTE_MASK));
            MirrorLink.command[ML_CMD_2] = (uint32_t)decryptedBuffer[ML_CMD_2];
            MLDEBUG_PRINT(F("Channel received:\t\t"));
            MLDEBUG_PRINTLN(MirrorLink.nextChannel);
            MLDEBUG_PRINT(F("PowerCMD received:\t\t"));
            #if defined(ENABLE_DEBUG_MIRRORLINK) /** Serial debug functions */
            // Calculate power level based on local max and powerCmd received from remote
            uint8_t powerCmd = MirrorLink.status.powerCmd;
            int16_t powerLevel = (10 * ((powerCmd & 0x4) >> 2)) + (5 * ((powerCmd & 0x2) >> 1)) + (powerCmd & 0x1);
            if (powerCmd & 0x8) powerLevel *= -1;
            powerLevel += MirrorLink.powerLevel;
            #endif
            MLDEBUG_PRINTLN(powerLevel);
            
            // Process payload
            if (MirrorLink.status.comStationState == (uint32_t)ML_LINK_COM_NORMAL) {
              uint32_t payload[ML_CMD_MAX];
              uint16_t timer = 0;
              int16_t pid = 0;
              uint16_t duration = 0;
              byte sid = 0;
              uint8_t en = 0;
              uint8_t stTimeNum = 0;
              uint8_t usw = 0;
              uint8_t oddeven = 0;
              uint8_t addProg = 0;
              uint8_t command = 0;
              char * latitude;
              char * longitude;
              MirrorLink.cmdErrors = ML_NO_ERROR;
              const float weight = 180./(1 << 23);
              if (MirrorLink.command[ML_CMD_1] != 0) {
                command = (MirrorLink.command[ML_CMD_1] >> CMD_REMOTE_POS);
                // Execute command
                switch (command) {
                  // Initial state
                  case ML_TESTSTATION:
                    // Message format: 
                    // bit 0 = status (1 = On, 0 = Off)
                    // bit 1 to 8 = sid
                    // bit 9 to 24 = time(sec)
                    // bit 25 to 31 = Not used
                    MirrorLink.lastCmd = ML_NO_CMD;
                    MirrorLinkGetCmd((uint8_t)ML_TESTSTATION, payload);
                    timer = (uint16_t) ((0xFFFF) & (payload[ML_CMD_2] >> 9));
                    sid = (byte) (0xFF & (payload[ML_CMD_2] >> 1));
                    if (!((os.status.mas==sid+1) || (os.status.mas2==sid+1))) {
                      en = (uint8_t) (payload[ML_CMD_2] & 0x1);
                      if (en) {
                        RuntimeQueueStruct *q = NULL;
                        byte sqi = pd.station_qid[sid];
                        // check if the station already has a schedule
                        if (sqi!=0xFF) {	// if we, we will overwrite the schedule
                          q = pd.queue+sqi;
                        } else {	// otherwise create a new queue element
                          q = pd.enqueue();
                        }
                        // if the queue is not full
                        if (q) {
                          q->st = 0;
                          q->dur = timer;
                          q->sid = sid;
                          q->pid = 99;	// testing stations are assigned program index 99

                          schedule_all_stations(os.now_tz());
                        }
                      } else {	// turn off station
                        turn_off_station(sid, os.now_tz());
                      }
                    }
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_TESTSTATION"));
                    break;
                  case ML_PROGRAMADDDEL:
                    // Message format: 
                    // bit 0 to 6 = program number (max. is 40)
                    // bit 7 = Add (1) or delete (0)
                    // bit 8 to 31 = Not used
                    // If command is not repeated then create program
                    if (MirrorLink.lastCmd == ML_NO_CMD) {
                      MirrorLink.lastCmd = ML_PROGRAMADDDEL;
                      MirrorLinkGetCmd((uint8_t)ML_PROGRAMADDDEL, payload);
                      addProg = ((payload[ML_CMD_2] >> 7) & 0x1);
                      pid = (int16_t) (0x7F & payload[ML_CMD_2]);
                      sprintf_P(mirrorlinkProg.name, "%d", pid);

                      // If the request is to add (or modify) a program
                      if (addProg)
                      {
                        MLDEBUG_PRINT(F("Add program: "));
                        MLDEBUG_PRINTLN(pid);
                        MLDEBUG_PRINT(F("Number of programs: "));
                        MLDEBUG_PRINTLN(pd.nprograms);
                        // In case the new pid does not match the max. program number
                        // SYNC issue identified, remove all programs
                        if (pid != pd.nprograms) {
                          // Delete all programs
                          delete_program_data(-1);
                          MirrorLink.cmdErrors = ML_SYNCERROR;
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
                        MLDEBUG_PRINT(F("Remove program/s: "));
                        MLDEBUG_PRINTLN(pid);
                        MLDEBUG_PRINT(F("Number of programs: "));
                        MLDEBUG_PRINTLN(pd.nprograms);
                        // In case the new pid to be removed is not within the available pid's range
                        // SYNC issue identified, remove all programs
                        if (pid >= pd.nprograms) {
                          // Delete all programs
                          pd.eraseall();
                          //delete_program_data(-1);
                          MirrorLink.cmdErrors = ML_SYNCERROR;
                        }
                        // Otherwise delete the program
                        else {
                          pd.del(pid);
                          //delete_program_data(pid);
                        }
                      }
                      MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                      MLDEBUG_PRINTLN(F("COMMAND: ML_PROGRAMADDDEL"));
                    }
                    break;
                  case ML_PROGRAMMAINSETUP:
                    // Message format:
                    // bit 0 to 6 = program number (max. is 40)
                    // bit 7 = enable/disable
                    // bit 8 = use weather
                    // bit 9 to 10 = Odd/even restriction
                    // bit 11 to 12 = schedule type
                    // bit 13 to 20 = Number of programs in remote
                    // bit 21 to 31 = Not used
                    if (MirrorLink.lastCmd == ML_PROGRAMDAYS) {
                      MirrorLink.lastCmd = ML_NO_CMD;
                      MirrorLinkGetCmd((uint8_t)ML_PROGRAMMAINSETUP, payload);
                      pid = (int16_t) (0x7F & payload[ML_CMD_2]);
                      mirrorlinkProg.enabled = (uint8_t)(0x1 & (payload[ML_CMD_2] >> 7));
                      mirrorlinkProg.use_weather = (uint8_t) (0x1 & (payload[ML_CMD_2] >> 8));
                      mirrorlinkProg.oddeven = (uint8_t) (0x3 & (payload[ML_CMD_2] >> 9));
                      mirrorlinkProg.type = (byte) (0x03 & (payload[ML_CMD_2] >> 11));
                      MirrorLink.maxProgrNum = (uint8_t) (0xFF & (payload[ML_CMD_2] >> 13));
                      sprintf_P(mirrorlinkProg.name, "%d", pid);
                      change_program_data(pid, pd.nprograms, &mirrorlinkProg);
                      // In case the remote max. prog. number does not match the max. program number of the station
                      // SYNC issue identified, remove all programs and report it
                      if (MirrorLink.maxProgrNum != pd.nprograms) {
                        // Delete all programs
                        delete_program_data(-1);
                        MirrorLink.cmdErrors = ML_SYNCERROR;
                      }
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

                      MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                      MLDEBUG_PRINTLN(F("COMMAND: ML_PROGRAMMAINSETUP"));
                    }
                    break;
                  case ML_PROGRAMDAYS:
                    // Message format:
                    // bit 0 to 6 = program number (max. is 40)
                    // bit 7 to 22 = days
                    // bit 23 to 31 = Not used
                    if (MirrorLink.lastCmd == ML_PROGRAMDURATION) {
                      MirrorLink.lastCmd = ML_PROGRAMDAYS;
                      MirrorLinkGetCmd((uint8_t)ML_PROGRAMDAYS, payload);
                      pid = (int16_t) (0x7F & payload[ML_CMD_2]);
                      mirrorlinkProg.days[0] = (byte) (0xFF & (payload[ML_CMD_2] >> 15));
                      mirrorlinkProg.days[1] = (byte) (0xFF & (payload[ML_CMD_2] >> 7));
                      MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                      MLDEBUG_PRINTLN(F("COMMAND: ML_PROGRAMDAYS"));
                    }
                    break;
                  case ML_PROGRAMSTARTTIME:
                    // Message format:
                    // bit 0 to 6 = program number (max. is 40)
                    // bit 7 to 8 = start time number (max. is 4 for each program)
                    // bit 9 to 24 = start time
                    // bit 25 = Starttime type
                    // bit 31 = Not used
                    MirrorLink.lastCmd = ML_PROGRAMSTARTTIME;
                    MirrorLinkGetCmd((uint8_t)ML_PROGRAMSTARTTIME, payload);
                    pid = (int16_t)(payload[ML_CMD_2] & 0x7F);
                    stTimeNum = (uint8_t)((payload[ML_CMD_2] >> 7) & 0x3);
                    mirrorlinkProg.starttimes[stTimeNum] = (int16_t)((payload[ML_CMD_2] >> 9) & 0xFFFF);
                    mirrorlinkProg.starttime_type = (uint8_t)((payload[ML_CMD_2] >> 25) & 0x1);
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_PROGRAMSTARTTIME"));
                  case ML_PROGRAMDURATION:
                    // Message format:
                    // bit 0 to 6 = program number (max. is 40)
                    // bit 7 to 14 = sid
                    // bit 15 to 25 = time (min)
                    // bit 31 = Not used
                    MirrorLink.lastCmd = ML_PROGRAMDURATION;
                    MirrorLinkGetCmd((uint8_t)ML_PROGRAMDURATION, payload);
                    pid = (int16_t)(payload[ML_CMD_2] & 0x7F);
                    sid = (byte)((payload[ML_CMD_2] >> 7) & 0xFF);
                    mirrorlinkProg.durations[sid] = (uint16_t)(60 * ((payload[ML_CMD_2] >> 15) & 0x7FF));
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_PROGRAMDURATION"));
                    break;
                  case ML_TIMESYNC:
                    // Message format:
                    // bit 0 to 26 = Unix Timestamp in minutes! not seconds
                    // bit 27 to 31 = Not used
                    MirrorLink.lastCmd = ML_NO_CMD;
                    MirrorLinkGetCmd((uint8_t)ML_TIMESYNC, payload);
                    setTime((time_t)(60*(0x7FFFFFF & payload[ML_CMD_2])));
                    RTC.set((time_t)(60*(0x7FFFFFF & payload[ML_CMD_2])));
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_TIMESYNC"));
                    break;
                  case ML_TIMEZONESYNC:
                    // Payload format:
                    // bit 0 to 7 = Time zone
                    // bit 8 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_TIMEZONESYNC, payload);
                    os.iopts[IOPT_TIMEZONE] = (byte)(0xFF & payload[ML_CMD_2]);
                    os.iopts[IOPT_USE_NTP] = 0;
                    os.iopts_save();
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_TIMEZONESYNC"));
                    break;
                  case ML_CURRENTREQUEST:
                    // TODO:
                    MirrorLinkGetCmd((uint8_t)ML_CURRENTREQUEST, payload);
                    MLDEBUG_PRINTLN(F("COMMAND: ML_CURRENTREQUEST"));
                    break;
                  case ML_EMERGENCYSHUTDOWN:
                    // TODO:
                    MirrorLinkGetCmd((uint8_t)ML_EMERGENCYSHUTDOWN, payload);
                    MLDEBUG_PRINTLN(F("COMMAND: ML_EMERGENCYSHUTDOWN"));
                    break;
                  case ML_STATIONREBOOT:
                    // Payload format:
                    // bit 0 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_STATIONREBOOT, payload);
                    MirrorLink.status.rebootRequest = (uint16_t)true;
                    os.reboot_dev(REBOOT_CAUSE_MIRRORLINK);
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_STATIONREBOOT"));
                    break;
                  case ML_LATITUDE:
                    // Payload format: 
                    // bit 0 to 24 = latitude (bit 24 for sign)
                    // bit 24 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_LATITUDE, payload);
                    MirrorLink.latitude = (int32_t)(0xFFFFFF & payload[ML_CMD_2]);
                    if (payload[ML_CMD_2] & 0x1000000) MirrorLink.latitude *= -1;
                    sprintf_P(tmp_buffer, PSTR("%f,%f"), (float) (weight * ((float)MirrorLink.latitude- 0.5f)), (float) (weight * ((float)MirrorLink.longitude- 0.5f)));
                    os.sopt_save(SOPT_LOCATION, tmp_buffer);
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_LATITUDE"));
                    break;
                  case ML_LONGITUDE:
                    // Payload format: 
                    // bit 0 to 23 = longitude
                    // bit 24 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_LONGITUDE, payload);
                    MirrorLink.longitude = (int32_t)(0xFFFFFF & payload[ML_CMD_2]);
                    if (payload[ML_CMD_2] & 0x1000000) MirrorLink.longitude *= -1;
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_LONGITUDE"));
                    break;
                  case ML_SUNRISE:
                    // Payload format: 
                    // bit 0 to 15 = sunrise time
                    // bit 16 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_SUNRISE, payload);
                    os.nvdata.sunrise_time = payload[ML_CMD_2];
                    os.nvdata_save();
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_SUNRISE"));
                    break;
                  case ML_SUNSET:
                    // Payload format: 
                    // bit 0 to 15 = sunset time
                    // bit 16 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_SUNSET, payload);
                    os.nvdata.sunset_time = payload[ML_CMD_2];
                    os.nvdata_save();
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_SUNSET"));
                    break;
                  case ML_RAINDELAYSTOPTIME:
                    // TODO:
                    MirrorLinkGetCmd((uint8_t)ML_RAINDELAYSTOPTIME, payload);
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_RAINDELAYSTOPTIME"));
                    break;
                  case ML_STAYALIVE:
                    // Payload format: 
                    // bit 0 to 25 = Stayalive configured counter
                    // bit 26 = Bit indicating if stayalive feature shall be used (true) or ignored (false)
                    // bit 27 to 31 = Not used
                    MirrorLinkGetCmd((uint8_t)ML_STAYALIVE, payload);
                    MirrorLink.status.stayAlive = (uint8_t)((payload[ML_CMD_2] >> 26) & 0x01);
                    MirrorLink.stayAliveMaxPeriod = ((uint32_t)(payload[ML_CMD_2] & 0x3FFFFFF) * 1000);
                    MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
                    MLDEBUG_PRINTLN(F("COMMAND: ML_STAYALIVE"));
                    break;
                }
              }
              MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
              MLDEBUG_PRINT(F("SendTimer: "));
              MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
              MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_SEND"));
              MirrorLink.status.mirrorlinkState = MIRRORLINK_SEND;
              // Delay to allow the remote to turn to rx mode
              delay(100);
            }
            // If the command received requests a key change
            // then go to change key
            else if (MirrorLink.status.comStationState == ML_LINK_COM_NONCEUPDATE) {
              MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_NONCEUPDATE"));
              MirrorLink.status.mirrorlinkState = MIRRORLINK_NONCEUPDATE;
              // Update first part of the nonce
              MirrorLink.nonce[0] = decryptedBuffer[1];
              // Reset Stayalive Timer
              MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
              MirrorLink.assocRetryCtr = 0;
              MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_DEAD_TIME * 1000);
            }
            // If the command received requests an association
            // then go to association
            else if (MirrorLink.status.comStationState == ML_LINK_COM_ASSOCIATION) {
              MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
              MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
              // Update first part of the nonce
              MirrorLink.nonce[0] = decryptedBuffer[ML_CMD_2];
              // Reset Stayalive Timer
              MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
              MirrorLink.assocRetryCtr = 0;
              MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_DEAD_TIME * 1000);
              MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
            }
            // Update Link status
            MirrorLink.status.link = ML_LINK_UP;
            MLDEBUG_PRINT(F("SendTimer: "));
            MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
          }
          // If message decryption unsuccessful
          // then check if association message
          else {
            // Check if association message
            // Decrypt plain message
            MirrorLinkSpeckDecrypt(encryptedBuffer, decryptedBuffer, MirrorLinkSpeckKeyExp);
            packetOk = MirrorLinkCheckDecryptedMessage(decryptedBuffer);
            if (packetOk == true) {
              // Check if association or renewval command from station
              MirrorLink.status.comStationState = ((decryptedBuffer[0] & 0xC00000) >> STATE_POS);
              if (MirrorLink.status.comStationState == ML_LINK_COM_ASSOCIATION) {
                MirrorLink.status.powerCmd = ((decryptedBuffer[0] & 0x3C0000) >> POWERCMD_POS);
                MirrorLink.nextChannel = ((decryptedBuffer[0] & 0x3C000) >> CHNUMBER_POS);
                // Update first part of the nonce
                MirrorLink.nonce[0] = decryptedBuffer[1];
                // Start transmission timer
                MirrorLink.sendTimer = (millis() + ((uint32_t)MIRRORLINK_RXTX_DEAD_TIME * 1000));
                MLDEBUG_PRINT(F("SendTimer: "));
                MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
                // Reset Stayalive Timer
                MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
              }
              else {
                MLDEBUG_PRINTLN(F("Error, association message w/o proper command!"));
              }
            }
            else {
              MLDEBUG_PRINTLN(F("Packet decryption failed!"));
              MirrorLinkReceiveInit();
            }
            MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
            MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
          }
        }
        // If reception timeout then go to association
        else if (MirrorLink.stayAliveTimer <= millis()) {
            MLDEBUG_PRINTLN(F("STATE: MIRRORLINK_ASSOCIATE"));
            MirrorLink.status.mirrorlinkState = MIRRORLINK_ASSOCIATE;
            MirrorLink.status.channelNumber = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
            MirrorLink.sendTimer = 0;
            MirrorLinkReceiveInit();
            // Update Link status
            MirrorLink.status.link = ML_LINK_DOWN;
        }
        MirrorLinkStayAliveControl();
      }
      break;
  }
}

// MirrorLink module state maction actions function, called once every second
void MirrorLinkWork(void) {
  switch (MirrorLink.status.mirrorlinkState) {
    // Initial state
    case MIRRORLINK_INIT:
      // Do nothing
      if (MirrorLink.status.mirrorLinkStationType == ML_STATION) {
        MirrorLinkStayAliveControl();
      }
      break;
    // Association state 
    case MIRRORLINK_ASSOCIATE:
    // Nonce update state 
    case MIRRORLINK_NONCEUPDATE:
      // Remote
      if (MirrorLink.status.mirrorLinkStationType == ML_REMOTE) {
        // If timeout send association request
        if (MirrorLink.sendTimer <= millis()) {
          uint8_t nextChannel = 0;
          byte txArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
          uint32_t plainBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];

          // Update channel
          MirrorLink.status.channelNumber = MirrorLink.nextChannel;

          // Send association request
          // Process header
          // Calculate Power Level to station
          MirrorLink.status.powerCmd = MirrorLinkPowerCmdEncode();
          // Increase number of association attempts
          MirrorLink.associationAttempts++;
          // If not associated
          // Set channel and command association from station
          if (MirrorLink.status.mirrorlinkState == (uint32_t)MIRRORLINK_ASSOCIATE) {
            if (MirrorLink.assocRetryCtr < ML_CH_MAX) {
              nextChannel = ((MirrorLink.status.channelNumber + 1) % ML_CH_MAX);
#if defined(MIRRORLINK_ENABLE_BLACKLIST)
              while (MirrorLink.bannedChannelTimer[nextChannel] > (millis() / 1000)) nextChannel = ((nextChannel + 1) % ML_CH_MAX);
#endif
            }
            else {
              //nextChannel = ((((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF) + 1) % ML_CH_MAX);
              nextChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
              // Reset assocRetryCtr
              MirrorLink.assocRetryCtr = 0;
            }
            MirrorLink.status.comStationState = ML_LINK_COM_ASSOCIATION;
          }
          // Set NonceUpdate channel and command nonce update from station
          else {
            nextChannel = MirrorLinkSelectChannel();
            // if (MirrorLink.status.freqHopState == 1) {
            //   nextChannel = random(0, ML_CH_MAX);
            // }
            // else {
            //   nextChannel = ((uint32_t)(os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF);
            // }
            MirrorLink.status.comStationState = ML_LINK_COM_NONCEUPDATE;
          }
          // Set send timer control
          MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());

          plainBuffer[0] = ((((uint32_t)MirrorLink.status.networkId) << NETWORKID_POS) | ((((uint32_t)MirrorLink.status.comStationState) & 0x3) << STATE_POS) | ((((uint32_t)MirrorLink.status.powerCmd) & 0xF) << POWERCMD_POS) | ((((uint32_t)nextChannel) & 0xF) << CHNUMBER_POS) | (random(INT32_MAX) & 0x3FFF));

          // Process payload
          // Generate first part of future random nonce for the link
          MirrorLink.nonce[0] = (uint32_t)random(INT32_MAX);
          plainBuffer[1] = MirrorLink.nonce[0];

          if (MirrorLink.status.mirrorlinkState == MIRRORLINK_ASSOCIATE) {
            // Encrypt plain message
            MirrorLinkSpeckEncrypt(plainBuffer, encryptedBuffer, MirrorLinkSpeckKeyExp);
          }
          else {
            // Increment nounceCtr
            MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
            // Encrypt plain message
            MirrorLinkSpeckCtrEncrypt(encryptedBuffer, plainBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          }
          // Encode message
          MirrorLinkEncodeMessage(txArray, encryptedBuffer);

          // Send encrypted message
          MirrorLinkTransmit(txArray);

          // Update channel number
          MirrorLink.nextChannel = nextChannel;
        }
        // If association/nonce update request sent
        else if (MirrorLinkTransmitStatus() == true) {
          // First attempt made
          MirrorLink.assocRetryCtr++;
          // Wait for answer to association/change of keys request command
          MirrorLinkReceiveInit();
          if (MirrorLink.status.mirrorlinkState == MIRRORLINK_ASSOCIATE) {
            if (MirrorLink.assocRetryCtr < ML_CH_MAX) {
            // if (MirrorLink.status.channelNumber != ((((os.iopts[IOPT_ML_DEFCHANNEL]) & 0xF) + (ML_CH_MAX - 1)) % ML_CH_MAX)) {
              MirrorLink.txTime = (lora.getTimeOnAir(8) / 1000);
              // Set send timer control
              MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
              //MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
            // }
            }
            else {
              // Set send timer control
              MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_STAYALIVE_PERIOD * 1000);
            }
          }
          else {
            // Calculate transmission-free time based on duty cycle and time of last message
            MirrorLink.sendTimer = millis() + ((uint32_t)MIRRORLINK_RXTX_MAX_TIME * 1000);
            //MirrorLink.sendTimer = millis() + (((MirrorLink.txTime * 2) * (10000 / (MirrorLink.dutyCycle))) / 10);
          }
          // Update channel
          MirrorLink.status.channelNumber = MirrorLink.nextChannel;
          MLDEBUG_PRINT(F("SendTimer: "));
          MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
        }
      }
      // Station
      else {
        byte rxArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
        // If message received and decoded propertly (it could be association or renewval)
        if (MirrorLinkReceive(rxArray) == true) {
          uint32_t decryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          bool packetOk = false;
          // Decode message
          MirrorLinkDecodeMessage(rxArray, encryptedBuffer);
          if (MirrorLink.status.mirrorlinkState == MIRRORLINK_ASSOCIATE) {
            // Decrypt plain message
            MirrorLinkSpeckDecrypt(encryptedBuffer, decryptedBuffer, MirrorLinkSpeckKeyExp);
          }
          else {
            // Increment nounceCtr
            MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
            // Decrypt plain message
            MirrorLinkSpeckCtrDecrypt(encryptedBuffer, decryptedBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          }

          packetOk = MirrorLinkCheckDecryptedMessage(decryptedBuffer);
          if (packetOk == true) {
            // Check if association or renewval command from station
            MirrorLink.status.comStationState = ((decryptedBuffer[0] & 0xC00000) >> STATE_POS);
            if (   (MirrorLink.status.comStationState == ML_LINK_COM_ASSOCIATION)
                || (MirrorLink.status.comStationState == ML_LINK_COM_NONCEUPDATE) ) {
              MirrorLink.status.powerCmd = ((decryptedBuffer[0] & 0x3C0000) >> POWERCMD_POS);
              MirrorLink.nextChannel = ((decryptedBuffer[0] & 0x3C000) >> CHNUMBER_POS);
              // Update first part of the nonce
              MirrorLink.nonce[0] = decryptedBuffer[1];
              // Start transmission timer
              MirrorLink.sendTimer = (millis() + ((uint32_t)MIRRORLINK_RXTX_DEAD_TIME * 1000));
              MLDEBUG_PRINT(F("SendTimer: "));
              MLDEBUG_PRINTLN(MirrorLink.sendTimer - millis());
              // Reset Stayalive Timer
              MirrorLink.stayAliveTimer = millis() + MirrorLink.stayAliveMaxPeriod;
            }
            else {
              MLDEBUG_PRINTLN(F("Remote state is not association/renewval!"));
            }
          }
          else {
            MLDEBUG_PRINTLN(F("Packet decryption failed!"));
            MirrorLinkReceiveInit();
          }
        }
        // Else if transmission timer is due
        else if (  ((MirrorLink.sendTimer / 1000) == (millis() / 1000))
                && (MirrorLink.status.flagRxTx == ML_RECEIVING)) {
          byte txArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
          uint32_t plainBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          if (MirrorLink.status.comStationState == (uint32_t)ML_LINK_COM_ASSOCIATION) MirrorLink.associationAttempts++;

          // Process header
          // If association request sent
          plainBuffer[0] = ((((uint32_t)MirrorLink.status.networkId) << NETWORKID_POS) | ((((uint32_t)MirrorLink.status.comStationState) & 0x3) << STATE_POS) | ((((uint32_t)MirrorLink.status.powerCmd) & 0xF) << POWERCMD_POS) | ((((uint32_t)MirrorLink.status.channelNumber) & 0xF) << CHNUMBER_POS));

          // Process payload
          // Generate second part of future nonce for the link
          MirrorLink.nonce[1] = (uint32_t)random(INT32_MAX);
          plainBuffer[1] = MirrorLink.nonce[1];

          if (MirrorLink.status.mirrorlinkState == MIRRORLINK_ASSOCIATE) {
            // Encrypt plain message
            MirrorLinkSpeckEncrypt(plainBuffer, encryptedBuffer, MirrorLinkSpeckKeyExp);
          }
          else {
            // Increment nounceCtr
            MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
            // Encrypt plain message
            MirrorLinkSpeckCtrEncrypt(encryptedBuffer, plainBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);
          }
          // Encode message
          MirrorLinkEncodeMessage(txArray, encryptedBuffer);

          // Send encrypted message
          MirrorLinkTransmit(txArray);

          // Update NonceCtr
          for (uint8_t i = 0; i < MIRRORLINK_SPECK_TEXT_LEN; i++) {
            MirrorLink.nonceCtr[i] = MirrorLink.nonce[i];
          }
        }
        MirrorLinkStayAliveControl();
      }
      break;
    // Buffering state
    case MIRRORLINK_BUFFERING:
      // Nothing to do
      break;
    // Send state
    case MIRRORLINK_SEND:
      // Station
      if (MirrorLink.status.mirrorLinkStationType == ML_STATION) {
        // Send answer if not yet transmitted
        if (MirrorLink.status.flagRxTx == ML_RECEIVING) {
          byte txArray[MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH];
          uint32_t plainBuffer[MIRRORLINK_SPECK_TEXT_LEN];
          uint32_t encryptedBuffer[MIRRORLINK_SPECK_TEXT_LEN];

          // Process header
          // Encode RSSI to 0.2 resolution in 8 bit (value range from -255dBm to 255dBm)
          uint8_t rssi, snr;
          if (MirrorLink.rssiLocal[MirrorLink.status.channelNumber] >= 0) {
            if ((MirrorLink.rssiLocal[MirrorLink.status.channelNumber] / 2) > 127) {
              rssi = (uint8_t)127;
            }
            else {
              rssi = (uint8_t)(MirrorLink.rssiLocal[MirrorLink.status.channelNumber] / 2);
            }
          } 
          else {
            if ((MirrorLink.rssiLocal[MirrorLink.status.channelNumber] / 2) < -127) {
              rssi = ((uint8_t)127 | 0x80);
            }
            else {
              rssi = (((uint8_t)((MirrorLink.rssiLocal[MirrorLink.status.channelNumber] / 2) * -1)) | 0x80);
            }
          }

          // Encode SNR to 0.2 resolution in 8 bit (value range from -25.55dB to 25.55dB)
          if (MirrorLink.snrLocal[MirrorLink.status.channelNumber] >= 0) {
            if ((MirrorLink.snrLocal[MirrorLink.status.channelNumber] / 2) > 127) {
              snr = (uint8_t)127;
            }
            else {
              snr = (uint8_t)(MirrorLink.snrLocal[MirrorLink.status.channelNumber] / 2);
            }
          } 
          else {
            if ((MirrorLink.snrLocal[MirrorLink.status.channelNumber] / 2) < -127) {
              snr = ((uint8_t)127 | 0x80);
            }
            else {
              snr = (((uint8_t)((MirrorLink.snrLocal[MirrorLink.status.channelNumber] / 2) * -1)) | 0x80);
            }
          }
          plainBuffer[0] = ((((uint32_t)MirrorLink.status.networkId) << NETWORKID_POS) | ((((uint32_t)MirrorLink.status.comStationState) & 0x3) << STATE_POS) | ((((uint32_t)MirrorLink.status.powerCmd) & 0xF) << POWERCMD_POS) | ((((uint32_t)MirrorLink.status.channelNumber) & 0xF) << CHNUMBER_POS) | ((uint32_t)(snr & 0xFF) << SNR_STATION_POS) | ((uint32_t)(rssi & 0xFF) >> RSSI_STATION_P1_POS));

          // Send status bits of selected board from remote for diagnostic
          MirrorLink.boardStatusBits[MirrorLink.boardSelected] = os.station_bits[MirrorLink.boardSelected];
          MLDEBUG_PRINT(F("STATION Board: "));
          MLDEBUG_PRINTLN(MirrorLink.boardSelected);
          MLDEBUG_PRINT(F("STATION status bits: "));
          MLDEBUG_PRINBIN(MirrorLink.boardStatusBits[MirrorLink.boardSelected]);
          MLDEBUG_PRINTLN(F(""));

          // Process payload
          plainBuffer[1] = (((uint32_t)(rssi & 0xFF) << RSSI_STATION_P2_POS) | ((uint32_t)(MirrorLink.command[ML_CMD_2] & 0x3F000000)) | (((uint32_t)ML_SYNCERROR) << APPERROR_STATION_POS) | ((MirrorLink.boardStatusBits[MirrorLink.boardSelected] & BOARDSTATUS_MASK) << BOARDSTATUS_STATION_POS));
          MirrorLink.command[ML_CMD_1] = 0;
          MirrorLink.command[ML_CMD_2] = 0;

          // Increment nounceCtr
          MirrorLinkIncrementNonceCtr(MirrorLink.nonceCtr);
          // Encrypt plain message
          MirrorLinkSpeckCtrEncrypt(encryptedBuffer, plainBuffer, MirrorLink.nonceCtr, MirrorLinkSpeckKeyExp);

          // Encode message
          MirrorLinkEncodeMessage(txArray, encryptedBuffer);

          // Send encrypted message
          MirrorLinkTransmit(txArray);

          MirrorLinkStayAliveControl();
        }
      }
      break;
    // Receive state
    case MIRRORLINK_RECEIVE:
      if (MirrorLink.status.mirrorLinkStationType == ML_STATION) {
        MirrorLinkStayAliveControl();
      }
      break;
  }
}

// MirrorLink module function, called once every second
void MirrorLinkMain(void) {
  // State changes and transition actions
  MirrorLinkState();

  // Actions within a state
  MirrorLinkWork();
}

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)
