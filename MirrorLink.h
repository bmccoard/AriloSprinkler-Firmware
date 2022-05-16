/*
  MirrorLink.h - LORA long range MirrorLink driver for OpenSprinkler

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

#ifndef _MIRRORLINK
#define _MIRRORLINK

#include "utils.h"

#if defined(ESP32) && defined(MIRRORLINK_ENABLE)

// Special config defines
//#define DISABLE_ESP32_BROWNOUT
#define ENABLE_DEBUG_MIRRORLINK
//#define ML_LOCALTEST
#define MIRRORLINK_ENABLE_BLACKLIST                // If defined blacklist is enabled
#define RADIOLIB_GODMODE

// Config defines
#define MIRRORLINK_BUFFERLENGTH             100    // Maximum command buffer length
#define MIRRORLINK_LORA_MESSAGE_BYTE_LENGTH 8      // Length in bytes from the Lora messages
#if defined(ML_LOCALTEST)
#define MIRRORLINK_REGCOMMANDS_FAST_PERIOD  80     // Period in seconds to send the regular commands to remote station (fast)
#else
#define MIRRORLINK_REGCOMMANDS_FAST_PERIOD  300    // Period in seconds to send the regular commands to remote station (fast)
#endif
#define MIRRORLINK_REGCOMMANDS_MID_PERIOD   86400  // Period in seconds to send the regular commands to remote station (mid)
#define MIRRORLINK_REGCOMMANDS_SLOW_PERIOD  604800 // Period in seconds to send the regular commands to remote station (slow)
#define MIRRORLINK_RXTX_MAX_TIME            5      // Maximum time in seconds to wait for response from station or command / response transmission
#if defined(ML_LOCALTEST)
#define MIRRORLINK_STAYALIVE_PERIOD         90     // Maximum time in seconds w/o message reception from counterpart station to consider the link dead
#else
#define MIRRORLINK_STAYALIVE_PERIOD         900    // Maximum time in seconds w/o message reception from counterpart station to consider the link dead
#endif
#define MIRRORLINK_RXTX_DEAD_TIME           2      // Time in seconds after receiving a message, to start transmitting one
#if defined(ML_LOCALTEST)
#define MIRRORLINK_KEYCHANGE_MAX_TIME       3600//120    // Maximum period in seconds to renew the keys
#define MIRRORLINK_KEYCHANGE_MIN_TIME       900//60     // Minimum period in seconds to renew the keys
#define MIRRORLINK_MAX_POWER                -17    // MirrorLink maximum transmission power in dBm
#else
#define MIRRORLINK_KEYCHANGE_MAX_TIME       3600   // Maximum period in seconds to renew the keys
#define MIRRORLINK_KEYCHANGE_MIN_TIME       900    // Minimum period in seconds to renew the keys
#define MIRRORLINK_MAX_POWER                16     // MirrorLink maximum transmission power in dBm
#endif
#define MIRRORLINK_MIN_POWER               -17     // MirrorLink minimum transmission power in dBm
#define MIRRORLINK_AMPLIF_FACTOR            14     // MirrorLink amplification factor in dB (amplifier plus antenna, for plain E22-900T30S with 0dB antenna is 14dBm -> for 16dBm tx, amplification up to 30dBm)
#define MIRRORLINK_MIN_POWER_BUDGET        -90     // MirrorLink minimum link budget in dBm for proper reception
#define MIRRORLINK_NETWORK_ID               0x94   // Default Network ID
#define MIRRORLINK_CHANNEL_BLACKLIST_TIME   604800 // Time a channel surpasing the maximum number of fail transmissions (in the same period) shall be banned
#define MIRRORLINK_CHANNEL_BLACKLIST_NUM    15     // Number of lost packets within MIRRORLINK_CHANNEL_BLACKLIST_TIME period to set the channel as banned

// Payload positions and masks
#define CMD_STATION_POS                     24
#define CMD_REMOTE_POS                      8
#define NETWORKID_POS                       24
#define STATE_POS                           22
#define POWERCMD_POS                        18
#define CHNUMBER_POS                        14
#define SNR_STATION_POS                     6
#define RSSI_STATION_P1_POS                 2
#define RSSI_STATION_P2_POS                 30
#define BOARDSEL_REMOTE_POS                 4
#define BOARDSTATUS_STATION_POS             16
#define APPERROR_STATION_POS                8
#define SNR_STATION_MASK                    0x1FC0
#define SNR_SIGN_STATION_MASK               0x2000
#define RSSI_SIGN_STATION_P1_MASK           0x20
#define RSSI_STATION_P1_MASK                0x1F
#define RSSI_STATION_P2_MASK                0xC0000000
#define PAYLOAD1_REMOTE_MASK                0xFF
#define PAYLOAD2_STATION_MASK               0xFFFFFF
#define HEADER_MASK                         0xFFFFC000
#define CMD_REMOTE_MASK                     0x3F00
#define STATE_MASK                          0xC00000
#define POWERCMD_MASK                       0x3C0000
#define CHNUMBER_MASK                       0x3C000
#define BOARDSEL_MASK                       0xF
#define BOARDSTATUS_MASK                    0xFF

// Speck64/128 encryption based on NSA implementation guide plus CTR cipher mode
#define MIRRORLINK_SPECK_ROUNDS             27
#define MIRRORLINK_SPECK_KEY_LEN            4  // 4x uint32_t
#define MIRRORLINK_SPECK_TEXT_LEN           2  // 2x uint32_t
#define ROR(x, r)                           ((x >> r) | (x << (32 - r)))
#define ROL(x, r)                           ((x << r) | (x >> (32 - r)))
#define ER(x, y, k)                         (x = ROR(x, 8), x += y, x ^= k, y = ROL(y, 3), y ^= x)
#define DR(x, y, k)                         (y ^= x, y = ROR(y, 3), x ^= k, x -= y, x = ROL(x, 8))
#define MIRRORLINK_SPECK_DEFAULT_KEY_N1     0x03020100 // 50462976
#define MIRRORLINK_SPECK_DEFAULT_KEY_N2     0x0b0a0908 // 185207048
#define MIRRORLINK_SPECK_DEFAULT_KEY_N3     0x13121110 // 319951120
#define MIRRORLINK_SPECK_DEFAULT_KEY_N4     0x1b1a1918 // 454695192

#if defined(ENABLE_DEBUG_MIRRORLINK) /** Serial debug functions */

  #if defined(ARDUINO)
    #define MLDEBUG_BEGIN(x)   {Serial.begin(x);}
    #define MLDEBUG_PRINT(x)   {Serial.print(x);}
    #define MLDEBUG_PRINTLN(x) {Serial.println(x);}
    #define MLDEBUG_PRINTX(x)  {Serial.print(F("0x")); Serial.print(x, HEX);}
    #define MLDEBUG_PRINBIN(x) {Serial.print(x, BIN);}
  #endif

#else

  #define MLDEBUG_BEGIN(x)   {}
  #define MLDEBUG_PRINT(x)   {}
  #define MLDEBUG_PRINTLN(x) {}
  #define MLDEBUG_PRINTX(x) {}

#endif

// Enum for station types
enum MirrorlinkStationType {
  ML_STATION = 0,     // Type is station
  ML_REMOTE,          // Type is remote
};

// Enum for commands
enum MirrorlinkCommands {
  ML_NO_CMD = 0,              // No command
  ML_TESTSTATION,             // Switch on station for a specific duration in minutes
  ML_PROGRAMADDDEL,           // Create/delete a program
  ML_PROGRAMMAINSETUP,        // Configure program enable, type, use weather, odd/eve
  ML_PROGRAMDAYS,             // Configure duration program days
  ML_PROGRAMSTARTTIME,        // Configure start time of a specific program (max. 4 per program)
  ML_PROGRAMDURATION,         // Configure duration of a specific station for a specific program (max. 8)
  ML_TIMESYNC,                // Command to configure the time in the remote station
  ML_TIMEZONESYNC,            // Command to configure the time zone in the remote station
  ML_CURRENTREQUEST,          // Command to request the station to send its current to the remote
  ML_EMERGENCYSHUTDOWN,       // Command to shutoff all outputs in the remote station
	ML_STATIONREBOOT,           // Perform a reboot
  ML_LATITUDE,                // Command to configure the latitude in the remote station
  ML_LONGITUDE,               // Command to configure the longitude in the remote station
  ML_SUNRISE,                 // Command to configure the sunrise time in the remote station
  ML_SUNSET,                  // Command to configure the sunset time in the remote station
  ML_RAINDELAYSTOPTIME,       // Command to configure the rain delay stop time
  ML_STAYALIVE,               // Command sent regularily, needed for the remote station to not shutdown all outputs
  ML_MAX_CMD                  // Total number of commands
};

// Enum for command parts on station side
enum MirrorlinkStationCommandParts {
  ML_CMD_1 = 0,      // First part of the command (14 bits)
  ML_CMD_2,          // Second part of the command (32 bits)
  ML_CMD_MAX         // Max payload commands
};

// Enum for channel numbers
enum MirrorlinkChannels {
  ML_CH_0 = 0,     // Channel 0
  ML_CH_1,         // Channel 1
  ML_CH_2,         // Channel 2
  ML_CH_3,         // Channel 3
  ML_CH_4,         // Channel 4
  ML_CH_5,         // Channel 5
  ML_CH_MAX        // Max. channel number
};

// Enum for transmit/receive status
enum MirrorlinkTxRx {
  ML_RECEIVING = 0,// Module is transmitting
  ML_TRANSMITTING  // Module is receiving
};

// Enum for link active/inactive
enum MirrorlinkStatus {
  ML_LINK_DOWN = 0,// Link is down
  ML_LINK_UP       // Link is up
};

// Enum for errors (max. 255 error types)
enum MirrorlinkErrors {
  ML_NO_ERROR = 0,  // No error
  ML_SYNCERROR      // Sync. error between remote and station
};

// Enum for link communication phase command from station
enum MirrorlinkComPhase {
  ML_LINK_COM_ASSOCIATION = 0,  // Request to start association
  ML_LINK_COM_NONCEUPDATE,      // Request to start Nonce update
  ML_LINK_COM_NORMAL            // Request to start normal communication
};

// Enums for states of the MirrorLink driver
enum MirrorlinkModes { 
  MIRRORLINK_INIT = 0,         // Init state
  MIRRORLINK_ASSOCIATE,        // Associate state
  MIRRORLINK_NONCEUPDATE,      // Nonce update state
  MIRRORLINK_BUFFERING,        // Buffering state
  MIRRORLINK_SEND,             // Send state
  MIRRORLINK_RECEIVE           // Receive state
};

void MirrorLinkBuffCmd(uint8_t cmd, uint32_t payload);
void MirrorLinkPeriodicCommands();
uint32_t MirrorLinkGetCmd(uint8_t cmd);
uint8_t MirrorLinkGetStationType(void);
bool MirrorLinkSetNetworkId(uint8_t networkid);
bool MirrorLinkSetStationType(uint8_t type);
bool MirrorLinkSetKeys(uint32_t ask1, uint32_t ask2, uint32_t ask3, uint32_t ask4);
bool MirrorLinkSetChannel(uint8_t channel);
bool MirrorLinkSetFrequencyHoppingStatus(uint8_t freqhopstatus);
bool MirrorLinkSetMaxPower(int8_t maxpower);
bool MirrorLinkSetATPCStatus(uint8_t atpcstatus);
bool MirrorLinkSetDutyCycle(float dutycycle);
bool MirrorLinkSetBoardSelect(uint8_t boardNumber);
bool MirrorLinkGetAssociationStatus();
uint8_t MirrorLinkGetChannel();
int8_t MirrorLinkGetPower();
void MirrorLinkInit();
void MirrorLinkMain();
String MirrorLinkStatusGeneral();
String MirrorLinkStatusRadio();
String MirrorLinkStatusPackets();
String MirrorLinkStatusBoards();

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)

#endif	// _MIRRORLINK
