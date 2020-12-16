/*
  MirrorLink.h - LORA long range Mirror Link driver for OpenSprinkler

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

// Config defines
#if defined(MIRRORLINK_OSREMOTE)
#define MIRRORLINK_BUFFERLENGTH             30     // Maximum command buffer length
#define MIRRORLINK_REGCOMMANDS_FAST_PERIOD  30//3600   // Period in seconds to send the regular commands to remote station (fast)
#define MIRRORLINK_REGCOMMANDS_MID_PERIOD   60//86400  // Period in seconds to send the regular commands to remote station (mid)
#define MIRRORLINK_REGCOMMANDS_SLOW_PERIOD  90//604800 // Period in seconds to send the regular commands to remote station (slow)
#define MIRRORLINK_MAX_DUTY_CYCLE           1000//10     // Maximum duty cycle in tenths of % (1 = 0.1)
#endif // defined(MIRRORLINK_OSREMOTE)
#define MIRRORLINK_RXTX_MAX_TIME            5      // Maximum time in seconds to wait for response from station or command / response transmission
#define MIRRORLINK_STAYALIVE_PERIOD         35     // Maximum time in seconds w/o message reception from counterpart station to consider the link dead
#define MIRRORLINK_RXTX_DEAD_TIME           2      // Time in seconds after receiving a message, to start transmitting one
#define MIRRORLINK_MODRADIOLIB                     // If defined Radiohead protected writeRegister function needs to be accesible (move away from protected in class)
#define ML_FREQUENCY                        866.2  // MirrorLink Frequency
#define ML_TX_POWER                         2      // TX Power without counting the amplification (max. is 16dBm for 30dBm output)
#if defined(MIRRORLINK_OSREMOTE)
#define MIRRORLINK_NETWORK_ID               0x94 // Default Network ID
#else
#define MIRRORLINK_NETWORK_ID               0x94 // Default Network ID
#endif

// Speck64/128 encryption based on Moritz Bitsch implementation
#define SPECK_TYPE                          uint32_t
#define SPECK_ROUNDS                        27
#define SPECK_KEY_LEN                       4
#define ROR(x, r)                           ((x >> r) | (x << ((sizeof(SPECK_TYPE) * 8) - r)))
#define ROL(x, r)                           ((x << r) | (x >> ((sizeof(SPECK_TYPE) * 8) - r)))
#define R(x, y, k)                          (x = ROR(x, 8), x += y, x ^= k, y = ROL(y, 3), y ^= x)
#define RR(x, y, k)                         (y ^= x, y = ROR(y, 3), x ^= k, x -= y, x = ROL(x, 8))
#define SPECK_DEFAULT_KEY_N1                0x03020100    
#define SPECK_DEFAULT_KEY_N2                0x0b0a0908
#define SPECK_DEFAULT_KEY_N3                0x13121110
#define SPECK_DEFAULT_KEY_N4                0x1b1a1918

// Enum for station types
enum {
  ML_STATION = 0,     // Type is station
  ML_REMOTE,          // Type is remote
};

// Enum for commands
enum {
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
  ML_APC,                     // Command for Adaptative Power Control (regulate remote station transmission power)
  ML_CHANNEL,                 // Command for channel selection
  ML_MAX_CMD                  // total number of commands
};

// Enum for channel numbers
enum {
  ML_CH_0 = 0,     // Channel 0
	ML_CH_1,         // Channel 1
	ML_CH_2,         // Channel 2
	ML_CH_3,         // Channel 3
	ML_CH_4,         // Channel 4
	ML_CH_5,         // Channel 5
	ML_CH_6,         // Channel 6
	ML_CH_7,         // Channel 7
	ML_CH_8,         // Channel 8
	ML_CH_9,         // Channel 9
	ML_CH_10,        // Channel 10
	ML_CH_11,        // Channel 11
	ML_CH_12,        // Channel 12
	ML_CH_13,        // Channel 13
	ML_CH_14,        // Channel 14
	ML_CH_15,        // Channel 15
};

// Enum for transmit/receive status
enum {
  ML_RECEIVING = 0,// Module is transmitting
	ML_TRANSMITTING  // Module is receiving
};

// Enum for link active/inactive
enum {
  ML_LINK_DOWN = 0,// Link is down
	ML_LINK_UP       // Link is up
};

// Enum for errors
enum {
  ML_NO_ERROR = 0,  // No error
  ML_SYNCERROR      // Sync. error between remote and station
};

// Enum for link communication phase
enum {
  ML_LINK_COM_ASSOCIATION = 0, // Association process ongoing
	ML_LINK_COM_CHANGEKEY,       // Change of key process ongoing
  ML_LINK_COM_NORMAL,          // Normal communication ongoing
};

// Enums for states of the MirrorLink driver
#if defined(MIRRORLINK_OSREMOTE)
enum MirrorlinkModes { MIRRORLINK_INIT, MIRRORLINK_ASSOCIATE, MIRRORLINK_BUFFERING, MIRRORLINK_SEND, MIRRORLINK_RECEIVE };
#else
enum MirrorlinkModes { MIRRORLINK_INIT, MIRRORLINK_ASSOCIATE, MIRRORLINK_SEND, MIRRORLINK_RECEIVE };
#endif

#if defined(MIRRORLINK_OSREMOTE)
void MirrorLinkBuffCmd(uint8_t cmd, uint32_t payload);
void MirrorLinkPeriodicCommands();
#else
uint32_t MirrorLinkGetCmd(uint8_t cmd);
#endif //defined(MIRRORLINK_OSREMOTE)

void MirrorLinkInit();
void MirrorLinkMain();
String MirrorLinkStatus();

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)

#endif	// _MIRRORLINK
