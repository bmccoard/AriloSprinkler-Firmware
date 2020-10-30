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
//#define MIRRORLINK_DEBUGRF            // If defined, RF send/receive test between remote and station is performed
#define MIRRORLINK_DUTYCYCLE       100  // Maximum duty cycle allowed in tenths of percentage, default 100 = 10%
#if defined(MIRRORLINK_OSREMOTE)
#define MIRRORLINK_BUFFERLENGTH     10  // Maximum command buffer length
#endif // defined(MIRRORLINK_OSREMOTE)
#define MIRRORLINK_RXTX_MAX_TIME    60  // Maximum time in seconds to wait for response from station or command / response transmission
#define MIRRORLINK_RXTX_DEAD_TIME   5   // Time in seconds after receiving a message, to start transmitting one

// Enum for commands
enum {
  ML_TESTSTATION = 0,// read-only (ro)
	ML_MAX_CMD         // total number of integer options
};

// Enum for transmit/receive status
enum {
  ML_RECEIVING = 0,// Module is transmitting
	ML_TRANSMITTING  // Module is receiving
};

#if defined(MIRRORLINK_OSREMOTE)
void MirrorLinkBuffCmd(uint8_t cmd, uint16_t payload);
#else
uint16_t MirrorLinkGetCmd(uint8_t cmd);
#endif //defined(MIRRORLINK_OSREMOTE)

void MirrorLinkInit();
void MirrorLinkMain();
#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)

#endif	// _MIRRORLINK
