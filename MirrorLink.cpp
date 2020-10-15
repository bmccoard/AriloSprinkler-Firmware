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

#include "defines.h"
#include "MirrorLink.h"
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

// SX1262 has the following connections:
// NSS pin:   18
// DIO1 pin:  33
// DIO2 pin:  32
// BUSY pin:  26
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_DIO2, LORA_BUSY);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// save transmission state between loops
int transmissionState = ERR_NONE;

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
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlagRx(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlagTx(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
}

// MirrorLink module initialization
void MirrorLinkInit(void) {
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
	int state = lora.begin(868.0, 125.0, 7, 5, 0x1424, 0, 8);
	  if (state == ERR_NONE) {
    Serial.println(F("success!"));
	} else {
		Serial.print(F("failed, code "));
		Serial.println(state);
	}
	// eByte E22-900M uses DIO3 to supply the external TCXO
	if (lora.setTCXO(2.4) == ERR_INVALID_TCXO_VOLTAGE)
	{
		Serial.println(F("Selected TCXO voltage is invalid for this module!"));
	}

#if defined(MIRRORLINK_OSREMOTE)
	// set the function that will be called
	// when new packet is transmitted
	lora.setDio1Action(setFlagTx);

	// Important! To enable transmit you need to switch the SX126x antenna switch to TRANSMIT
	enableTX();

	// start transmitting the first packet
	Serial.print(F("[SX1262] Sending first packet ... "));

	// you can transmit C-string or Arduino string up to
	// 256 characters long
	transmissionState = lora.startTransmit("Hello World!");

	// you can also transmit byte array up to 256 bytes long
	/*
		byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
						0x89, 0xAB, 0xCD, 0xEF};
		state = lora.startTransmit(byteArr, 8);
	*/
#else
	// set the function that will be called
	// when new packet is received
	lora.setDio1Action(setFlagRx);

	// Important! To enable receive you need to switch the SX126x antenna switch to RECEIVE 
	enableRX();

	// start listening for LoRa packets
	Serial.print(F("[SX1262] Starting to listen ... "));
	state = lora.startReceive();
	if (state == ERR_NONE) {
		Serial.println(F("success!"));
	} else {
		Serial.print(F("failed, code "));
		Serial.println(state);
		while (true);
	}
#endif
}

// MirrorLink module main function
void MirrorLinkMain(void) {
// If MirrorLink LORA station is a remote controller
#if defined(MIRRORLINK_OSREMOTE)
  // check if the previous transmission finished
  if(transmittedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    transmittedFlag = false;

    if (transmissionState == ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // wait a second before transmitting again
    //delay(1000);

    // send another one
    Serial.print(F("[SX1262] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    transmissionState = lora.startTransmit("Hello World!");

    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      int state = lora.startTransmit(byteArr, 8);
    */

    // we're ready to send more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }
#else
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = lora.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = lora.readData(byteArr, 8);
    */

    if (state == ERR_NONE) {
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

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);

    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }
#endif

}

#endif // defined(ESP32) && defined(MIRRORLINK_ENABLE)