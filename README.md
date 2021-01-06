# AriloSprinkler-Firmware
OpenSprinkler firmware variant based on the ESP32-WROOM (using port from J.Charer) and including Lora SX1262 (tested module is E22-900M30S-S1262) Long Range support. A so called "MirrorLink" driver has been developed and included allowing 2 stations to synchronize in a master (remote) / slave (station) mode via the long range SX1262 based transceivers. All communications are encrypted and protected against manipulation.  and supporting the following commands (further ones in development):

- Test command synchronization -> Any manual valve control command in the web interface of the "remote" device will trigger the same command in the "station" device over the MirrorLink Lora protocol
- Program synchronization  -> The creation or modification of programs with all relevant data in the web interface of the "remote" device will trigger the creation/modification of the program in the "station" device over the MirrorLink Lora protocol
- Time synchronization -> The time on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol
- Time zone synchronization -> The time zone on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol
- Station reboot -> Triggering a reboot via switch button in the "remote" device will lead to a reboot in the "station" over the MirrorLink Lora protocol. The remote device will not be rebooted
- Longitude synchronization -> The longitude on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol
- Latitude synchronization -> The latitude on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol
- Sunrise time synchronization -> The sunrise time on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol
- Sunset time synchronization -> The sunset time on the "station" device will be synchronized with the "remote" over the MirrorLink Lora protocol

Further features:
- Plaformio support, as well on MacOs (I am using the libraries for reference in case of compilation issues: ESP8266_SSD1306, PubSubClient, RadioLib, Time, U8g2, UIPEthernet)
- Encrypted lora protocol with Speck64/128 block cipher and CTR cipher mode. Protected against manipulation (the repetition of an intercepted command will not induce any actions neither on the remote nor on the station)
- Frequency Hopping, ATPC (Adaptive Transmission Power Control, based on SNR/RSSI) as well as Duty Cycle options have been added to comply with most country RF regulations 
- MirrorLink control panel (http://device_ip/mlcontrol):
![MirrorLink Control Panel](https://github.com/arijav/AriloSprinkler-Firmware/blob/master/pictures/AriloSprinkler_MirrorLink_Control.jpg)

Tested features:
- Opensprinkler SW features work
- Lora transceiver works
- Current measurement works
- Valve control works
- Button switches work

The compatible HW can be found in the following OpenSprinkler fork:
https://github.com/arijav/AriloSprinkler-Hardware/blob/master/README.md#arilosprinkler-hardware

Final design (not yet produced, use at your own risk):

![Final design front](https://github.com/arijav/AriloSprinkler-Hardware/blob/master/AriloSprinkler/Pictures/AriloSprinklerAC_front.jpg)
![Final design back](https://github.com/arijav/AriloSprinkler-Hardware/blob/master/AriloSprinkler/Pictures/AriloSprinklerAC_back.jpg)

Prototype:

![Prototype front](https://github.com/arijav/AriloSprinkler-Hardware/blob/master/AriloSprinkler/Pictures/AriloSprinklerAC_prototype_front.jpg)
![Prototype back](https://github.com/arijav/AriloSprinkler-Hardware/blob/master/AriloSprinkler/Pictures/AriloSprinklerAC_prototype_back.jpg)

Note: This is a home made prototype. No guarantee of any kind is given. If you want a commercial product please order the regular OpenSprinkler official parts.

For questions send a message to:
gmail email: arijav2020
