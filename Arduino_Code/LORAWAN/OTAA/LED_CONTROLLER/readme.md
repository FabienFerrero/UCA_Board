# What the code is doing ?
* LoRaWan OTAA activation
* Adaptative Spreading factor
* Control a RGB LED
* Cayenne LPP Payload

The luminosity of each color can be adjusted from 0 to 255 using Cayenne my devices

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/cayenne_LED.png">

We are using Class C protocol, so the latency mainly depends on the uplink duty cycle.

For now, only one command can be sent at once, if two commands are sent in the same uplink cycle, only the last one is really sent.

This code has been tested on EUR868 and AS923 bands

The bands can be selected in config.h of Arduino_LMIC Lib

You can use https://mydevices.com/ to control your LED

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_Blue.png">

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_Green.png">

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_Red.png">

