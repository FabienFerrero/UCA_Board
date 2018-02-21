This code has been modified from : https://github.com/tkerby/minilora-test

To register on the network, you need to specify :

*AppEUI :[8 Bytes] LSB format <BR>
*DevEUI :[8 Bytes] LSB format <BR>
*APPKEY :[8 Bytes] MSB format <BR>


It includes :

*LoRaWan OTAA activation
*Adaptative Spreading factor
*Low-power sleep between Tx
*Vcc MCU measurement
*Cayenne Payload
This code has been tested on EUR868 and AS923 bands

The bands can be selected in config.h of Arduino_LMIC Lib

The starting SF for the joining process can be changed in the LMIC library :
lmic.c, change in  "setDrJoin()"
