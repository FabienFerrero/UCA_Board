This code has been modified from : https://github.com/tkerby/minilora-test

It includes :

* LoRaWan OTAA activation
* Adaptative Spreading factor
* Low-power sleep between Tx 
* TEMT6000 sensor for light
* Cayenne Payload

This code has been tested on EUR868 and AS923 bands

The bands can be selected in config.h of Arduino_LMIC Lib

The light sensor is the TEMT6000, and the module integrating this sensor is provided by Sparkfun : https://www.sparkfun.com/products/8688

This board can be find easily for less than 1€.

As shown is figure below, the photo transistor is mounted with a 10kohm resistor.

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/TEMT6000_sch.png">


From the voltage measured on Sig PIN, the source current can be easily calculated, and the figure below extracted from the datasheet can be used to calculate the illuminance in Lx.
The datasheet can be find in this link : https://www.vishay.com/docs/81579/temt6000.pdf


<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/TEMT6000_lx.jpg">

For Illuminance, the Cayenne LPP code is 101 (Hex is 65), the Data size is 2 Byte with a resoluation of 1 Lux (Unsigned MSB)




