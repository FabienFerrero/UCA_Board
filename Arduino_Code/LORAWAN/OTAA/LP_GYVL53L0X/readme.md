# What is inside : 

* LoRaWan OTAA activation
* Adaptative Spreading factor
* Low-power sleep between Tx 
* GYVL53L0X sensor
* Cayenne LPP Payload

# VL53L0X Time of flight Laser ranging sensor

This I2C sensor is a laser-ranging sensor, with capability to detect a target object up to 2 m away. 
The VL53L0X uses time-of-flight measurements of infrared pulses for ranging, allowing it to give accurate results independent of the target’s color and surface. 
Distance measurements can be read through a digital I²C interface. 

This component can be found for less than 5$.

It consumes 5uA in standby mode and 20mA in active ranging mode. A ranging measurement require from 30 to 200ms.

Nice information on this link : https://www.pololu.com/product/2490

Datasheet : https://www.st.com/resource/en/datasheet/vl53l0x.pdf

This code has been tested on EUR868 and AS923 bands

The bands can be selected in config.h of Arduino_LMIC Lib

You can use https://mydevices.com/ to plot your data,


