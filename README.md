# UCA_Board


Version 1.0.0, November, 2017
Author: Fabien Ferrero

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA.jpg">

This PCB was developed to ease connection between an Arduino Mini Pro, and RFM95 LoRa module, AA battery and a printed antenna.

Codes for the microcontroller for LoRa communications can be found on this link in the Arduino section : https://github.com/CongducPham/LowCostLoRaGw

I'm also including in this github some codes compatible with LoRaWan in the /Arduino_Code section.

# PCB UCA

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_top.png">

This PCB has been designed to enable the connection of multiple sensors, few possibilities are listed in this figure :

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_sensors.png">

# Bill Of Material


* Arduino Mini Pro 3.3V 8MHz 
https://fr.aliexpress.com/item/10pcs-ATMEGA328P-Pro-Mini-328-Mini-ATMEGA328-3-3V-16-MHz-for-Arduino-3-3V-16/32784455996.html?spm=a2g0s.13010208.99999999.276.LOMiw9
* RFM95W
https://fr.aliexpress.com/item/RFM95W-RFM95-868MHz-LORA-SX1276-wireless-transceiver-module-20DBM-3KM-Best-quality/32810607598.html?spm=a2g0s.9042311.0.0.bMWhGH

Optional :

* DHT-22 ( temperature and humidity sensor)
https://fr.aliexpress.com/item/Free-Shipping-1pcs-DHT22-digital-temperature-and-humidity-sensor-Temperature-and-humidity-module-AM2302-replace-SHT11/32730343520.html?spm=a2g0s.9042311.0.0.RsBFuh
* BME280 ( temperature, barometer and humidity sensor)
https://fr.aliexpress.com/item/3In1-BME280-GY-BME280-Digital-Sensor-SPI-I2C-Humidity-Temperature-and-Barometric-Pressure-Sensor-Module-1/32830483099.html?spm=a2g0s.9042311.0.0.2GH3nf
* RGB LED
https://fr.aliexpress.com/item/100pcs-Ultra-Bright-Red-Green-Blue-SMT-SMD-LED-RGB-5050-LED-DIODE-Chip-Light-Emitting/32340120687.html?spm=a2g0s.9042311.0.0.RsBFuh
* Ultrasound distance sensor
https://fr.aliexpress.com/item/Free-shipping-10Pcs-lot-Ultrasonic-Module-HC-SR04-Distance-Measuring-Transducer-Sensor-for-ar-du-ino/32745224945.html?spm=a2g0s.9042311.0.0.RsBFuh
* Step-up voltage booster
https://fr.aliexpress.com/item/5pcs-2-in-1-DC-DC-Step-Down-Step-Up-Converter-0-8V-6V-to-3/32759171595.html?spm=a2g0s.9042311.0.0.bMWhGH


<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA_mounted.png">

# Wiring

```
 Arduino Mini       LoRa RFM95W 
 Mini Pro           Module
 D8          <----> RST
 MISO  (D12) <----> MISO
 MOSI  (D11) <----> MOSI
 SCK   (D13) <----> CLK
 SS    (D10) <----> SEL (Chip Select)
 D2          <----> DIO0
 D7          <----> DIO1
 D9          <----> DIO2
 3.3V        <----> Vcc

 ```

# Layout

The layout of the PCB is available the gerber section.
It has been designed to be fabricated on a 0.8mm-thick FR4 2 layer PCB.
Depending of your need, it can be fabricated with different thickness, but the antenna has to be adjusted.

The board can be directly ordered on this link : <BR><BR>
<a href="https://www.pcbway.com/project/shareproject/UCA_Board.html"><img src="https://www.pcbway.com/project/img/images/frompcbway.png" alt="PCB from PCBWay"></img></a>

# Librairies

You will need some librairies to run the code, you can look in the libraries section to find the different libs needed for running the code.

# License

All rights reserved. This Gerber, program and the accompanying materials are made available under the terms of the MIT License which accompanies this distribution, and is available at https://opensource.org/licenses/mit-license.php
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Maintained by Fabien Ferrero

