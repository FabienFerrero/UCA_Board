# UCA_Board


Version 1.0.0, November, 2017
Author: Fabien Ferrero

<img src="https://github.com/FabienFerrero/UCA_Board/blob/master/documents/pictures/UCA.jpg">

This PCB was developed to ease connection between an Arduino Mini Pro, and RFM95 LoRa module, AA battery and a printed antenna.

Codes for the microcontroller can be found on this link in the Arduino section : https://github.com/CongducPham/LowCostLoRaGw

# Bill Of Material

* Arduino Mini Pro 3.3V 8MHz
* RFM95

Optional :

* DHT-22 ( temperature and humidity sensor)
* BME280 ( temperature, barometer and humidity sensor)
* RGB LED

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

# Librairies

You will need some librairies to run the code, you can look in the libraries section to find the different libs needed for running the code.

# License

All rights reserved. This Gerber, program and the accompanying materials are made available under the terms of the MIT License which accompanies this distribution, and is available at https://opensource.org/licenses/mit-license.php
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Maintained by Fabien Ferrero

