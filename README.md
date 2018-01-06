# UCA_Board


Version 1.0.0, November, 2017
Author: Fabien Ferrero

This PCB was developed to ease connection between an Arduino Mini Pro, and RFM95 LoRa module, AA battery and a printed antenna.

Codes for the microcontroller can be found on this link in the Arduino section : https://github.com/CongducPham/LowCostLoRaGw

*Bill Of Material
TBD

*Wiring

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


All rights reserved. This Gerber, program and the accompanying materials are made available under the terms of the MIT License which accompanies this distribution, and is available at https://opensource.org/licenses/mit-license.php
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Maintained by Fabien Ferrero

Changes for ESP32 by Jac Kersing, don't blame Maarten... See the note at the end for what (should) work.

Gerber Layout of the UCA board
