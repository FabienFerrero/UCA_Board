Channel sounding is based on Heltec 32 LoRa or TTGO LoRa 32 terminal with the OLED screen.

In order to perform a channel measurements, two devices are needed : a sender and a receiver.

For the sender, UCA board or OLED screen board can be used.
The sender send lora packet at different frequencies with the next frequencies in the packet payload.

For the receiver, use the "LoRaOLED_ChannelSounder_Master" code.

The code is hopping the receiving frequency by decoding the received payload. 
The RSSI is stored in an array and averaged. The min and max value are also displayed

When the button is pushed the counter is reset and the measurement is restarted.
