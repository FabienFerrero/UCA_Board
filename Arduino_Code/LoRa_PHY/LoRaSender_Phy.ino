#include <SPI.h>
#include <LoRa.h>

int counter = 0;

// Parameters you can play with :

int txPower = 14; // from 0 to 20
int spreadingFactor = 12; // from 7 to 12
long signalBandwidth = 125E3; // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,41.7E3,62.5E3,125E3,250E3,500e3
int codingRateDenominator=5; // Numerator is 4, and denominator from 5 to 8
int preambleLength=8; // from 2 to 20
String payload = "hello"; // you can change the payload

#define SS 10
#define RST 8
#define DI0 2
#define BAND 865E6

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  SPI.begin();
  LoRa.setPins(SS,RST,DI0);

  

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 LoRa.setTxPower(txPower,1);
 LoRa.setSpreadingFactor(spreadingFactor);
 LoRa.setSignalBandwidth(signalBandwidth);
 LoRa.setCodingRate4(codingRateDenominator);
 LoRa.setPreambleLength(preambleLength);
// LoRa.setPolarity(1);

 //LoRa.setFSK();
  
 
 
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.print(counter);
  
  LoRa.endPacket();
  counter++;
  
 

  delay(100);
}
