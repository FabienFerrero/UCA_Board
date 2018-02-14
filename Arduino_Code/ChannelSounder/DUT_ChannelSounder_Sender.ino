//#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
//#include<Arduino.h>


#define SS 10
#define RST 8
#define DI0 2
#define BAND 866E6
#define spreadingFactor 7
#define SignalBandwidth 125E3
//#define SignalBandwidth 31.25E3
#define preambleLength 8
#define codingRateDenominator 8
#define Pow 2

//#define SHOW_DEBUGINFO    // Uncomment if you want to see info on Serial

int counter = 0;
long freq_start = 750e6;
long freq_stop = 1000e6;
long freq_step = 5e6;

long next_freq = freq_start;

void setup() {
    
  LoRa.setPins(SS,RST,DI0);
  Serial.begin(115200); 
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(spreadingFactor);
  LoRa.setSignalBandwidth(SignalBandwidth);
  LoRa.setCodingRate4(codingRateDenominator);
  LoRa.setPreambleLength(preambleLength);
  LoRa.setTxPower(Pow, 1);
   
  delay(2000);
  
  #ifdef SHOW_DEBUGINFO 
  Serial.println("LoRa Sender");
  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  Serial.print("LoRa power: ");
  Serial.println(Pow);
  Serial.println("LoRa Initial OK!");
  delay(5000);
  #endif  
  
}

void loop() {

  LoRa.setFrequency(next_freq); // change frequency

  #ifdef SHOW_DEBUGINFO
  Serial.print("Sending packet: ");
  Serial.println((String)next_freq + " MHz");
  #endif
  
  next_freq = next_freq + freq_step ; // increment of the next freq
  if(next_freq > freq_stop){  // when freq arrived at freq stop, it restart at freq_start
  next_freq = freq_start;
      }
  
  // send packet
  LoRa.beginPacket();
    LoRa.print(next_freq);
  LoRa.endPacket();  
  counter++;
  
  digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(100); // wait for a second
  digitalWrite(13, LOW); // turn the LED off by making the voltage LOW
  delay(100); // wait for a second

 
  

}
