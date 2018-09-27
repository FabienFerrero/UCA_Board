/* 
 *  
 *  Copyright (c) 2018 Fabien Ferrero
 *
 *  Permission is hereby granted, free of charge, to anyone
 *  obtaining a copy of this document and accompanying files,
 *  to do whatever they want with them without any restriction,
 *  including, but not limited to, copying, modification and redistribution.
 *  NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 *  This example receive LoRa packet with frequency hooping information in the
 *  payload and change the frequency for the next packet.
 *  This code made for Heltec LoRa board 860-915MHz version
 *  It can be use to measure LoRa device  radiation performance and to tune the 
 *  device antenna
 *  
 */


#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include <SPI.h>
#include <LoRa.h>
// #include "SSD1306.h"

SSD1306 display(0x3c, 4, 15); // Set the display


#define SS 18
#define RST 14
#define DI0 26

// LoRa Settings 
#define BAND 866E6
#define spreadingFactor 7 // Use of a low SF to speed the scanning process
#define SignalBandwidth 125E3
//#define SignalBandwidth 31.25E3

#define codingRateDenominator 8

#define interruptPin 0  // PIN 0 for interrupt with PRG button

long freq = 868e6; // Default frequency at initialisation
long freq_max=870e6;
long freq_min=863e6;

long max_freq=freq; // freq with max RSSI
long min_freq=freq; // freq with max RSSI

bool Pushdetected = false; // reset peak freq

long counter = -1; // number of received packet since the last init
int RSSI = -140;
int max_RSSI=-140;
int min_RSSI=0;
int average_RSSI=0;
int RSSI_array[10000];



void gotPush(){   // reset peak freq on interrupt
 Pushdetected = true;
}
  
void setup() {

  // Define interrupt on PRG button to reset peak freq
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), gotPush, CHANGE);
    
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  Serial.begin(115200);
  while (!Serial); //if just the the basic function, must connect to a computer
  delay(1000);
  
  Serial.println("LoRa Receiver");
  display.drawString(5,5,"LoRa Receiver");
  display.display();
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);



  if (!LoRa.begin(BAND)) {
    display.drawString(5,25,"Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Init OK");
  
  Serial.print("LoRa Frequency: ");
  Serial.println(BAND);
  
  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  LoRa.setSpreadingFactor(spreadingFactor);
  
  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  LoRa.setSignalBandwidth(SignalBandwidth);

  LoRa.setCodingRate4(codingRateDenominator);
  
  display.drawString(5,25,"LoRa Init OK");
  display.drawString(5,45,"Wait @ "+(String)((int)(freq/1e6))+ "MHz");
  display.display();
}

void loop() {
  
  if(Pushdetected){ // reset Peak freq and max RSSI value
    Pushdetected = false;
    max_RSSI=-140;
    min_RSSI=0;
    counter = -5; // start with count < 0 to add a small delay before the start of the measurement
    max_freq=0;
    min_freq=0;
  
       
  }
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packets
    
    
        
    counter++;
    
    // read packet
    while (LoRa.available()) {
      String data = LoRa.readString();
      freq = data.toInt(); // data converted to int
     // If a packet is not received with a suitable frequency, set in the good freq range
     if(freq > freq_max){
     freq = freq_max;
     }
     if(freq < freq_min){
     freq = freq_min;
     }
     
    //RSSI calculation
      RSSI = LoRa.packetRssi()+157; // Extract raw value of packetrssi register
      RSSI = (16*RSSI)/15; //value correction from the datasheet
      RSSI = RSSI -157; // Re-Calculate value in dBm

      if(counter >= 0){
      RSSI_array[counter]=RSSI;
      }
      
     
      
        
      long freq_MHz= freq / 1e6; // freq in MHz 

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Rx N°" + (String)counter+" "+ (String)freq_MHz + "M: " + (String)RSSI + "dBm");
    display.display();

     if(counter > 0){

    average_RSSI = 0;
    for( int i=0; i<=counter; i++ ) {
    average_RSSI = average_RSSI + RSSI_array[i];
      }
    average_RSSI = average_RSSI / counter;

    display.drawString(2, 15,"Av:"+(String)average_RSSI + "dBm" );
    display.display();

     if(RSSI > max_RSSI && counter > 0){
      max_RSSI=RSSI;
      max_freq=freq;
      }

  if(RSSI < min_RSSI && counter > 0){
      min_RSSI=RSSI;
      min_freq=freq;
      }    
    // print RSSI of packet
           
      Serial.print(freq);
      long minfreq_MHz= min_freq / 1e6; // freq in MHz
      display.drawString(0,33,"Min:"+(String)minfreq_MHz+"MHz:"+(String)min_RSSI + "dBm");
      
      display.display();
      }
    }
   
    Serial.print("  ");
    Serial.print(RSSI);
    Serial.print("  ");
    Serial.println(LoRa.packetSnr());
    long maxfreq_MHz= max_freq / 1e6; // freq in MHz
    display.drawString(0, 52, "Max:"+(String)maxfreq_MHz+"MHz:"+(String)max_RSSI + "dBm");
    display.display();

    LoRa.setFrequency(freq); // change frequency
  }
}
