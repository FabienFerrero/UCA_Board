#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

#include <SPI.h>
#include <LoRa.h>
// #include "SSD1306.h"
#include<Arduino.h>


//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA — GPIO4
//OLED_SCL — GPIO15
//OLED_RST — GPIO16

SSD1306 display(0x3c, 4, 15);

 

// WIFI_LoRa_32 ports

// GPIO5 — SX1278’s SCK
// GPIO19 — SX1278’s MISO
// GPIO27 — SX1278’s MOSI
// GPIO18 — SX1278’s CS
// GPIO14 — SX1278’s RESET
// GPIO26 — SX1278’s IRQ(Interrupt Request)

#define SS 18
#define RST 14
#define DI0 26
#define BAND 866E6

// #define BAND 434500000.00
//#define BAND 866500000.00

#define spreadingFactor 7
#define SignalBandwidth 125E3
//#define SignalBandwidth 31.25E3
#define preambleLength 8
#define codingRateDenominator 8
#define Pow 14

int counter = 0;


long freq_start = 863e6;
long freq_stop = 870e6;
long freq_step = 1e6;

long next_freq = freq_start;

void setup() {
  pinMode(25,OUTPUT); //Send success, LED will bright 1 second
  
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
  
  Serial.begin(115200);
  while (!Serial); //If just the the basic function, must connect to a computer

// Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();
  
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  LoRa.setSpreadingFactor(spreadingFactor);
  
  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  LoRa.setSignalBandwidth(SignalBandwidth);

  Serial.print("LoRa power: ");
  Serial.println(Pow);
  LoRa.setTxPower(Pow, 14);

  LoRa.setCodingRate4(codingRateDenominator);

  LoRa.setPreambleLength(preambleLength);
  
  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);
}

void loop() {

  LoRa.setFrequency(next_freq); // change frequency

  
  Serial.print("Sending packet: ");
  Serial.println(counter);
  
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(3, 5, "Sending packet ");
  display.drawString(50, 30, String(next_freq/1e6) + " MHz");
  display.display();

   next_freq = next_freq + freq_step ; // increment of the next freq

  if(next_freq > freq_stop){  // when freq arrived at freq stop, it restart at freq_start

  next_freq = freq_start;
      }
  
  // send packet
  LoRa.beginPacket();
  //LoRa.print("Hello..");
  LoRa.print(next_freq);
  LoRa.endPacket();
  
  counter++;
  digitalWrite(25, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(100); // wait for a second
  digitalWrite(25, LOW); // turn the LED off by making the voltage LOW
  delay(200); // wait for a second

 
  
// delay(3000);
}
