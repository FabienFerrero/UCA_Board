
/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   
   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

/*******************************************************************************
 This exemples has been modified by Fabien Ferrero to work on UCA board 
 and to send various sensors payload
 ****************************************************************************************
 */



#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"

//Define sensor PIN

#define LAPIN A0 // PIN with Light sensor analog output 
#define LPPIN 6 // PIN with Light power input


#define debugSerial Serial
#define SHOW_DEBUGINFO
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }



//Commented out keys have been zeroed for github

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// global enviromental parameters
//static float temp = 0.0;
//static float pressure = 0.0;
//static float humidity = 0.0;
static float batvalue;
static float light;



// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {2, 7, 9},
};

// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned int TX_INTERVAL = 300;

void setDataRate() {
  switch (LMIC.datarate) {
    case DR_SF12:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF12"));
    #endif      
      TX_INTERVAL = 4800;
      break;
    case DR_SF11: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF11"));
    #endif
      TX_INTERVAL = 2400;
      break;
    case DR_SF10: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF10"));
    #endif
      TX_INTERVAL = 1200;
      break;
    case DR_SF9: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF9"));
    #endif
      TX_INTERVAL = 600;
      break;
    case DR_SF8: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF8"));
    #endif
      TX_INTERVAL = 360;
      break;
    case DR_SF7: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF7"));
    #endif
      TX_INTERVAL = 180;
      break;
    case DR_SF7B: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF7B"));
    #endif
      TX_INTERVAL = 180;
      break;
    case DR_FSK: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: FSK"));
    #endif
      TX_INTERVAL = 180;
      break;
    default: debugPrint(F("Datarate Unknown Value: "));
      debugPrintLn(LMIC.datarate); TX_INTERVAL = 600;
      break;
  }
}

extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

void do_sleep(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;

#ifdef SHOW_DEBUGINFO
  debugPrint(F("Sleeping for "));
  debugPrint(sleepyTime);
  debugPrint(F(" seconds = "));
  debugPrint(eights);
  debugPrint(F(" x 8 + "));
  debugPrint(fours);
  debugPrint(F(" x 4 + "));
  debugPrint(twos);
  debugPrint(F(" x 2 + "));
  debugPrintLn(ones);
  delay(500); //Wait for serial to complete
#endif


  for ( int x = 0; x < eights; x++) {
    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < fours; x++) {
    // put the processor to sleep for 4 seconds
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < twos; x++) {
    // put the processor to sleep for 2 seconds
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < ones; x++) {
    // put the processor to sleep for 1 seconds
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  addMillis(sleepyTime * 1000);
}


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

float readLight() {
  float result;
  // Light sensor Voltage
digitalWrite(LPPIN, HIGH); // Power the sensor
delay(1);
int sensorValue = analogRead(LAPIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  float voltage = sensorValue * (batvalue / 1023.0)/100; // Batvalue is in tens of mV, so the result has to be divided by 100
result = voltage*200; // multiply by 2000 to have Lx
digitalWrite(LPPIN, LOW); // switch off the sensor
  return result;
}

void updateEnvParameters()
{
  batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt for Cayenne Payload
  light = readLight();
    

  #ifdef SHOW_DEBUGINFO
  // print out the value you read:
  Serial.print("Vbatt : ");
  Serial.println(batvalue);
  Serial.print("Light : ");
  Serial.println(light);
  #endif
  
}


void onEvent (ev_t ev) {
  #ifdef SHOW_DEBUGINFO
  Serial.print(os_getTime());
  Serial.print(": ");
  #endif
  switch (ev) {
    case EV_SCAN_TIMEOUT:
    #ifdef SHOW_DEBUGINFO
  debugPrintLn(F("EV_SCAN_TIMEOUT"));
  #endif
     
      break;
    case EV_BEACON_FOUND:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_BEACON_FOUND"));
    #endif      
      break;
    case EV_BEACON_MISSED:
      //debugPrintLn(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //debugPrintLn(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOINING"));
    #endif      
      break;
    case EV_JOINED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOINED"));
    #endif      
      setDataRate();      
      // Ok send our first data in 10 ms
      os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      break;
    case EV_RFU1:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_RFU1"));
    #endif
      
      break;
    case EV_JOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_REJOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_REJOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_TXCOMPLETE:

    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    #endif
      
      if (LMIC.txrxFlags & TXRX_ACK)
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("Received ack"));
      #endif
        
      if (LMIC.dataLen) {
        #ifdef SHOW_DEBUGINFO
        debugPrintLn(F("Received "));
        debugPrintLn(LMIC.dataLen);
        debugPrintLn(F(" bytes of payload"));
        #endif 
       
      }
            // Schedule next transmission
      setDataRate();
      do_sleep(TX_INTERVAL);
      os_setCallback(&sendjob, do_send);
      break;
    case EV_LOST_TSYNC:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LOST_TSYNC"));
      #endif      
      break;
    case EV_RESET:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_RESET"));
      #endif        
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_RXCOMPLETE"));
      #endif      
      break;
    case EV_LINK_DEAD:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LINK_DEAD"));
      #endif       
      break;
    case EV_LINK_ALIVE:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LINK_ALIVE"));
      #endif       
      break;
    default:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("Unknown event"));
      #endif      
      break;
  }
}


void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    debugPrintLn(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    // Here the sensor information should be retrieved
    
    updateEnvParameters();
       

#ifdef SHOW_DEBUGINFO

    debugPrint(F("BV="));
    debugPrintLn(batvalue);
    debugPrint(F("L="));
    debugPrintLn(light);
#endif
    int bat = batvalue; // Cayenne analog output is 0.01 Signed
    int l = light; // light sensor in Lx

    unsigned char mydata[8];
    mydata[0] = 0x3;
    mydata[1] = 0x2;
    mydata[2] = bat >> 8;
    mydata[3] = bat & 0xFF;
    mydata[4] = 0x4;
    mydata[5] = 0x65;
    mydata[6] = l >> 8;
    mydata[7] = l & 0xFF;

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugPrintLn(F("PQ")); //Packet queued
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void lmicStartup() {
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(1);
  LMIC_setAdrMode(1);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // Increase window time for clock accuracy problem
  
  

  // Start job (sending automatically starts OTAA too)
  // Join the network, sending will be
  // started after the event "Joined"
  LMIC_startJoining();
}

void setup() {
  Serial.begin(115200);
  delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used
  
  Serial.begin(115200);
  
  #ifdef SHOW_DEBUGINFO
  debugPrintLn(F("Starting"));
  delay(100);
  #endif
  
  Wire.begin();
  pinMode(LPPIN, OUTPUT);

  updateEnvParameters(); // To have value for the first Tx
  

  // LMIC init
  os_init();
  lmicStartup();

}

void loop() {
  os_runloop_once();
}

