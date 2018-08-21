
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

//Sensors librairies

#include <BME280I2C.h>

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



// global enviromental parameters : Place here the environment data you want to measure


static float temp = 0.0;
static float hum = 0.0;
static float pres = 0.0;
static float batvalue = 0.0;


BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,



// Pin mapping for RFM95
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
      debugPrintLn(F("Datarate: SF12"));
      TX_INTERVAL = 4800;
      break;
    case DR_SF11: debugPrintLn(F("Datarate: SF11"));
      TX_INTERVAL = 2400;
      break;
    case DR_SF10: debugPrintLn(F("Datarate: SF10"));
      TX_INTERVAL = 1200;
      break;
    case DR_SF9: debugPrintLn(F("Datarate: SF9"));
      TX_INTERVAL = 600;
      break;
    case DR_SF8: debugPrintLn(F("Datarate: SF8"));
      TX_INTERVAL = 360;
      break;
    case DR_SF7: debugPrintLn(F("Datarate: SF7"));
      TX_INTERVAL = 180;
      break;
    case DR_SF7B: debugPrintLn(F("Datarate: SF7B"));
      TX_INTERVAL = 180;
      break;
    case DR_FSK: debugPrintLn(F("Datarate: FSK"));
      TX_INTERVAL = 180;
      break;
    default: debugPrint(F("Datarate Unknown Value: "));
      debugPrintLn(LMIC.datarate); TX_INTERVAL = 600;
      break;
  }
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
  delay(100); //Wait for serial to complete
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

void updateEnvParameters() // place here your sensing
{  
  
  batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt 

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);


  #ifdef SHOW_DEBUGINFO
  // print out the value you read:
  Serial.print(temp);
  Serial.println("°C");
  Serial.print(hum);
  Serial.println("%");
  Serial.print(pres);
  Serial.println("Pa");
  Serial.print("Vbatt : ");
  Serial.println(batvalue);
  #endif 
}


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      //debugPrintLn(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      //debugPrintLn(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      //debugPrintLn(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //debugPrintLn(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      debugPrintLn(F("EV_JOINING"));
      
      break;
    case EV_JOINED:
      debugPrintLn(F("EV_JOINED"));
      setDataRate();
      
      // Ok send our first data in 10 ms
      os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      break;
    case EV_RFU1:
      debugPrintLn(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      debugPrintLn(F("EV_JOIN_FAILED"));
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_REJOIN_FAILED:
      debugPrintLn(F("EV_REJOIN_FAILED"));
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_TXCOMPLETE:
      debugPrintLn(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        debugPrintLn(F("Received ack"));
      if (LMIC.dataLen) {
        debugPrintLn(F("Received "));
        debugPrintLn(LMIC.dataLen);
        debugPrintLn(F(" bytes of payload"));
      }
            // Schedule next transmission
      setDataRate();
      do_sleep(TX_INTERVAL);
      os_setCallback(&sendjob, do_send);
      break;
    case EV_LOST_TSYNC:
      debugPrintLn(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      debugPrintLn(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      debugPrintLn(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      debugPrintLn(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      debugPrintLn(F("EV_LINK_ALIVE"));
      break;
    default:
      debugPrintLn(F("Unknown event"));
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
    
    updateEnvParameters(); // Sensing parameters are updated
   

#ifdef SHOW_DEBUGINFO
    debugPrint(F("T="));
    debugPrintLn(temp);
    debugPrint(F("BV="));
    debugPrintLn(batvalue);
#endif

// Formatting for Cayenne LPP
    
    int t = (int)((temp) * 10.0);  // multiply by 10 Cayenne
    // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250]
    int h = (int)(hum * 2.0); // Cayene in 0.5%
    unsigned int p = (unsigned int)(pres / 10.0); // Cayenne in 0.1 hPa unsigned
    int bat = batvalue; // multifly by 10 for V in Cayenne

    unsigned char mydata[15];
    mydata[0] = 0x1; // 1st Channel
    mydata[1] = 0x67; // Temp
    mydata[2] = t >> 8;
    mydata[3] = t & 0xFF; 
    mydata[4] = 0x2;  // 2nd Channel
    mydata[5] = 0x2;  // Analog Value
    mydata[6] = bat >> 8;
    mydata[7] = bat & 0xFF;
    mydata[8] = 0x3;  // 3rd Channel
    mydata[9] = 0x68;  // Humidity
    mydata[10] = h & 0xFF;
    mydata[11] = 0x4;  // 4th Channel
    mydata[12] = 0x73;  // Pressure
    mydata[13] = p >> 8;
    mydata[14] = p & 0xFF;
    
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugPrintLn(F("PQ")); //Packet queued
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void bmeStartup() {
while(!bme.begin())
  {
    #ifdef SHOW_DEBUGINFO
    Serial.println("Could not find BME280 sensor!");
    #endif
    delay(1000);
  }

  switch(bme.chipModel())
  {
     #ifdef SHOW_DEBUGINFO
     case BME280::ChipModel_BME280:       
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
     #endif
  }
  
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


// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  debugPrintLn(F("Starting"));
  delay(100);
  Wire.begin();

  bmeStartup(); // Start bme sensor
   
  updateEnvParameters(); // To have value for the first Tx
  

  // LMIC init

  os_init();
  lmicStartup();
  

}

void loop() {
  os_runloop_once();
}
