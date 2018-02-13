
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
 and to remotly control a RGB LED
 ****************************************************************************************
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>


//Sensors librairies

#define debugSerial Serial
#define SHOW_DEBUGINFO
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }

// Pin mapping for the RGBLED object:
#define GREEN 3
#define BLUE 6
#define RED 5



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


static float batvalue = 0.0;

static int LED_RED = 0;
static int LED_BLUE = 0;
static int LED_GREEN = 0;



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
      TX_INTERVAL = 30;
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

void setColor(int redValue,  int blueValue, int greenValue) {
  analogWrite(RED, redValue);
  analogWrite(GREEN, greenValue);
  analogWrite(BLUE, blueValue);
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



void updateEnvParameters() // place here your sensing
{  
  
  batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt 

  


  #ifdef SHOW_DEBUGINFO
  // print out the value you read:
  Serial.print("Vbatt : ");
  Serial.println(batvalue);
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
    setColor(0, 255, 255);   //RED   
      break;
    case EV_JOINED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOINED"));
    #endif
    setColor(255, 255, 0); //GREEN  
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
        debugPrint(F("Received "));
        debugPrint(LMIC.dataLen/4);
        debugPrintLn(F(" downlink(s)"));
        for (int i = 0; i < LMIC.dataLen; i++) {
        if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
        }
        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
    }
    Serial.println();
    #endif 
     
      for(int i = 0; i < LMIC.dataLen/4;i++){
        
        switch (LMIC.frame[LMIC.dataBeg+4*i]){
        case 0x06 :
        LED_RED = (word(LMIC.frame[LMIC.dataBeg+1],LMIC.frame[LMIC.dataBeg+2]))/100; // Converter download payload in int
        #ifdef SHOW_DEBUGINFO
        Serial.println(LED_RED);
        #endif
        break;
       
        case 0x07 :
        LED_BLUE = (word(LMIC.frame[LMIC.dataBeg+1],LMIC.frame[LMIC.dataBeg+2]))/100; // Converter download payload in int
        #ifdef SHOW_DEBUGINFO
        Serial.println(LED_BLUE);
        #endif
        break;

        case 0x08 :
        LED_GREEN = (word(LMIC.frame[LMIC.dataBeg+1],LMIC.frame[LMIC.dataBeg+2]))/100; // Converter download payload in int
        #ifdef SHOW_DEBUGINFO
        Serial.println(LED_GREEN);
        #endif
        break;
        }
      }
     }
     delay(5);
      setColor(255-LED_RED, 255-LED_BLUE, 255-LED_GREEN);
     delay(5000);
           
       // Schedule next transmission
      setDataRate();
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
    
    updateEnvParameters(); // Sensing parameters are updated
   

#ifdef SHOW_DEBUGINFO
    debugPrint(F("BV="));
    debugPrintLn(batvalue);
#endif

// Formatting for Cayenne LPP
    
    
    int bat = batvalue; // multifly by 10 for V in Cayenne

    unsigned char mydata[17];
    mydata[1] = 0x2;  // 2nd Channel
    mydata[2] = 0x2;  // Analog Value
    mydata[3] = bat >> 8;
    mydata[4] = bat & 0xFF;
    mydata[5] = 0x6;  // 6th Channel
    mydata[6] = 0x3;  // Analog Value PWM red
    mydata[7] = LED_RED >> 8;
    mydata[8] = LED_RED & 0xFF;
    mydata[9] = 0x7;  // 7th Channel
    mydata[10] = 0x3;  // Analog Value PWM red
    mydata[11] = LED_BLUE >> 8;
    mydata[12] = LED_BLUE & 0xFF;
    mydata[13] = 0x8;  // 8th Channel
    mydata[14] = 0x3;  // Analog Value PWM red
    mydata[15] = LED_GREEN >> 8;
    mydata[16] = LED_GREEN & 0xFF;
    
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


// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used
  
  Serial.begin(115200);
  
  #ifdef SHOW_DEBUGINFO
  debugPrintLn(F("Starting"));
  delay(100);
  #endif
  
  Wire.begin();

  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(RED, OUTPUT);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
  digitalWrite(RED, HIGH);

  updateEnvParameters(); // To have value for the first Tx
  

  // LMIC init

  os_init();
  lmicStartup();  

}

void loop() {
  os_runloop_once();
}
