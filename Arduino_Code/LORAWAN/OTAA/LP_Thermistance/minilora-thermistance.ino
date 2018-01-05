



/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

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

 *******************************************************************************/
/*
 ****************************************************************************************
   TheThingsNetwork Payload functions :
    function Decoder(bytes, port)
    {
      var retValue =   {
        bytes: bytes
      };

      retValue.batt = bytes[0] / 10.0;
      if (retValue.batt === 0)
         delete retValue.batt;

      if (bytes.length >= 2)
      {
        retValue.humidity = bytes[1];
        if (retValue.humidity === 0)
          delete retValue.humidity;
      }
      if (bytes.length >= 3)
      {
        retValue.temperature = (((bytes[2] << 8) | bytes[3]) / 10.0) - 40.0;
      }
      if (bytes.length >= 5)
      {
        retValue.pressure = ((bytes[4] << 8) | bytes[5]);
        if (retValue.pressure === 0)
          delete retValue.pressure;
      }

      return retValue;
    }
*/


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>



#include  "adcvcc.h"

//#include <BME280I2C.h>
#include "LowPower.h"
#include "keys.h"

#define debugSerial Serial
#define SHOW_DEBUGINFO
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }


// Pin mapping for the RGBLED object:
#define redLed  9
#define greenLed  6
#define blueLed  5

//Commented out keys have been zeroed for github - the lines can be copied to a keys.h file and real keys inserted


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0xE9, 0x8E, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x04, 0x00, 0x00, 0x71, 0x00, 0x5F, 0x63, 0x20 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x41, 0x01, 0x3E, 0x98, 0xF8, 0xD4, 0x3F, 0xD9, 0x0F, 0x99, 0xF5, 0xD1, 0x87, 0x5C, 0xB4, 0x51 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}


static osjob_t sendjob;

// global enviromental parameters
static float temp = 0.0;
//static float pressure = 0.0;
static float humidity = 0.0;

static float R2 = 10000;
static float Vcc = 5;
static float T0= 298.15;
static float beta= 3977;
static float R0 = 10000;



/* ======================================================================
  Function: ADC_vect
  Purpose : IRQ Handler for ADC
  Input   : -
  Output  : -
  Comments: used for measuring 8 samples low power mode, ADC is then in
          free running mode for 8 samples
  ====================================================================== */
ISR(ADC_vect)
{
  // Increment ADC counter
  _adc_irq_cnt++;
}


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned int TX_INTERVAL = 300;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {2, 7, 9},
};

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

void updateEnvParameters()
{
  int sensorValue = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  float resistance = R2 * (1-(voltage/Vcc))/(voltage/Vcc); // Thermistance between Vcc and A0, 10kohm resistor between A0 and gnd
  float ln = log (resistance / R0);
  temp = (beta / (ln+ (beta/T0)))- 274.15;
  // print out the value you read:
  Serial.print("Voltage : ");
  Serial.println(voltage);
  Serial.print("Resistance : ");
  Serial.println(resistance);
  Serial.print("T°c : ");
  Serial.println(temp);
  
   
 
}


void setColor(bool redValue, bool greenValue, bool blueValue) {
  digitalWrite(redLed, !redValue);
  digitalWrite(greenLed, !greenValue);
  digitalWrite(blueLed, !blueValue);
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
      setColor(1, 0, 0);
      break;
    case EV_JOINED:
      debugPrintLn(F("EV_JOINED"));
      setDataRate();
      setColor(1, 1, 0);
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
    //  Pressure: 300...1100 hPa
    //  Temperature: -40…85°C
    updateEnvParameters();
    

    int batt = (int)(10*readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps // and multifly by 10 for V in Cayenne
    byte batvalue = (byte)batt; // no problem putting it into a int.

#ifdef SHOW_DEBUGINFO
    debugPrint(F("T="));
    debugPrintLn(temp);

    debugPrint(F("H="));
    debugPrintLn(humidity);
    debugPrint(F("BV="));
    debugPrintLn(batvalue);
#endif
    int t = (int)((temp) * 10.0);
    // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250]
    
    int h = (int)(humidity * 2.0);

    unsigned char mydata[8];
    mydata[0] = 0x1;
    mydata[1] = 0x67;
    mydata[2] = t >> 8;
    mydata[3] = t & 0xFF;
    mydata[4] = 0x3;
    mydata[5] = 0x2;
    mydata[6] = 0x0;
    mydata[7] = batvalue;
    
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugPrintLn(F("PQ")); //Packet queued
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void lmicStartup() {
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif


  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  LMIC_setLinkCheckMode(1);
  LMIC_setAdrMode(1);

  // Start job (sending automatically starts OTAA too)
  // Join the network, sending will be
  // started after the event "Joined"
  LMIC_startJoining();
}

void setup() {
  Serial.begin(9600);
  debugPrintLn(F("Starting"));
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  setColor(1, 0, 1);
  delay(500);
  Wire.begin();

  updateEnvParameters(); // To have value for the first Tx
  

  // LMIC init
  os_init();
  lmicStartup();

}

void loop() {
  os_runloop_once();
}
