/*
 *  continous wave, based on LoRaMac-Node code
 *
 *  Copyright (C) 2017 Congduc Pham, University of Pau, France
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************
 * last update: Apr. 28th by C. Pham
 */
#include <SPI.h>  
// Include the SX1272
#include "SX1272.h"

// from LoRaMac-Node
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"

// wrap for our own function
#define SX1276Write(param1,param2) sx1272.writeRegister(param1,param2)
#define SX1276Read(param) sx1272.readRegister(param)

// from LoRaMac-Node, sx1276.h

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

// IMPORTANT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//
#define ETSI_EUROPE_REGULATION
//#define FCC_US_REGULATION
//#define SENEGAL_REGULATION
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

// IMPORTANT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// uncomment if your radio is an HopeRF RFM92W, HopeRF RFM95W, Modtronix inAir9B, NiceRF1276
// or you known from the circuit diagram that output use the PABOOST line instead of the RFO line
#define PABOOST
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

// IMPORTANT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
#define BAND868
//#define BAND900
//#define BAND433
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ETSI_EUROPE_REGULATION
#define MAX_DBM 14
#elif defined SENEGAL_REGULATION
#define MAX_DBM 10
#endif

#ifdef BAND868
#ifdef SENEGAL_REGULATION
const uint32_t DEFAULT_CHANNEL=CH_04_868;
#else
const uint32_t DEFAULT_CHANNEL=CH_10_868;
#endif
#elif defined BAND900
const uint32_t DEFAULT_CHANNEL=CH_05_900;
#elif defined BAND433
const uint32_t DEFAULT_CHANNEL=CH_00_433;
#endif

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE LORA MODE, NODE ADDRESS 
#define LORAMODE  1
#define node_addr 8
//////////////////////////////////////////////////////////////////

// we wrapped Serial.println to support the Arduino Zero or M0
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0
#define PRINTLN                   SerialUSB.println("")              
#define PRINT_CSTSTR(fmt,param)   SerialUSB.print(F(param))
#define PRINT_STR(fmt,param)      SerialUSB.print(param)
#define PRINT_VALUE(fmt,param)    SerialUSB.print(param)
#define FLUSHOUTPUT               SerialUSB.flush();
#else
#define PRINTLN                   Serial.println("")
#define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define FLUSHOUTPUT               Serial.flush();
#endif

#define DEFAULT_DEST_ADDR 1

int loraMode=LORAMODE;

void setup()
{
  int e;
  
  // Open serial communications and wait for port to open:
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0 
  SerialUSB.begin(38400);
#else
  Serial.begin(38400);  
#endif 

  // Print a start message
  PRINT_CSTSTR("%s","Continous wave\n");  

#ifdef ARDUINO_AVR_PRO
  PRINT_CSTSTR("%s","Arduino Pro Mini detected\n");
#endif

#ifdef ARDUINO_AVR_NANO
  PRINT_CSTSTR("%s","Arduino Nano detected\n");
#endif

#ifdef ARDUINO_AVR_MINI
  PRINT_CSTSTR("%s","Arduino MINI/Nexus detected\n");
#endif

#ifdef __MK20DX256__
  PRINT_CSTSTR("%s","Teensy31/32 detected\n");
#endif

#ifdef __SAMD21G18A__ 
  PRINT_CSTSTR("%s","Arduino M0/Zero detected\n");
#endif

  // Power ON the module
  sx1272.ON();
  
  // Set transmission mode and print the result
  e = sx1272.setMode(loraMode);
  PRINT_CSTSTR("%s","Setting Mode: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
    
  // Select frequency channel
  e = sx1272.setChannel(DEFAULT_CHANNEL);
  PRINT_CSTSTR("%s","Setting Channel: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  
  // Select amplifier line; PABOOST or RFO
#ifdef PABOOST
  sx1272._needPABOOST=true;
  // previous way for setting output power
  // powerLevel='x';
#else
  // previous way for setting output power
  // powerLevel='M';  
#endif

  e = sx1272.setPowerDBM((uint8_t)MAX_DBM); 
  PRINT_CSTSTR("%s","Setting Power: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  
  // Set the node address and print the result
  e = sx1272.setNodeAddress(node_addr);
  PRINT_CSTSTR("%s","Setting node addr: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  /////////////////////////////////////////////////////////////////////////////
  
  PRINT_CSTSTR("%s","SX1276 configuring for continuous wave mode in FSK\n");

  // set to sleep mode
  SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );
  // set to FSK mode        
  SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
  SX1276Write( REG_DIOMAPPING1, 0x00 );
  SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady

  uint32_t fdev = 0;
  uint32_t datarate = 4800;
  uint16_t preambleLen = 5;
  bool fixLen = false;
  bool crcOn = false;

  // taken from SX1276SetTxConfig
  //    
  fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
  SX1276Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
  SX1276Write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );
  
  datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
  SX1276Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
  SX1276Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );
  
  SX1276Write( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
  SX1276Write( REG_PREAMBLELSB, preambleLen & 0xFF );

  SX1276Write( REG_PACKETCONFIG1,
               ( SX1276Read( REG_PACKETCONFIG1 ) &
                 RF_PACKETCONFIG1_CRC_MASK &
                 RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                 ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                 ( crcOn << 4 ) );
  SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );    

  // taken from SX1272SetTxContinuousWave
  //
  SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
  // Disable radio interrupts
  SX1276Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
  SX1276Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );
      
  // here we transmit
  SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_TRANSMITTER );
      
  /////////////////////////////////////////////////////////////////////////////

  // change here for longer continous wave
  // here 180000ms = 180s = 3mm
  delay(1800000);

  // set to sleep mode
  SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );
}

void loop(void)
{
        
}
