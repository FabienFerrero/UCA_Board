/* 
 *  Library for LoRa 868 / 915MHz SX1272 LoRa module
 *
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Version:           1.1
 *  Design:            David Gascón
 *  Implementation:    Covadonga Albiñana & Victor Boria
 */

//**********************************************************************/
// Includes
//**********************************************************************/

#include "SX1272.h"
#include <SPI.h>

/*  CHANGE LOGS by C. Pham
 *	June 29th, 2018
 *		- SX1272_WRST (not defined by default) controls whether there will be a RST procedure or not. Normally, there is no need for the RST.
 *		- If RST is used, it is currently pin 4 on the Arduino but sometimes there is conflict, so better not use RST
 *  March 28th, 2018
 *		- check at packet reception that the packet type is correct, otherwise discard the packet and returned error code is 5
 *      - add max number of retries for CarrierSense
 *  Feb 28th, 2018
 *		- there is no longer is_binary flag, replaced by is_downlink flag
 *		- the flags are then from left to right: ack_requested|encrypted|with_appkey|is_downlink
 *  Feb 25th, 2018
 *      - use shared payload buffer for packet_sent and packet_received
 *      - use dedicated smaller buffer for ACK
 *  Feb 13th, 2018
 *      - fix bug in availableData() to set back the LoRa module into standby mode. This affected only some radio modules
 *	Jan 19th, 2018
 *		- add a setCSPin(uint8_t cs) function to set the Chip Select (CS) pin
 *		- call sx1272.setCSPin(18) for instance before calling sx1272.ON()
 *		- by default, the CS pin will be set to SX1272_SS defined in SX1272.h
 *  November 10th, 2017
 *		- change the way packet's RSSI is computed
 *  November 7th, 2017
 *      - bug fix in how the CRC is checked at receiver in getPacket() function
 *  November 3rd, 2017
 *      - IMPORTANT: the CS pin is now always pin number 10 on Arduino boards
 *      - if you use the Libelium Multiprotocol shield to connect a Libelium LoRa then change the CS pin to pin 2 in SX1272.h
 *      - CRC (RxPayloadCrcOn) is now ON by default for transmitter side (end-device)
 *  June, 22th, 2017
 *      - setPowerDBM(uint8_t dbm) calls setPower('X') when dbm is set to 20
 *  Apr, 21th, 2017
 *      - change the way timeout are detected: exitTime=millis()+(unsigned long)wait; then millis() < exitTime;
 *  Mar, 26th, 2017
 *      - insert delay(100) before setting radio module to sleep mode. Remove unstability issue
 *      - (proposed by escyes - https://github.com/CongducPham/LowCostLoRaGw/issues/53#issuecomment-289237532)
 *  Jan, 11th, 2017
 *      - fix bug in getRSSIpacket() when SNR < 0 thanks to John Rohde from Aarhus University
 *  Dec, 17th, 2016
 *      - fix bug making -DPABOOST in radio.makefile inoperant
 *  Dec, 1st, 2016
 *      - add RSSI computation while performing CAD with doCAD()
 *      - WARNING: the SX1272 lib for gateway (Raspberry) does not have this functionality
 *  Nov, 26th, 2016
 *		- add preliminary support for ToA limitation
 *      - when in "production" mode, uncomment #define LIMIT_TOA
 *  Nov, 16th, 2016
 *		- provide better power management mechanisms
 *		- manage PA_BOOST and dBm setting 
 *  Jan, 23rd, 2016
 *      - the packet format at transmission does not use the original Libelium format anymore
 *      * the retry field is removed therefore all operations using retry will probably not work well, not tested though
 *          - therefore DO NOT use sendPacketTimeoutACKRetries()
 *          - the reason is that we do not want to have a reserved byte after the payload
 *      * the length field is removed because it is much better to get the packet length at the reception side
 *      * after the dst field, we inserted a packet type field to better identify the packet type: DATA, ACK, encryption, app key,...
 *          - the format is now dst(1B) ptype(1B) src(1B) seq(1B) payload(xB)
 *          - ptype is decomposed in 2 parts type(4bits) flags(4bits)
 *          - type can take current value of DATA=0001 and ACK=0010
 *          - the flags are from left to right: ack_requested|encrypted|with_appkey|is_binary
 *          - ptype can be set with setPacketType(), see constant defined in SX1272.h
 *          - the header length is then 4 instead of 5
 *  Jan, 16th, 2016
 *      - add support for SX1276, automatic detect
 *      - add LF/HF calibaration copied from LoRaMAC-Node. Don't know if it is really necessary though
 *      - change various radio settings
 *  Dec, 10th, 2015
 *      - add SyncWord for test with simple LoRaWAN
 *      - add mode 11 that have BW=125, CR=4/5, SF=7 on channel 868.1MHz
 *          - use following in your code if (loraMode==11) { e = sx1272.setChannel(CH_18_868); }
 *  Nov, 13th, 2015
 *      - add CarrierSense() to perform some Listen Before Talk procedure
 *      - add dynamic ACK suport
 *          - compile with W_REQUESTED_ACK, retry field is used to indicate at the receiver
 *			  that an ACK should be sent
 *          - receiveWithTimeout() has been modified to send an ACK if retry is 1
 *          - at sender side, sendPacketTimeoutACK() has been modified to indicate
 *			  whether retry should be set to 1 or not in setPacket()
 *          - receiver should always use receiveWithTimeout() while sender decides to use
 *			  sendPacketTimeout() or sendPacketTimeoutACK()
 *  Jun, 2015
 *      - Add time on air computation and CAD features
*/

// Added by C. Pham
// based on SIFS=3CAD
uint8_t sx1272_SIFS_value[11]={0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4};
uint8_t sx1272_CAD_value[11]={0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1};

//#define LIMIT_TOA
// 0.1% for testing
//#define MAX_DUTY_CYCLE_PER_HOUR 3600L
// 1%, regular mode
#define MAX_DUTY_CYCLE_PER_HOUR 36000L
// normally 1 hour, set to smaller value for testing
#define DUTYCYCLE_DURATION 3600000L
// 4 min for testing
//#define DUTYCYCLE_DURATION 240000L

// end

//**********************************************************************/
// Public functions.
//**********************************************************************/

SX1272::SX1272()
{
	//set the Chip Select pin
	_SX1272_SS=SX1272_SS;
	
    // Initialize class variables
    _bandwidth = BW_125;
    _codingRate = CR_5;
    _spreadingFactor = SF_7;
    _channel = CH_12_900;
    _header = HEADER_ON;
    _CRC = CRC_OFF;
    _modem = FSK;
    _power = 15;
    _packetNumber = 0;
    _reception = CORRECT_PACKET;
    _retries = 0;
    // added by C. Pham
    _defaultSyncWord=0x12;
    _rawFormat=false;
    _extendedIFS=false;
    _RSSIonSend=true;
    // disabled by default
    _enableCarrierSense=false;
    // DIFS by default
    _send_cad_number=9;
#ifdef PABOOST
    _needPABOOST=true;
#else
    _needPABOOST=false;
#endif
    _limitToA=false;
    _startToAcycle=millis();
    _remainingToA=MAX_DUTY_CYCLE_PER_HOUR;
    _endToAcycle=_startToAcycle+DUTYCYCLE_DURATION;
#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
#ifdef W_NET_KEY
    _my_netkey[0] = net_key_0;
    _my_netkey[1] = net_key_1;
#endif
    // we use the same memory area to reduce memory footprint
    packet_sent.data=packet_data;
    packet_received.data=packet_data;
    // ACK packet has a very small separate memory area
    ACK.data=ack_data;
    // end
    // modified by C. Pham
    _maxRetries = 0;
    packet_sent.retry = _retries;
};

// added by C. Pham
// copied from LoRaMAC-Node
/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
void SX1272::RxChainCalibration()
{
    if (_board==SX1276Chip) {

        Serial.println(F("SX1276 LF/HF calibration"));

        // Cut the PA just in case, RFO output, power = -1 dBm
        writeRegister( REG_PA_CONFIG, 0x00 );
    
        // Launch Rx chain calibration for LF band
        writeRegister( REG_IMAGE_CAL, ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
        while( ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
        {
        }
    
        // Sets a Frequency in HF band
        setChannel(CH_17_868);
    
        // Launch Rx chain calibration for HF band
        writeRegister( REG_IMAGE_CAL, ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
        while( ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
        {
        }
    }
}


/*
 Function: Sets the module ON.
 Returns: uint8_t setLORA state
*/
uint8_t SX1272::ON()
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'ON'"));
#endif

    // Powering the module
    pinMode(_SX1272_SS,OUTPUT);
    digitalWrite(_SX1272_SS,HIGH);
    delay(100);

    //#define USE_SPI_SETTINGS

#ifdef USE_SPI_SETTINGS
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
#else
    //Configure the MISO, MOSI, CS, SPCR.
    SPI.begin();
    //Set Most significant bit first
    SPI.setBitOrder(MSBFIRST);
#ifdef _VARIANT_ARDUINO_DUE_X_ 
    // for the DUE, set to 4MHz
    SPI.setClockDivider(42);
#else
    // for the MEGA, set to 2MHz
    SPI.setClockDivider(SPI_CLOCK_DIV8);
#endif   
    //Set data mode
    SPI.setDataMode(SPI_MODE0);
#endif  

    delay(100);

#ifdef SX1272_WRST
    // added by C. Pham
    pinMode(SX1272_RST,OUTPUT);
    digitalWrite(SX1272_RST,HIGH);
    delay(100);
    digitalWrite(SX1272_RST,LOW);
    delay(100);
#endif
    
    // from single_chan_pkt_fwd by Thomas Telkamp
    uint8_t version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        Serial.println(F("SX1272 detected, starting"));
        _board = SX1272Chip;
    } else {
        // sx1276?
#ifdef SX1272_WRST        
        digitalWrite(SX1272_RST, LOW);
        delay(100);
        digitalWrite(SX1272_RST, HIGH);
        delay(100);
#endif        
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            Serial.println(F("SX1276 detected, starting"));
            _board = SX1276Chip;
        } else {
            Serial.println(F("Unrecognized transceiver"));
        }
    }
    // end from single_chan_pkt_fwd by Thomas Telkamp

    // added by C. Pham
    RxChainCalibration();

    setMaxCurrent(0x1B);
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Setting ON with maximum current supply ##"));
    Serial.println();
#endif

    // set LoRa mode
    state = setLORA();

    // Added by C. Pham     
    // set CRC ON
    setCRC_ON();

    // Added by C. Pham for ToA computation
    getPreambleLength();
#ifdef W_NET_KEY
    //#if (SX1272_debug_mode > 1)
    Serial.println(F("## SX1272 layer has net key##"));
    //#endif
#endif

#ifdef W_INITIALIZATION
    // CAUTION
    // doing initialization as proposed by Libelium seems not to work for the SX1276
    // so we decided to leave the default value of the SX127x, then configure the radio when
    // setting to LoRa mode

    //Set initialization values
    writeRegister(0x0,0x0);
    // comment by C. Pham
    // still valid for SX1276
    writeRegister(0x1,0x81);
    // end
    writeRegister(0x2,0x1A);
    writeRegister(0x3,0xB);
    writeRegister(0x4,0x0);
    writeRegister(0x5,0x52);
    writeRegister(0x6,0xD8);
    writeRegister(0x7,0x99);
    writeRegister(0x8,0x99);
    // modified by C. Pham
    // added by C. Pham
    if (_board==SX1272Chip)
        // RFIO_pin RFU OutputPower
        // 0 000 0000
        writeRegister(0x9,0x0);
    else
        // RFO_pin MaxP OutputPower
        // 0 100 1111
        // set MaxPower to 0x4 and OutputPower to 0
        writeRegister(0x9,0x40);

    writeRegister(0xA,0x9);
    writeRegister(0xB,0x3B);

    // comment by C. Pham
    // still valid for SX1276
    writeRegister(0xC,0x23);

    // REG_RX_CONFIG
    writeRegister(0xD,0x1);

    writeRegister(0xE,0x80);
    writeRegister(0xF,0x0);
    writeRegister(0x10,0x0);
    writeRegister(0x11,0x0);
    writeRegister(0x12,0x0);
    writeRegister(0x13,0x0);
    writeRegister(0x14,0x0);
    writeRegister(0x15,0x0);
    writeRegister(0x16,0x0);
    writeRegister(0x17,0x0);
    writeRegister(0x18,0x10);
    writeRegister(0x19,0x0);
    writeRegister(0x1A,0x0);
    writeRegister(0x1B,0x0);
    writeRegister(0x1C,0x0);

    // added by C. Pham
    if (_board==SX1272Chip) {
        // comment by C. Pham
        // 0x4A = 01 001 0 1 0
        // BW=250 CR=4/5 ImplicitH_off RxPayloadCrcOn_on LowDataRateOptimize_off
        writeRegister(0x1D,0x4A);
        // 1001 0 1 11
        // SF=9 TxContinuous_off AgcAutoOn SymbTimeOut
        writeRegister(0x1E,0x97);
    }
    else {
        // 1000 001 0
        // BW=250 CR=4/5 ImplicitH_off
        writeRegister(0x1D,0x82);
        // 1001 0 1 11
        // SF=9 TxContinuous_off RxPayloadCrcOn_on SymbTimeOut
        writeRegister(0x1E,0x97);
    }
    // end

    writeRegister(0x1F,0xFF);
    writeRegister(0x20,0x0);
    writeRegister(0x21,0x8);
    writeRegister(0x22,0xFF);
    writeRegister(0x23,0xFF);
    writeRegister(0x24,0x0);
    writeRegister(0x25,0x0);

    // added by C. Pham
    if (_board==SX1272Chip)
        writeRegister(0x26,0x0);
    else
        // 0000 0 1 00
        // reserved LowDataRateOptimize_off AgcAutoOn reserved
        writeRegister(0x26,0x04);

    // REG_SYNC_CONFIG
    writeRegister(0x27,0x0);

    writeRegister(0x28,0x0);
    writeRegister(0x29,0x0);
    writeRegister(0x2A,0x0);
    writeRegister(0x2B,0x0);
    writeRegister(0x2C,0x0);
    writeRegister(0x2D,0x50);
    writeRegister(0x2E,0x14);
    writeRegister(0x2F,0x40);
    writeRegister(0x30,0x0);
    writeRegister(0x31,0x3);
    writeRegister(0x32,0x5);
    writeRegister(0x33,0x27);
    writeRegister(0x34,0x1C);
    writeRegister(0x35,0xA);
    writeRegister(0x36,0x0);
    writeRegister(0x37,0xA);
    writeRegister(0x38,0x42);
    writeRegister(0x39,0x12);
    //writeRegister(0x3A,0x65);
    //writeRegister(0x3B,0x1D);
    //writeRegister(0x3C,0x1);
    //writeRegister(0x3D,0xA1);
    //writeRegister(0x3E,0x0);
    //writeRegister(0x3F,0x0);
    //writeRegister(0x40,0x0);
    //writeRegister(0x41,0x0);
    // commented by C. Pham
    // since now we handle also the SX1276
    //writeRegister(0x42,0x22);
#endif
    // added by C. Pham
    // default sync word for non-LoRaWAN
    setSyncWord(_defaultSyncWord);
    getSyncWord();
    _defaultSyncWord=_syncWord;

#ifdef LIMIT_TOA
    uint16_t remainingToA=limitToA();
    Serial.println(F("## Limit ToA ON ##"));
    Serial.print(F("cycle begins at "));
    Serial.print(_startToAcycle);
    Serial.print(F(" cycle ends at "));
    Serial.print(_endToAcycle);
    Serial.print(F(" remaining ToA is "));
    Serial.print(remainingToA);
    Serial.println();
#endif
    //end

    //init random generator
    randomSeed(millis());

    return state;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void SX1272::OFF()
{
#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'OFF'"));
#endif

    SPI.end();
    // Powering the module
    pinMode(_SX1272_SS,OUTPUT);
    digitalWrite(_SX1272_SS,LOW);
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Setting OFF ##"));
    Serial.println();
#endif
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
byte SX1272::readRegister(byte address)
{
    byte value = 0x00;

    digitalWrite(_SX1272_SS,LOW);
    bitClear(address, 7);		// Bit 7 cleared to write in registers
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    digitalWrite(_SX1272_SS,HIGH);

#if (SX1272_debug_mode > 1)
    Serial.print(F("## Reading:  ##\t"));
    Serial.print(F("Register "));
    Serial.print(address, HEX);
    Serial.print(F(":  "));
    Serial.print(value, HEX);
    Serial.println();
#endif

    return value;
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void SX1272::writeRegister(byte address, byte data)
{
    digitalWrite(_SX1272_SS,LOW);
    bitSet(address, 7);			// Bit 7 set to read from registers
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(_SX1272_SS,HIGH);

#if (SX1272_debug_mode > 1)
    Serial.print(F("## Writing:  ##\t"));
    Serial.print(F("Register "));
    bitClear(address, 7);
    Serial.print(address, HEX);
    Serial.print(F(":  "));
    Serial.print(data, HEX);
    Serial.println();
#endif

}

/*
 Function: Clears the interruption flags
 Returns: Nothing
*/
void SX1272::clearFlags()
{
    byte st0;

    st0 = readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby mode to write in registers
        writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register
        writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
#if (SX1272_debug_mode > 1)
        Serial.println(F("## LoRa flags cleared ##"));
#endif
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby mode to write in registers
        writeRegister(REG_IRQ_FLAGS1, 0xFF); // FSK mode flags1 register
        writeRegister(REG_IRQ_FLAGS2, 0xFF); // FSK mode flags2 register
        writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
#if (SX1272_debug_mode > 1)
        Serial.println(F("## FSK flags cleared ##"));
#endif
    }
}

/*
 Function: Sets the module in LoRa mode.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setLORA()
{
    uint8_t state = 2;
    byte st0;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setLORA'"));
#endif

    // modified by C. Pham
    uint8_t retry=0;

    do {
        delay(200);
        writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);    // Sleep mode (mandatory to set LoRa mode)
        writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
        delay(50+retry*10);
        st0 = readRegister(REG_OP_MODE);
        Serial.println(F("..."));

        if ((retry % 2)==0) {
            if (retry==20)
                retry=0;
            else
                retry++;
        }       
        /*
        if (st0!=LORA_STANDBY_MODE) {
            pinMode(SX1272_RST,OUTPUT);
            digitalWrite(SX1272_RST,HIGH);
            delay(100);
            digitalWrite(SX1272_RST,LOW);
        }
        */

    } while (st0!=LORA_STANDBY_MODE);	// LoRa standby mode

    if( st0 == LORA_STANDBY_MODE)
    { // LoRa mode
        _modem = LORA;
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## LoRa set with success ##"));
        Serial.println();
#endif
    }
    else
    { // FSK mode
        _modem = FSK;
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while setting LoRa **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Sets the module in FSK mode.
 Returns:   Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setFSK()
{
    uint8_t state = 2;
    byte st0;
    byte config1;

    if (_board==SX1276Chip)
        Serial.println(F("Warning: FSK has not been tested on SX1276!"));

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setFSK'"));
#endif

    writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
    writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
    config1 = readRegister(REG_PACKET_CONFIG1);
    config1 = config1 & B01111101;		// clears bits 8 and 1 from REG_PACKET_CONFIG1
    config1 = config1 | B00000100;		// sets bit 2 from REG_PACKET_CONFIG1
    writeRegister(REG_PACKET_CONFIG1,config1);	// AddressFiltering = NodeAddress + BroadcastAddress
    writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
    config1 = readRegister(REG_SYNC_CONFIG);
    config1 = config1 & B00111111;
    writeRegister(REG_SYNC_CONFIG,config1);

    delay(100);

    st0 = readRegister(REG_OP_MODE);	// Reading config mode
    if( st0 == FSK_STANDBY_MODE )
    { // FSK mode
        _modem = FSK;
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## FSK set with success ##"));
        Serial.println();
#endif
    }
    else
    { // LoRa mode
        _modem = LORA;
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while setting FSK **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Gets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getMode()
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getMode'"));
#endif

    st0 = readRegister(REG_OP_MODE);		// Save the previous status
    if( _modem == FSK )
    {
        setLORA();					// Setting LoRa mode
    }
    value = readRegister(REG_MODEM_CONFIG1);
    // added by C. Pham
    if (_board==SX1272Chip) {
        _bandwidth = (value >> 6);   			// Storing 2 MSB from REG_MODEM_CONFIG1 (=_bandwidth)
        // added by C. Pham
        // convert to common bandwidth values used by both SX1272 and SX1276
        _bandwidth += 7;
    }
    else
        _bandwidth = (value >> 4);   			// Storing 4 MSB from REG_MODEM_CONFIG1 (=_bandwidth)

    if (_board==SX1272Chip)
        _codingRate = (value >> 3) & 0x07;  		// Storing third, forth and fifth bits from
    else
        _codingRate = (value >> 1) & 0x07;  		// Storing 3-1 bits REG_MODEM_CONFIG1 (=_codingRate)

    value = readRegister(REG_MODEM_CONFIG2);
    _spreadingFactor = (value >> 4) & 0x0F; 	// Storing 4 MSB from REG_MODEM_CONFIG2 (=_spreadingFactor)
    state = 1;

    if( isBW(_bandwidth) )		// Checking available values for:
    {								//		_bandwidth
        if( isCR(_codingRate) )		//		_codingRate
        {							//		_spreadingFactor
            if( isSF(_spreadingFactor) )
            {
                state = 0;
            }
        }
    }

#if (SX1272_debug_mode > 1)
    Serial.println(F("## Parameters from configuration mode are:"));
    Serial.print(F("Bandwidth: "));
    Serial.print(_bandwidth, HEX);
    Serial.println();
    Serial.print(F("\t Coding Rate: "));
    Serial.print(_codingRate, HEX);
    Serial.println();
    Serial.print(F("\t Spreading Factor: "));
    Serial.print(_spreadingFactor, HEX);
    Serial.println(F(" ##"));
    Serial.println();
#endif

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Sets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   mode: mode number to set the required BW, SF and CR of LoRa modem.
*/
int8_t SX1272::setMode(uint8_t mode)
{
    int8_t state = 2;
    byte st0;
    byte config1 = 0x00;
    byte config2 = 0x00;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setMode'"));
#endif

    st0 = readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
        setLORA();
    }
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode

    switch (mode)
    {
    // mode 1 (better reach, medium time on air)
    case 1:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_12);       // SF = 12
        setBW(BW_125);      // BW = 125 KHz
        break;

    // mode 2 (medium reach, less time on air)
    case 2:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_12);       // SF = 12
        setBW(BW_250);      // BW = 250 KHz
        break;

    // mode 3 (worst reach, less time on air)
    case 3:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_10);       // SF = 10
        setBW(BW_125);      // BW = 125 KHz
        break;

    // mode 4 (better reach, low time on air)
    case 4:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_12);       // SF = 12
        setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 5 (better reach, medium time on air)
    case 5:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_10);       // SF = 10
        setBW(BW_250);      // BW = 250 KHz
        break;

    // mode 6 (better reach, worst time-on-air)
    case 6:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_11);       // SF = 11
        setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 7 (medium-high reach, medium-low time-on-air)
    case 7:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_9);        // SF = 9
        setBW(BW_250);      // BW = 250 KHz
        break;

        // mode 8 (medium reach, medium time-on-air)
    case 8:     
    	setCR(CR_5);        // CR = 4/5
        setSF(SF_9);        // SF = 9
        setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 9 (medium-low reach, medium-high time-on-air)
    case 9:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_8);        // SF = 8
        setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 10 (worst reach, less time_on_air)
    case 10:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_7);        // SF = 7
        setBW(BW_500);      // BW = 500 KHz
        break;

    // added by C. Pham
    // test for LoRaWAN channel
    case 11:
        setCR(CR_5);        // CR = 4/5
        setSF(SF_12);        // SF = 12
        setBW(BW_125);      // BW = 125 KHz
        // set the sync word to the LoRaWAN sync word which is 0x34
        setSyncWord(0x34);
        Serial.print(F("** Using sync word of 0x"));
        Serial.println(_syncWord, HEX);
        break;

    default:    state = -1; // The indicated mode doesn't exist

    };

    if( state == -1 )	// if state = -1, don't change its value
    {
#if (SX1272_debug_mode > 1)
        Serial.print(F("** The indicated mode doesn't exist, "));
        Serial.println(F("please select from 1 to 10 **"));
#endif
    }
    else
    {
        state = 1;
        config1 = readRegister(REG_MODEM_CONFIG1);
        switch (mode)
        {   //      Different way to check for each mode:
        // (config1 >> 3) ---> take out bits 7-3 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
        // (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)

        // mode 1: BW = 125 KHz, CR = 4/5, SF = 12.
        case 1:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;


            // mode 2: BW = 250 KHz, CR = 4/5, SF = 12.
        case 2:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;

            // mode 3: BW = 125 KHz, CR = 4/5, SF = 10.
        case 3:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_10 )
                {
                    state = 0;
                }
            }
            break;

            // mode 4: BW = 500 KHz, CR = 4/5, SF = 12.
        case 4:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;

            // mode 5: BW = 250 KHz, CR = 4/5, SF = 10.
        case 5:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_10 )
                {
                    state = 0;
                }
            }
            break;

            // mode 6: BW = 500 KHz, CR = 4/5, SF = 11.
        case 6:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_11 )
                {
                    state = 0;
                }
            }
            break;

            // mode 7: BW = 250 KHz, CR = 4/5, SF = 9.
        case 7:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_9 )
                {
                    state = 0;
                }
            }
            break;

            // mode 8: BW = 500 KHz, CR = 4/5, SF = 9.
        case 8:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_9 )
                {
                    state = 0;
                }
            }
            break;

            // mode 9: BW = 500 KHz, CR = 4/5, SF = 8.
        case 9:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_8 )
                {
                    state = 0;
                }
            }
            break;

            // mode 10: BW = 500 KHz, CR = 4/5, SF = 7.
        case 10:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_7 )
                {
                    state = 0;
                }
            }
            break;

            // added by C. Pham
            // test of LoRaWAN channel
            // mode 11: BW = 125 KHz, CR = 4/5, SF = 12.
        case 11:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;
        }// end switch

        if (mode!=11) {
            setSyncWord(_defaultSyncWord);
#if (SX1272_debug_mode > 1)
            Serial.print(F("** Using sync word of 0x"));
            Serial.println(_defaultSyncWord, HEX);
#endif
        }
    }
    // added by C. Pham
    if (state == 0)
        _loraMode=mode;

#if (SX1272_debug_mode > 1)

    if( state == 0 )
    {
        Serial.print(F("## Mode "));
        Serial.print(mode, DEC);
        Serial.println(F(" configured with success ##"));
    }
    else
    {
        Serial.print(F("** There has been an error while configuring mode "));
        Serial.print(mode, DEC);
        Serial.println(F(". **"));
    }
#endif

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Indicates if module is configured in implicit or explicit header mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::getHeader()
{
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getHeader'"));
#endif

    // added by C. Pham
    uint8_t theHeaderBit;

    if (_board==SX1272Chip)
        theHeaderBit=2;
    else
        theHeaderBit=0;

    // take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
    if( bitRead(REG_MODEM_CONFIG1, theHeaderBit) == 0 )
    { // explicit header mode (ON)
        _header = HEADER_ON;
        state = 1;
    }
    else
    { // implicit header mode (OFF)
        _header = HEADER_OFF;
        state = 1;
    }

    state = 0;

    if( _modem == FSK )
    { // header is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Notice that FSK mode packets hasn't header ##"));
        Serial.println();
#endif
    }
    else
    { // header in LoRa mode
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Header is "));
        if( _header == HEADER_ON )
        {
            Serial.println(F("in explicit header mode ##"));
        }
        else
        {
            Serial.println(F("in implicit header mode ##"));
        }
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Sets the module in explicit header mode (header is sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::setHeaderON()
{
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setHeaderON'"));
#endif

    if( _modem == FSK )
    {
        state = -1;		// header is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("## FSK mode packets hasn't header ##"));
        Serial.println();
#endif
    }
    else
    {
        config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit
        if( _spreadingFactor == 6 )
        {
            state = -1;		// Mandatory headerOFF with SF = 6
#if (SX1272_debug_mode > 1)
            Serial.println(F("## Mandatory implicit header mode with spreading factor = 6 ##"));
#endif
        }
        else
        {
            // added by C. Pham
            if (_board==SX1272Chip)
                config1 = config1 & B11111011;		// clears bit 2 from config1 = headerON
            else
                config1 = config1 & B11111110;              // clears bit 0 from config1 = headerON

            writeRegister(REG_MODEM_CONFIG1,config1);	// Update config1
        }

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( _spreadingFactor != 6 )
        { // checking headerON taking out bit 2 from REG_MODEM_CONFIG1
            config1 = readRegister(REG_MODEM_CONFIG1);
            // modified by C. Pham
            if( bitRead(config1, theHeaderBit) == HEADER_ON )
            {
                state = 0;
                _header = HEADER_ON;
#if (SX1272_debug_mode > 1)
                Serial.println(F("## Header has been activated ##"));
                Serial.println();
#endif
            }
            else
            {
                state = 1;
            }
        }
    }
	return state;
}

/*
 Function: Sets the module in implicit header mode (header is not sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::setHeaderOFF()
{
    uint8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setHeaderOFF'"));
#endif

    if( _modem == FSK )
    { // header is not available in FSK mode
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Notice that FSK mode packets hasn't header ##"));
        Serial.println();
#endif
    }
    else
    {
        config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit

        // modified by C. Pham
        if (_board==SX1272Chip)
            config1 = config1 | B00000100;			// sets bit 2 from REG_MODEM_CONFIG1 = headerOFF
        else
            config1 = config1 | B00000001;                      // sets bit 0 from REG_MODEM_CONFIG1 = headerOFF

        writeRegister(REG_MODEM_CONFIG1,config1);		// Update config1

        config1 = readRegister(REG_MODEM_CONFIG1);

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( bitRead(config1, theHeaderBit) == HEADER_OFF )
        { // checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
            state = 0;
            _header = HEADER_OFF;

#if (SX1272_debug_mode > 1)
            Serial.println(F("## Header has been desactivated ##"));
            Serial.println();
#endif
        }
        else
        {
            state = 1;
#if (SX1272_debug_mode > 1)
            Serial.println(F("** Header hasn't been desactivated ##"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::getCRC()
{
    int8_t state = 2;
    byte value;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getCRC'"));
#endif

    if( _modem == LORA )
    { // LoRa mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
        value = readRegister(theRegister);
        if( bitRead(value, theCrcBit) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC is desactivated ##"));
            Serial.println();
#endif
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC is activated ##"));
            Serial.println();
#endif
            state = 0;
        }
    }
    else
    { // FSK mode

        // take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
        value = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(value, 4) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC is desactivated ##"));
            Serial.println();
#endif
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC is activated ##"));
            Serial.println();
#endif
            state = 0;
        }
    }
    if( state != 0 )
    {
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while getting configured CRC **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::setCRC_ON()
{
    uint8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setCRC_ON'"));
#endif

    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = readRegister(theRegister);	// Save config1 to modify only the CRC bit

        if (_board==SX1272Chip)
            config1 = config1 | B00000010;				// sets bit 1 from REG_MODEM_CONFIG1 = CRC_ON
        else
            config1 = config1 | B00000100;                               // sets bit 2 from REG_MODEM_CONFIG2 = CRC_ON

        writeRegister(theRegister,config1);

        state = 1;

        config1 = readRegister(theRegister);

        if( bitRead(config1, theCrcBit) == CRC_ON )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC has been activated ##"));
            Serial.println();
#endif
        }
    }
    else
    { // FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
        config1 = config1 | B00010000;				// set bit 4 and 3 from REG_MODEM_CONFIG1 = CRC_ON
        writeRegister(REG_PACKET_CONFIG1,config1);

        state = 1;

        config1 = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_ON )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
            state = 0;
            _CRC = CRC_ON;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC has been activated ##"));
            Serial.println();
#endif
        }
    }
    if( state != 0 )
    {
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while setting CRC ON **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::setCRC_OFF()
{
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setCRC_OFF'"));
#endif

    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = readRegister(theRegister);	// Save config1 to modify only the CRC bit
        if (_board==SX1272Chip)
            config1 = config1 & B11111101;				// clears bit 1 from config1 = CRC_OFF
        else
            config1 = config1 & B11111011;				// clears bit 2 from config1 = CRC_OFF

        writeRegister(theRegister,config1);

        config1 = readRegister(theRegister);
        if( (bitRead(config1, theCrcBit)) == CRC_OFF )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC has been desactivated ##"));
            Serial.println();
#endif
        }
    }
    else
    { // FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
        config1 = config1 & B11101111;				// clears bit 4 from config1 = CRC_OFF
        writeRegister(REG_PACKET_CONFIG1,config1);

        config1 = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_OFF )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
#if (SX1272_debug_mode > 1)
            Serial.println(F("## CRC has been desactivated ##"));
            Serial.println();
#endif
        }
    }
    if( state != 0 )
    {
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while setting CRC OFF **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Checks if SF is a valid value.
 Returns: Boolean that's 'true' if the SF value exists and
          it's 'false' if the SF value does not exist.
 Parameters:
   spr: spreading factor value to check.
*/
boolean	SX1272::isSF(uint8_t spr)
{
#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'isSF'"));
#endif

    // Checking available values for _spreadingFactor
    switch(spr)
    {
    case SF_6:
    case SF_7:
    case SF_8:
    case SF_9:
    case SF_10:
    case SF_11:
    case SF_12:
        return true;
        break;

    default:
        return false;
    }
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Finished 'isSF' ##"));
    Serial.println();
#endif
}

/*
 Function: Gets the SF within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::getSF()
{
    int8_t state = 2;
    byte config2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getSF'"));
#endif

    if( _modem == FSK )
    {
        state = -1;		// SF is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("** FSK mode hasn't spreading factor **"));
        Serial.println();
#endif
    }
    else
    {
        // take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
        config2 = (readRegister(REG_MODEM_CONFIG2)) >> 4;
        _spreadingFactor = config2;
        state = 1;

        if( (config2 == _spreadingFactor) && isSF(_spreadingFactor) )
        {
            state = 0;
#if (SX1272_debug_mode > 1)
            Serial.print(F("## Spreading factor is "));
            Serial.print(_spreadingFactor,HEX);
            Serial.println(F(" ##"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated SF in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   spr: spreading factor value to set in LoRa modem configuration.
*/
uint8_t	SX1272::setSF(uint8_t spr)
{
    byte st0;
    int8_t state = 2;
    byte config1=0;
    byte config2=0;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setSF'"));
#endif

    st0 = readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == FSK )
    {
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Notice that FSK hasn't Spreading Factor parameter, "));
        Serial.println(F("so you are configuring it in LoRa mode ##"));
#endif
        state = setLORA();				// Setting LoRa mode
    }
    else
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
        config2 = (readRegister(REG_MODEM_CONFIG2));	// Save config2 to modify SF value (bits 7-4)
        switch(spr)
        {
        case SF_6: 	config2 = config2 & B01101111;	// clears bits 7 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B01100000;	// sets bits 6 & 5 from REG_MODEM_CONFIG2
            setHeaderOFF();		// Mandatory headerOFF with SF = 6
            break;
        case SF_7: 	config2 = config2 & B01111111;	// clears bits 7 from REG_MODEM_CONFIG2
            config2 = config2 | B01110000;	// sets bits 6, 5 & 4
            break;
        case SF_8: 	config2 = config2 & B10001111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10000000;	// sets bit 7 from REG_MODEM_CONFIG2
            break;
        case SF_9: 	config2 = config2 & B10011111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10010000;	// sets bits 7 & 4 from REG_MODEM_CONFIG2
            break;
        case SF_10:	config2 = config2 & B10101111;	// clears bits 6 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10100000;	// sets bits 7 & 5 from REG_MODEM_CONFIG2
            break;
        case SF_11:	config2 = config2 & B10111111;	// clears bit 6 from REG_MODEM_CONFIG2
            config2 = config2 | B10110000;	// sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
            getBW();

            // modified by C. Pham
            if( _bandwidth == BW_125)
            { // LowDataRateOptimize (Mandatory with SF_11 if BW_125)
                if (_board==SX1272Chip) {
                    config1 = (readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the LowDataRateOptimize
                    config1 = config1 | B00000001;
                    writeRegister(REG_MODEM_CONFIG1,config1);
                }
                else {
                    byte config3=readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | B00001000;
                    writeRegister(REG_MODEM_CONFIG3,config3);
                }
            }
            break;
        case SF_12: config2 = config2 & B11001111;	// clears bits 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B11000000;	// sets bits 7 & 6 from REG_MODEM_CONFIG2
            if( _bandwidth == BW_125)
            { // LowDataRateOptimize (Mandatory with SF_12 if BW_125)
                // modified by C. Pham
                if (_board==SX1272Chip) {
                    config1 = (readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the LowDataRateOptimize
                    config1 = config1 | B00000001;
                    writeRegister(REG_MODEM_CONFIG1,config1);
                }
                else {
                    byte config3=readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | B00001000;
                    writeRegister(REG_MODEM_CONFIG3,config3);
                }
            }
            break;
        }

        // Check if it is neccesary to set special settings for SF=6
        if( spr == SF_6 )
        {
            // Mandatory headerOFF with SF = 6 (Implicit mode)
            setHeaderOFF();

            // Set the bit field DetectionOptimize of
            // register RegLoRaDetectOptimize to value "0b101".
            writeRegister(REG_DETECT_OPTIMIZE, 0x05);

            // Write 0x0C in the register RegDetectionThreshold.
            writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
        }
        else
        {
            // added by C. Pham
            setHeaderON();

            // LoRa detection Optimize: 0x03 --> SF7 to SF12
            writeRegister(REG_DETECT_OPTIMIZE, 0x03);

            // LoRa detection threshold: 0x0A --> SF7 to SF12
            writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
        }

        // added by C. Pham
        if (_board==SX1272Chip) {
            // comment by C. Pham
            // bit 9:8 of SymbTimeout are then 11
            // single_chan_pkt_fwd uses 00 and then 00001000
            // why?
            // sets bit 2-0 (AgcAutoOn and SymbTimout) for any SF value
            //config2 = config2 | B00000111;
            // modified by C. Pham
            config2 = config2 | B00000100;
            writeRegister(REG_MODEM_CONFIG1, config1);		// Update config1
        }
        else {
            // set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
            uint8_t config3 = (readRegister(REG_MODEM_CONFIG3));
            config3=config3 | B00000100;
            writeRegister(REG_MODEM_CONFIG3, config3);
        }

        // here we write the new SF
        writeRegister(REG_MODEM_CONFIG2, config2);		// Update config2

        delay(100);

        // added by C. Pham
        byte configAgc;
        uint8_t theLDRBit;

        if (_board==SX1272Chip) {
            config1 = (readRegister(REG_MODEM_CONFIG1));	// Save config1 to check update
            config2 = (readRegister(REG_MODEM_CONFIG2));	// Save config2 to check update
            // comment by C. Pham
            // (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
            // bitRead(config1, 0) ---> take out bits 1 from config1 (=LowDataRateOptimize)
            // config2 is only for the AgcAutoOn
            configAgc=config2;
            theLDRBit=0;
        }
        else {
            config1 = (readRegister(REG_MODEM_CONFIG3));	// Save config1 to check update
            config2 = (readRegister(REG_MODEM_CONFIG2));
            // LowDataRateOptimize is in REG_MODEM_CONFIG3
            // AgcAutoOn is in REG_MODEM_CONFIG3
            configAgc=config1;
            theLDRBit=3;
        }


        switch(spr)
        {
        case SF_6:	if(		((config2 >> 4) == spr)
                            && 	(bitRead(configAgc, 2) == 1)
                            && 	(_header == HEADER_OFF))
            {
                state = 0;
            }
            break;
        case SF_7:	if(		((config2 >> 4) == 0x07)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_8:	if(		((config2 >> 4) == 0x08)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_9:	if(		((config2 >> 4) == 0x09)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_10:	if(		((config2 >> 4) == 0x0A)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_11:	if(		((config2 >> 4) == 0x0B)
                            && (bitRead(configAgc, 2) == 1)
                            && (bitRead(config1, theLDRBit) == 1))
            {
                state = 0;
            }
            break;
        case SF_12:	if(		((config2 >> 4) == 0x0C)
                            && (bitRead(configAgc, 2) == 1)
                            && (bitRead(config1, theLDRBit) == 1))
            {
                state = 0;
            }
            break;
        default:	state = 1;
        }
    }

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);

    if( isSF(spr) )
    { // Checking available value for _spreadingFactor
        state = 0;
        _spreadingFactor = spr;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Spreading factor "));
        Serial.print(_spreadingFactor, DEC);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        if( state != 0 )
        {
#if (SX1272_debug_mode > 1)
            Serial.print(F("** There has been an error while setting the spreading factor **"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Checks if BW is a valid value.
 Returns: Boolean that's 'true' if the BW value exists and
          it's 'false' if the BW value does not exist.
 Parameters:
   band: bandwidth value to check.
*/
boolean	SX1272::isBW(uint16_t band)
{
#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'isBW'"));
#endif

    // Checking available values for _bandwidth
    // added by C. Pham
    if (_board==SX1272Chip) {
        switch(band)
        {
        case BW_125:
        case BW_250:
        case BW_500:
            return true;
            break;

        default:
            return false;
        }
    }
    else {
        switch(band)
        {
        case BW_7_8:
        case BW_10_4:
        case BW_15_6:
        case BW_20_8:
        case BW_31_25:
        case BW_41_7:
        case BW_62_5:
        case BW_125:
        case BW_250:
        case BW_500:
            return true;
            break;

        default:
            return false;
        }
    }

#if (SX1272_debug_mode > 1)
    Serial.println(F("## Finished 'isBW' ##"));
    Serial.println();
#endif
}

/*
 Function: Gets the BW within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::getBW()
{
    uint8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getBW'"));
#endif

    if( _modem == FSK )
    {
        state = -1;		// BW is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("** FSK mode hasn't bandwidth **"));
        Serial.println();
#endif
    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-6 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 6;
        }
        else {
            // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 4;
        }

        _bandwidth = config1;

        if( (config1 == _bandwidth) && isBW(_bandwidth) )
        {
            state = 0;
#if (SX1272_debug_mode > 1)
            Serial.print(F("## Bandwidth is "));
            Serial.print(_bandwidth,HEX);
            Serial.println(F(" ##"));
            Serial.println();
#endif
        }
        else
        {
            state = 1;
#if (SX1272_debug_mode > 1)
            Serial.print(F("** There has been an error while getting bandwidth **"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated BW in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   band: bandwith value to set in LoRa modem configuration.
*/
int8_t	SX1272::setBW(uint16_t band)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setBW'"));
#endif

    if(!isBW(band) )
    {
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.print(F("** Bandwidth "));
        Serial.print(band, HEX);
        Serial.println(F(" is not a correct value **"));
        Serial.println();
#endif
        return state;
    }

    st0 = readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == FSK )
    {
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Notice that FSK hasn't Bandwidth parameter, "));
        Serial.println(F("so you are configuring it in LoRa mode ##"));
#endif
        state = setLORA();
    }
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
    config1 = (readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the BW

    // added by C. Pham for SX1276
    if (_board==SX1272Chip) {
        switch(band)
        {
        case BW_125:  config1 = config1 & B00111111;	// clears bits 7 & 6 from REG_MODEM_CONFIG1
            getSF();
            if( _spreadingFactor == 11 )
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
                config1 = config1 | B00000001;
            }
            if( _spreadingFactor == 12 )
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
                config1 = config1 | B00000001;
            }
            break;
        case BW_250:  config1 = config1 & B01111111;	// clears bit 7 from REG_MODEM_CONFIG1
            config1 = config1 | B01000000;	// sets bit 6 from REG_MODEM_CONFIG1
            break;
        case BW_500:  config1 = config1 & B10111111;	//clears bit 6 from REG_MODEM_CONFIG1
            config1 = config1 | B10000000;	//sets bit 7 from REG_MODEM_CONFIG1
            break;
        }
    }
    else {
        // SX1276
        config1 = config1 & B00001111;	// clears bits 7 - 4 from REG_MODEM_CONFIG1
        switch(band)
        {
        case BW_125:
            // 0111
            config1 = config1 | B01110000;
            getSF();
            if( _spreadingFactor == 11 || _spreadingFactor == 12)
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_11 or SF_12)
                byte config3=readRegister(REG_MODEM_CONFIG3);
                config3 = config3 | B00001000;
                writeRegister(REG_MODEM_CONFIG3,config3);
            }
            break;
        case BW_250:
            // 1000
            config1 = config1 | B10000000;
            break;
        case BW_500:
            // 1001
            config1 = config1 | B10010000;
            break;
        }
    }
    // end

    writeRegister(REG_MODEM_CONFIG1,config1);		// Update config1

    delay(100);

    config1 = (readRegister(REG_MODEM_CONFIG1));

    // added by C. Pham
    if (_board==SX1272Chip) {
        // (config1 >> 6) ---> take out bits 7-6 from REG_MODEM_CONFIG1 (=_bandwidth)
        switch(band)
        {
        case BW_125: if( (config1 >> 6) == SX1272_BW_125 )
            {
                state = 0;
                if( _spreadingFactor == 11 )
                {
                    if( bitRead(config1, 0) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
                if( _spreadingFactor == 12 )
                {
                    if( bitRead(config1, 0) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
            }
            break;
        case BW_250: if( (config1 >> 6) == SX1272_BW_250 )
            {
                state = 0;
            }
            break;
        case BW_500: if( (config1 >> 6) == SX1272_BW_500 )
            {
                state = 0;
            }
            break;
        }
    }
    else {
        // (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
        switch(band)
        {
        case BW_125: if( (config1 >> 4) == BW_125 )
            {
                state = 0;

                byte config3 = (readRegister(REG_MODEM_CONFIG3));

                if( _spreadingFactor == 11 )
                {
                    if( bitRead(config3, 3) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
                if( _spreadingFactor == 12 )
                {
                    if( bitRead(config3, 3) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
            }
            break;
        case BW_250: if( (config1 >> 4) == BW_250 )
            {
                state = 0;
            }
            break;
        case BW_500: if( (config1 >> 4) == BW_500 )
            {
                state = 0;
            }
            break;
        }
    }

    if(state==0)
    {
        _bandwidth = band;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Bandwidth "));
        Serial.print(band, HEX);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Checks if CR is a valid value.
 Returns: Boolean that's 'true' if the CR value exists and
          it's 'false' if the CR value does not exist.
 Parameters:
   cod: coding rate value to check.
*/
boolean	SX1272::isCR(uint8_t cod)
{
#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'isCR'"));
#endif

    // Checking available values for _codingRate
    switch(cod)
    {
    case CR_5:
    case CR_6:
    case CR_7:
    case CR_8:
        return true;
        break;

    default:
        return false;
    }
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Finished 'isCR' ##"));
    Serial.println();
#endif
}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::getCR()
{
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getCR'"));
#endif

    if( _modem == FSK )
    {
        state = -1;		// CR is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("** FSK mode hasn't coding rate **"));
        Serial.println();
#endif
    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 3;
            config1 = config1 & B00000111;	// clears bits 7-3 ---> clears _bandwidth
        }
        else {
            // take out bits 7-1 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 1;
            config1 = config1 & B00000111;	// clears bits 7-3 ---> clears _bandwidth
        }

        _codingRate = config1;
        state = 1;

        if( (config1 == _codingRate) && isCR(_codingRate) )
        {
            state = 0;
#if (SX1272_debug_mode > 1)
            Serial.print(F("## Coding rate is "));
            Serial.print(_codingRate, HEX);
            Serial.println(F(" ##"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated CR in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: coding rate value to set in LoRa modem configuration.
*/
int8_t	SX1272::setCR(uint8_t cod)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setCR'"));
#endif

    st0 = readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Notice that FSK hasn't Coding Rate parameter, "));
        Serial.println(F("so you are configuring it in LoRa mode ##"));
#endif
        state = setLORA();
    }
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

    config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the CR

    // added by C. Pham
    if (_board==SX1272Chip) {
        switch(cod)
        {
        case CR_5: config1 = config1 & B11001111;	// clears bits 5 & 4 from REG_MODEM_CONFIG1
            config1 = config1 | B00001000;	// sets bit 3 from REG_MODEM_CONFIG1
            break;
        case CR_6: config1 = config1 & B11010111;	// clears bits 5 & 3 from REG_MODEM_CONFIG1
            config1 = config1 | B00010000;	// sets bit 4 from REG_MODEM_CONFIG1
            break;
        case CR_7: config1 = config1 & B11011111;	// clears bit 5 from REG_MODEM_CONFIG1
            config1 = config1 | B00011000;	// sets bits 4 & 3 from REG_MODEM_CONFIG1
            break;
        case CR_8: config1 = config1 & B11100111;	// clears bits 4 & 3 from REG_MODEM_CONFIG1
            config1 = config1 | B00100000;	// sets bit 5 from REG_MODEM_CONFIG1
            break;
        }
    }
    else {
        // SX1276
        config1 = config1 & B11110001;	// clears bits 3 - 1 from REG_MODEM_CONFIG1
        switch(cod)
        {
        case CR_5:
            config1 = config1 | B00000010;
            break;
        case CR_6:
            config1 = config1 | B00000100;
            break;
        case CR_7:
            config1 = config1 | B00000110;
            break;
        case CR_8:
            config1 = config1 | B00001000;
            break;
        }
    }
    writeRegister(REG_MODEM_CONFIG1, config1);		// Update config1

    delay(100);

    config1 = readRegister(REG_MODEM_CONFIG1);

    // added by C. Pham
    uint8_t nshift=3;

    // only 1 right shift for SX1276
    if (_board==SX1276Chip)
        nshift=1;

    // ((config1 >> 3) & B0000111) ---> take out bits 5-3 from REG_MODEM_CONFIG1 (=_codingRate)
    switch(cod)
    {
    case CR_5: if( ((config1 >> nshift) & B0000111) == 0x01 )
        {
            state = 0;
        }
        break;
    case CR_6: if( ((config1 >> nshift) & B0000111) == 0x02 )
        {
            state = 0;
        }
        break;
    case CR_7: if( ((config1 >> nshift) & B0000111) == 0x03 )
        {
            state = 0;
        }
        break;
    case CR_8: if( ((config1 >> nshift) & B0000111) == 0x04 )
        {
            state = 0;
        }
        break;
    }


    if( isCR(cod) )
    {
        _codingRate = cod;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Coding Rate "));
        Serial.print(cod, HEX);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while configuring Coding Rate parameter **"));
        Serial.println();
#endif
    }
    writeRegister(REG_OP_MODE,st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Checks if channel is a valid value.
 Returns: Boolean that's 'true' if the CR value exists and
          it's 'false' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
boolean	SX1272::isChannel(uint32_t ch)
{
#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'isChannel'"));
#endif

    // Checking available values for _channel
    switch(ch)
    {
        //added by C. Pham
    case CH_04_868:
    case CH_05_868:
    case CH_06_868:
    case CH_07_868:
    case CH_08_868:
    case CH_09_868:
        //end
    case CH_10_868:
    case CH_11_868:
    case CH_12_868:
    case CH_13_868:
    case CH_14_868:
    case CH_15_868:
    case CH_16_868:
    case CH_17_868:
        //added by C. Pham
    case CH_18_868:
        //end
    case CH_00_900:
    case CH_01_900:
    case CH_02_900:
    case CH_03_900:
    case CH_04_900:
    case CH_05_900:
    case CH_06_900:
    case CH_07_900:
    case CH_08_900:
    case CH_09_900:
    case CH_10_900:
    case CH_11_900:
        //added by C. Pham
    case CH_12_900:
    case CH_00_433:
    case CH_01_433:
    case CH_02_433:
    case CH_03_433:
        //end
        return true;
        break;

    default:
        return false;
    }
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Finished 'isChannel' ##"));
    Serial.println();
#endif
}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getChannel()
{
    uint8_t state = 2;
    uint32_t ch;
    uint8_t freq3;
    uint8_t freq2;
    uint8_t freq1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getChannel'"));
#endif

    freq3 = readRegister(REG_FRF_MSB);	// frequency channel MSB
    freq2 = readRegister(REG_FRF_MID);	// frequency channel MID
    freq1 = readRegister(REG_FRF_LSB);	// frequency channel LSB
    ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
    _channel = ch;						// frequency channel

    if( (_channel == ch) && isChannel(_channel) )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Frequency channel is "));
        Serial.print(_channel, HEX);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }
    return state;
}

/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t SX1272::setChannel(uint32_t ch)
{
    byte st0;
    int8_t state = 2;
    unsigned int freq3;
    unsigned int freq2;
    uint8_t freq1;
    uint32_t freq;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setChannel'"));
#endif

    // added by C. Pham
    _starttime=millis();

    st0 = readRegister(REG_OP_MODE);	// Save the previous status
    if( _modem == LORA )
    {
        // LoRa Stdby mode in order to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    {
        // FSK Stdby mode in order to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    freq3 = ((ch >> 16) & 0x0FF);		// frequency channel MSB
    freq2 = ((ch >> 8) & 0x0FF);		// frequency channel MIB
    freq1 = (ch & 0xFF);				// frequency channel LSB

    writeRegister(REG_FRF_MSB, freq3);
    writeRegister(REG_FRF_MID, freq2);
    writeRegister(REG_FRF_LSB, freq1);

    // added by C. Pham
    _stoptime=millis();

    delay(100);

    // storing MSB in freq channel value
    freq3 = (readRegister(REG_FRF_MSB));
    freq = (freq3 << 8) & 0xFFFFFF;

    // storing MID in freq channel value
    freq2 = (readRegister(REG_FRF_MID));
    freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

    // storing LSB in freq channel value
    freq = freq + ((readRegister(REG_FRF_LSB)) & 0xFFFFFF);

    if( freq == ch )
    {
        state = 0;
        _channel = ch;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Frequency channel "));
        Serial.print(ch, HEX);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }

    // commented by C. Pham to avoid adding new channel each time
    // besides, the test above is sufficient
    /*
    if(!isChannel(ch) )
    {
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.print(F("** Frequency channel "));
        Serial.print(ch, HEX);
        Serial.println(F("is not a correct value **"));
        Serial.println();
#endif
    }
    */

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPower()
{
    uint8_t state = 2;
    byte value = 0x00;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getPower'"));
#endif

    value = readRegister(REG_PA_CONFIG);
    state = 1;

    // modified by C. Pham
    // get only the OutputPower
    _power = value & B00001111;

    //if( (value > -1) & (value < 16) )
    if( _power < 16 )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Output power is "));
        Serial.print(_power, HEX);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }

    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPower(char p)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

    byte RegPaDacReg=(_board==SX1272Chip)?0x5A:0x4D;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPower'"));
#endif

    st0 = readRegister(REG_OP_MODE);	  // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    switch (p)
    {
    // L = Low. On SX1272/76: PA0 on RFO setting
    // H = High. On SX1272/76: PA0 on RFO setting
    // M = MAX. On SX1272/76: PA0 on RFO setting

    // x = extreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting
    // X = eXtreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting + 20dBm settings

    // added by C. Pham
    //
    case 'x':
    case 'X':
    case 'M':  value = 0x0F;
        // SX1272/76: 14dBm
        break;

    // modified by C. Pham, set to 0x03 instead of 0x00
    case 'L':  value = 0x03;
        // SX1272/76: 2dBm
        break;

    case 'H':  value = 0x07;
        // SX1272/76: 6dBm
        break;

    default:   state = -1;
        break;
    }

    // 100mA
    setMaxCurrent(0x0B);

    if (p=='x') {
        // we set only the PA_BOOST pin
        // limit to 14dBm
        value = 0x0C;
        value = value | B10000000;
        // set RegOcp for OcpOn and OcpTrim
        // 130mA
        setMaxCurrent(0x10);
    }
    
    if (p=='X') {
        // normally value = 0x0F;
        // we set the PA_BOOST pin
        value = value | B10000000;
        // and then set the high output power config with register REG_PA_DAC
        writeRegister(RegPaDacReg, 0x87);
        // set RegOcp for OcpOn and OcpTrim
        // 150mA
        setMaxCurrent(0x12);
    }
    else {
        // disable high power output in all other cases
        writeRegister(RegPaDacReg, 0x84);
    }

    // added by C. Pham
    if (_board==SX1272Chip) {
        // Pout = -1 + _power[3:0] on RFO
        // Pout = 2 + _power[3:0] on PA_BOOST
        // so: L=2dBm; H=6dBm, M=14dBm, x=14dBm (PA), X=20dBm(PA+PADAC)
        writeRegister(REG_PA_CONFIG, value);	// Setting output power value
    }
    else {
        // for the SX1276

        // set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
        value = value | B01110000;

        // then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +14dBm)
        // so L=3dBm; H=7dBm; M=15dBm (but should be limited to 14dBm by RFO pin)

        // and Pout = 17-(15-_power[3:0]) if  PaSelect=1 (PA_BOOST pin for +14dBm)
        // so x= 14dBm (PA);
        // when p=='X' for 20dBm, value is 0x0F and RegPaDacReg=0x87 so 20dBm is enabled

        writeRegister(REG_PA_CONFIG, value);
    }

    _power=value;

    value = readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Output power has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPowerNum(uint8_t pow)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPower'"));
#endif

    st0 = readRegister(REG_OP_MODE);	  // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    if ( (pow >= 0) && (pow < 15) )
    {
        _power = pow;
    }
    else
    {
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Power value is not valid ##"));
        Serial.println();
#endif
    }

    // added by C. Pham
    if (_board==SX1276Chip) {
        value=readRegister(REG_PA_CONFIG);
        // clear OutputPower, but keep current value of PaSelect and MaxPower
        value=value & B11110000;
        value=value + _power;
        _power=value;
    }
    writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
    value = readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Output power has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}


/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPreambleLength()
{
    int8_t state = 2;
    uint8_t p_length;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getPreambleLength'"));
#endif

    state = 1;
    if( _modem == LORA )
    { // LORA mode
        p_length = readRegister(REG_PREAMBLE_MSB_LORA);
        // Saving MSB preamble length in LoRa mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = readRegister(REG_PREAMBLE_LSB_LORA);
        // Saving LSB preamble length in LoRa mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Preamble length configured is "));
        Serial.print(_preamblelength, HEX);
        Serial.print(F(" ##"));
        Serial.println();
#endif
    }
    else
    { // FSK mode
        p_length = readRegister(REG_PREAMBLE_MSB_FSK);
        // Saving MSB preamble length in FSK mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = readRegister(REG_PREAMBLE_LSB_FSK);
        // Saving LSB preamble length in FSK mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Preamble length configured is "));
        Serial.print(_preamblelength, HEX);
        Serial.print(F(" ##"));
        Serial.println();
#endif
    }
    state = 0;
    return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t SX1272::setPreambleLength(uint16_t l)
{
    byte st0;
    uint8_t p_length;
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPreambleLength'"));
#endif

    st0 = readRegister(REG_OP_MODE);	// Save the previous status
    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in LoRa mode
        writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in LoRa mode
        writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in FSK mode
        writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in FSK mode
        writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
    }

    state = 0;
#if (SX1272_debug_mode > 1)
    Serial.print(F("## Preamble length "));
    Serial.print(l, HEX);
    Serial.println(F(" has been successfully set ##"));
    Serial.println();
#endif

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPayloadLength()
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getPayloadLength'"));
#endif

    if( _modem == LORA )
    { // LORA mode
        // Saving payload length in LoRa mode
        _payloadlength = readRegister(REG_PAYLOAD_LENGTH_LORA);
        state = 1;
    }
    else
    { // FSK mode
        // Saving payload length in FSK mode
        _payloadlength = readRegister(REG_PAYLOAD_LENGTH_FSK);
        state = 1;
    }

#if (SX1272_debug_mode > 1)
    Serial.print(F("## Payload length configured is "));
    Serial.print(_payloadlength, HEX);
    Serial.println(F(" ##"));
    Serial.println();
#endif

    state = 0;
    return state;
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1272::setPacketLength()
{
    uint16_t length;

    // added by C. Pham
    // if gateway is in rawFormat mode for packet reception, it will also send in rawFormat
    // unless we switch it back to normal format just for transmission, e.g. for downlink transmission
    if (_rawFormat)
        length = _payloadlength;
    else
        length = _payloadlength + OFFSET_PAYLOADLENGTH;

    return setPacketLength(length);
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t SX1272::setPacketLength(uint8_t l)
{
    byte st0;
    byte value = 0x00;
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPacketLength'"));
#endif

    st0 = readRegister(REG_OP_MODE);	// Save the previous status
    packet_sent.length = l;

    if( _modem == LORA )
    { // LORA mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
        writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
        // Storing payload length in LoRa mode
        value = readRegister(REG_PAYLOAD_LENGTH_LORA);
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
        writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
        // Storing payload length in FSK mode
        value = readRegister(REG_PAYLOAD_LENGTH_FSK);
    }

    if( packet_sent.length == value )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Packet length "));
        Serial.print(packet_sent.length, DEC);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    // comment by C. Pham
    // this delay is included in the send delay overhead
    // TODO: do we really need this delay?
    delay(250);
    return state;
}

/*
 Function: Gets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getNodeAddress()
{
    byte st0 = 0;
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getNodeAddress'"));
#endif

    if( _modem == LORA )
    { // LoRa mode
        st0 = readRegister(REG_OP_MODE);	// Save the previous status
        // Allowing access to FSK registers while in LoRa standby mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }

    // Saving node address
    _nodeAddress = readRegister(REG_NODE_ADRS);
    state = 1;

    if( _modem == LORA )
    {
        writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
    }

    state = 0;
#if (SX1272_debug_mode > 1)
    Serial.print(F("## Node address configured is "));
    Serial.print(_nodeAddress);
    Serial.println(F(" ##"));
    Serial.println();
#endif
    return state;
}

/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t SX1272::setNodeAddress(uint8_t addr)
{
    byte st0;
    byte value;
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setNodeAddress'"));
#endif

    if( addr > 255 )
    {
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Node address must be less than 255 **"));
        Serial.println();
#endif
    }
    else
    {
        // Saving node address
        _nodeAddress = addr;
        st0 = readRegister(REG_OP_MODE);	  // Save the previous status

        if( _modem == LORA )
        { // Allowing access to FSK registers while in LoRa standby mode
            writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
        }
        else
        { //Set FSK Standby mode to write in registers
            writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
        }

        // Storing node and broadcast address
        writeRegister(REG_NODE_ADRS, addr);
        writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

        value = readRegister(REG_NODE_ADRS);
        writeRegister(REG_OP_MODE, st0);		// Getting back to previous status

        if( value == _nodeAddress )
        {
            state = 0;
#if (SX1272_debug_mode > 1)
            Serial.print(F("## Node address "));
            Serial.print(addr);
            Serial.println(F(" has been successfully set ##"));
            Serial.println();
#endif
        }
        else
        {
            state = 1;
#if (SX1272_debug_mode > 1)
            Serial.println(F("** There has been an error while setting address ##"));
            Serial.println();
#endif
        }
    }
    return state;
}

/*
 Function: Gets the SNR value in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1272::getSNR()
{	// getSNR exists only in LoRa mode
    int8_t state = 2;
    byte value;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getSNR'"));
#endif

    if( _modem == LORA )
    { // LoRa mode
        state = 1;
        value = readRegister(REG_PKT_SNR_VALUE);
        _rawSNR = value;

        if( value & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            value = ( ( ~value + 1 ) & 0xFF ) >> 2;
            _SNR = -value;
        }
        else
        {
            // Divide by 4
            _SNR = ( value & 0xFF ) >> 2;
        }
        state = 0;
#if (SX1272_debug_mode > 0)
        Serial.print(F("## SNR value is "));
        Serial.print(_SNR, DEC);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    else
    { // forbidden command if FSK mode
        state = -1;
#if (SX1272_debug_mode > 0)
        Serial.println(F("** SNR does not exist in FSK mode **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Gets the current value of RSSI.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getRSSI()
{
    uint8_t state = 2;
    int rssi_mean = 0;
    int total = 5;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getRSSI'"));
#endif

    if( _modem == LORA )
    {
        /// LoRa mode
        // get mean value of RSSI
        for(int i = 0; i < total; i++)
        {
            // modified by C. Pham
            // with SX1276 we have to add 18 to OFFSET_RSSI to obtain -157
            _RSSI = -(OFFSET_RSSI+(_board==SX1276Chip?18:0)) + readRegister(REG_RSSI_VALUE_LORA);
            rssi_mean += _RSSI;
        }

        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;
#if (SX1272_debug_mode > 0)
        Serial.print(F("## RSSI value is "));
        Serial.print(_RSSI, DEC);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    else
    {
        /// FSK mode
        // get mean value of RSSI
        for(int i = 0; i < total; i++)
        {
            _RSSI = (readRegister(REG_RSSI_VALUE_FSK) >> 1);
            rssi_mean += _RSSI;
        }
        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;

#if (SX1272_debug_mode > 0)
        Serial.print(F("## RSSI value is "));
        Serial.print(_RSSI);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Gets the RSSI of the last packet received in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int16_t SX1272::getRSSIpacket()
{	// RSSIpacket only exists in LoRa
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getRSSIpacket'"));
#endif

    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = getSNR();
        if( state == 0 )
        {
            // added by C. Pham
            _RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);

            if( _SNR < 0 )
            {
                // commented by C. Pham
                //_RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;

                // added by C. Pham, using Semtech SX1272 rev3 March 2015
                // for SX1272 we use -139, for SX1276, we use -157
                // then for SX1276 when using low-frequency (i.e. 433MHz) then we use -164
                //_RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?18:0)+(_channel<CH_04_868?7:0)) + (double)_RSSIpacket + (double)_rawSNR*0.25;
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?18:0)+(_channel<CH_04_868?7:0)) + (double)_RSSIpacket + (double)_SNR*0.25;
                state = 0;
            }
            else
            {
                // commented by C. Pham
                //_RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?18:0)+(_channel<CH_04_868?7:0)) + (double)_RSSIpacket*16.0/15.0;
                //end
                state = 0;
            }
#if (SX1272_debug_mode > 0)
            Serial.print(F("## RSSI packet value is "));
            Serial.print(_RSSIpacket, DEC);
            Serial.println(F(" ##"));
            Serial.println();
#endif
        }
    }
    else
    { // RSSI packet doesn't exist in FSK mode
        state = -1;
#if (SX1272_debug_mode > 0)
        Serial.println(F("** RSSI packet does not exist in FSK mode **"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: It sets the maximum number of retries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 -->
*/
uint8_t SX1272::setRetries(uint8_t ret)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setRetries'"));
#endif

    state = 1;
    if( ret > MAX_RETRIES )
    {
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.print(F("** Retries value can't be greater than "));
        Serial.print(MAX_RETRIES, DEC);
        Serial.println(F(" **"));
        Serial.println();
#endif
    }
    else
    {
        _maxRetries = ret;
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Maximum retries value = "));
        Serial.print(_maxRetries, DEC);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
uint8_t SX1272::getMaxCurrent()
{
    int8_t state = 2;
    byte value;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getMaxCurrent'"));
#endif

    state = 1;
    _maxCurrent = readRegister(REG_OCP);

    // extract only the OcpTrim value from the OCP register
    _maxCurrent &= B00011111;

    if( _maxCurrent <= 15 )
    {
        value = (45 + (5 * _maxCurrent));
    }
    else if( _maxCurrent <= 27 )
    {
        value = (-30 + (10 * _maxCurrent));
    }
    else
    {
        value = 240;
    }

    _maxCurrent = value;
#if (SX1272_debug_mode > 1)
    Serial.print(F("## Maximum current supply configured is "));
    Serial.print(value, DEC);
    Serial.println(F(" mA ##"));
    Serial.println();
#endif
    state = 0;
    return state;
}

/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
int8_t SX1272::setMaxCurrent(uint8_t rate)
{
    int8_t state = 2;
    byte st0;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setMaxCurrent'"));
#endif

    // Maximum rate value = 0x1B, because maximum current supply = 240 mA
    if (rate > 0x1B)
    {
        state = -1;
#if (SX1272_debug_mode > 1)
        Serial.print(F("** Maximum current supply is 240 mA, "));
        Serial.println(F("so maximum parameter value must be 27 (DEC) or 0x1B (HEX) **"));
        Serial.println();
#endif
    }
    else
    {
        // Enable Over Current Protection
        rate |= B00100000;

        state = 1;
        st0 = readRegister(REG_OP_MODE);	// Save the previous status
        if( _modem == LORA )
        { // LoRa mode
            writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Set LoRa Standby mode to write in registers
        }
        else
        { // FSK mode
            writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
        }
        writeRegister(REG_OCP, rate);		// Modifying maximum current supply
        writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
        state = 0;
    }
    return state;
}

/*
 Function: Gets the content of different registers.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getRegs()
{
    int8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getRegs'"));
#endif

    state_f = 1;
    state = getMode();			// Stores the BW, CR and SF.
    if( state == 0 )
    {
        state = getPower();		// Stores the power.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting mode **"));
#endif
    }
    if( state == 0 )
    {
        state = getChannel();	// Stores the channel.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting power **"));
#endif
    }
    if( state == 0 )
    {
        state = getCRC();		// Stores the CRC configuration.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting channel **"));
#endif
    }
    if( state == 0 )
    {
        state = getHeader();	// Stores the header configuration.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting CRC **"));
#endif
    }
    if( state == 0 )
    {
        state = getPreambleLength();	// Stores the preamble length.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting header **"));
#endif
    }
    if( state == 0 )
    {
        state = getPayloadLength();		// Stores the payload length.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting preamble length **"));
#endif
    }
    if( state == 0 )
    {
        state = getNodeAddress();		// Stores the node address.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting payload length **"));
#endif
    }
    if( state == 0 )
    {
        state = getMaxCurrent();		// Stores the maximum current supply.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting node address **"));
#endif
    }
    if( state == 0 )
    {
        state_f = getTemp();		// Stores the module temperature.
    }
    else
    {
        state_f = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting maximum current supply **"));
#endif
    }
    if( state_f != 0 )
    {
#if (SX1272_debug_mode > 1)
        Serial.println(F("** Error getting temperature **"));
        Serial.println();
#endif
    }
    return state_f;
}

/*
 Function: It truncs the payload length if it is greater than 0xFF.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::truncPayload(uint16_t length16)
{
    uint8_t state = 2;

    state = 1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'truncPayload'"));
#endif

    if( length16 > MAX_PAYLOAD )
    {
        _payloadlength = MAX_PAYLOAD;
    }
    else
    {
        _payloadlength = (length16 & 0xFF);
    }
    state = 0;

    return state;
}

/*
 Function: It sets an ACK in FIFO in order to send it.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setACK()
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setACK'"));
#endif

    // added by C. Pham
    // check for enough remaining ToA
    // when operating under duty-cycle mode
    if (_limitToA) {
        if (getRemainingToA() - getToA(ACK_LENGTH) < 0) {
            Serial.print(F("## not enough ToA for ACK at"));
            Serial.println(millis());
            return SX1272_ERROR_TOA;
        }
    }

    // delay(1000);

    clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    // Setting ACK length in order to send it
    state = setPacketLength(ACK_LENGTH);
    if( state == 0 )
    {
        // Setting ACK
        ACK.dst = packet_received.src; // ACK destination is packet source
        ACK.type = PKT_TYPE_ACK;
        ACK.src = packet_received.dst; // ACK source is packet destination
        ACK.packnum = packet_received.packnum; // packet number that has been correctly received
        ACK.length = 2;
        ACK.data[0] = _reception;	// CRC of the received packet
        // added by C. Pham
        // store the SNR
        ACK.data[1]= readRegister(REG_PKT_SNR_VALUE);

        // Setting address pointer in FIFO data buffer
        writeRegister(REG_FIFO_ADDR_PTR, 0x80);

        state = 1;

        // Writing ACK to send in FIFO
        writeRegister(REG_FIFO, ACK.dst); 		// Writing the destination in FIFO
        writeRegister(REG_FIFO, ACK.type);
        writeRegister(REG_FIFO, ACK.src);		// Writing the source in FIFO
        writeRegister(REG_FIFO, ACK.packnum);	// Writing the packet number in FIFO
        writeRegister(REG_FIFO, ACK.length); 	// Writing the packet length in FIFO
        writeRegister(REG_FIFO, ACK.data[0]);	// Writing the ACK in FIFO
        writeRegister(REG_FIFO, ACK.data[1]);	// Writing the ACK in FIFO

        //#if (SX1272_debug_mode > 0)
        Serial.println(F("## ACK set and written in FIFO ##"));
        // Print the complete ACK if debug_mode
        Serial.println(F("## ACK to send:"));
        Serial.print(F("Destination: "));
        Serial.println(ACK.dst);			 	// Printing destination
        Serial.print(F("Source: "));
        Serial.println(ACK.src);			 	// Printing source
        Serial.print(F("ACK number: "));
        Serial.println(ACK.packnum);			// Printing ACK number
        Serial.print(F("ACK length: "));
        Serial.println(ACK.length);				// Printing ACK length
        Serial.print(F("ACK payload: "));
        Serial.println(ACK.data[0]);			// Printing ACK payload
        Serial.print(F("ACK SNR last rcv pkt: "));
        Serial.println(_SNR);
        Serial.println(F("##"));
        Serial.println();
        //#endif

        state = 0;
        _reception = CORRECT_PACKET;		// Updating value to next packet

        // comment by C. Pham
        // TODO: do we really need this delay?
        delay(500);
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receive()
{
    uint8_t state = 1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'receive'"));
#endif

    // Initializing packet_received struct
    memset( &packet_received, 0x00, sizeof(packet_received) );
    packet_received.data=packet_data;

    // Setting Testmode
    // commented by C. Pham
    //writeRegister(0x31,0x43);

    // Set LowPnTxPllOff
    // modified by C. Pham from 0x09 to 0x08
    writeRegister(REG_PA_RAMP, 0x08);

    //writeRegister(REG_LNA, 0x23);			// Important in reception
    // modified by C. Pham
    writeRegister(REG_LNA, LNA_MAX_GAIN);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    // change RegSymbTimeoutLsb
    // comment by C. Pham
    // single_chan_pkt_fwd uses 00 00001000
    // why here we have 11 11111111
    // change RegSymbTimeoutLsb
    //writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);

    // modified by C. Pham
    if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11 || _spreadingFactor == SF_12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    //end

    writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer
    //clearFlags();						// Initializing flags
    //state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = setPacketLength(MAX_LENGTH);	// With MAX_LENGTH gets all packets with length < MAX_LENGTH
        writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - Rx
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Receiving LoRa mode activated with success ##"));
        Serial.println();
#endif
    }
    else
    { // FSK mode
        state = setPacketLength();
        writeRegister(REG_OP_MODE, FSK_RX_MODE);  // FSK mode - Rx
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Receiving FSK mode activated with success ##"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketMAXTimeout()
{
    return receivePacketTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketTimeout()
{
    setTimeout();
    return receivePacketTimeout(_sendTime);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 5  --> The packet header (packet type) has not been recognized
   state = 4  --> The packet has been incorrectly received (CRC for instance)
   state = 3  --> No packet has been received during the receive windows 
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
#ifdef W_REQUESTED_ACK

// added by C. Pham
// receiver always use receivePacketTimeout()
// sender should either use sendPacketTimeout() or sendPacketTimeoutACK()

uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'receivePacketTimeout'"));
#endif

    state = receive();
    if( state == 0 )
    {
        if( availableData(wait) )
        {
            state = getPacket();
        }
        else
        {
            state = 1;
            state_f = 3;  // There is no packet received
        }
    }
    else
    {
        state = 1;
        state_f = 1; // There has been an error with the 'receive' function
    }

    if( (state == 0) || (state == 3) || (state == 5) )
    {
        if( _reception == INCORRECT_PACKET )
        {
            state_f = 4;  // The packet has been incorrectly received
        }
        else if ( _reception == INCORRECT_PACKET_TYPE )
        {
            state_f = 5;  // The packet type has not been recognized
        }
        else        
        {
            state_f = 0;  // The packet has been correctly received
            // added by C. Pham
            // we get the SNR and RSSI of the received packet for future usage
            getSNR();
            getRSSIpacket();
        }

        // need to send an ACK
        if ( state == 5 && state_f == 0) {

            state = setACK();

            if( state == 0 )
            {
                state = sendWithTimeout();
                if( state == 0 )
                {
                    state_f = 0;
#if (SX1272_debug_mode > 1)
                    Serial.println(F("This last packet was an ACK, so ..."));
                    Serial.println(F("ACK successfully sent"));
                    Serial.println();
#endif
                }
                else
                {
                    state_f = 1; // There has been an error with the 'sendWithTimeout' function
                }
            }
            else
            {
                state_f = 1; // There has been an error with the 'setACK' function
            }
        }
    }
    else
    {
        // we need to conserve state_f=3 to indicate that no packet has been received after timeout
        //state_f = 1;
    }
    return state_f;
}
#else

uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'receivePacketTimeout'"));
#endif

    state = receive();
    if( state == 0 )
    {
        if( availableData(wait) )
        {
            // If packet received, getPacket
            state_f = getPacket();
        }
        else
        {
            state_f = 1;
        }
    }
    else
    {
        state_f = state;
    }
    return state_f;
}
#endif

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketMAXTimeoutACK()
{
    return receivePacketTimeoutACK(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketTimeoutACK()
{
    setTimeout();
    return receivePacketTimeoutACK(_sendTime);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 4  --> The command has been executed but the packet received is incorrect
   state = 3  --> The command has been executed but there is no packet received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketTimeoutACK(uint16_t wait)
{
    // commented by C. Pham because not used
    /*
    uint8_t state = 2;
    uint8_t state_f = 2;


#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'receivePacketTimeoutACK'"));
#endif

    state = receive();
    if( state == 0 )
    {
        if( availableData(wait) )
        {
            state = getPacket();
        }
        else
        {
            state = 1;
            state_f = 3;  // There is no packet received
        }
    }
    else
    {
        state = 1;
        state_f = 1; // There has been an error with the 'receive' function
    }
    if( (state == 0) || (state == 3) )
    {
        if( _reception == INCORRECT_PACKET )
        {
            state_f = 4;  // The packet has been incorrectly received
        }
        else
        {
            state_f = 1;  // The packet has been correctly received
        }
        state = setACK();
        if( state == 0 )
        {
            state = sendWithTimeout();
            if( state == 0 )
            {
                state_f = 0;
#if (SX1272_debug_mode > 1)
                Serial.println(F("This last packet was an ACK, so ..."));
                Serial.println(F("ACK successfully sent"));
                Serial.println();
#endif
            }
            else
            {
                state_f = 1; // There has been an error with the 'sendWithTimeout' function
            }
        }
        else
        {
            state_f = 1; // There has been an error with the 'setACK' function
        }
    }
    else
    {
        state_f = 1;
    }
    return state_f;
    */
    return 0;
}

/*
 Function: Configures the module to receive all the information on air, before MAX_TIMEOUT expires.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::receiveAll()
{
    return receiveAll(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive all the information on air.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receiveAll(uint16_t wait)
{
    uint8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'receiveAll'"));
#endif

    if( _modem == FSK )
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);		// Setting standby FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1);
        config1 = config1 & B11111001;			// clears bits 2-1 from REG_PACKET_CONFIG1
        writeRegister(REG_PACKET_CONFIG1, config1);		// AddressFiltering = None
    }
#if (SX1272_debug_mode > 1)
    Serial.println(F("## Address filtering desactivated ##"));
    Serial.println();
#endif
    state = receive();	// Setting Rx mode
    if( state == 0 )
    {
        state = getPacket(wait);	// Getting all packets received in wait
    }
    return state;
}

/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'true' if the packet is for the module and
          it's 'false' if the packet is not for the module.
*/
boolean	SX1272::availableData()
{
    return availableData(MAX_TIMEOUT);
}

/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'true' if the packet is for the module and
          it's 'false' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
boolean	SX1272::availableData(uint16_t wait)
{
    byte value;
    byte header = 0;
    boolean forme = false;
    boolean	_hreceived = false;
    //unsigned long previous;
    unsigned long exitTime;


#if (SX1272_debug_mode > 0)
    Serial.println();
    Serial.println(F("Starting 'availableData'"));
#endif

    exitTime=millis()+(unsigned long)wait;

    //previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        value = readRegister(REG_IRQ_FLAGS);
        // Wait to ValidHeader interrupt
        //while( (bitRead(value, 4) == 0) && (millis() - previous < (unsigned long)wait) )
        while( (bitRead(value, 4) == 0) && (millis() < exitTime) )
        {
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP32
            yield();
#endif        
            value = readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        } // end while (millis)

        if( bitRead(value, 4) == 1 )
        { // header received
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Valid Header received in LoRa mode ##"));
#endif
            _hreceived = true;

#ifdef W_NET_KEY
            // actually, need to wait until 3 bytes have been received
            //while( (header < 3) && (millis() - previous < (unsigned long)wait) )
            while( (header < 3) && (millis() < exitTime) )
#else
            //while( (header == 0) && (millis() - previous < (unsigned long)wait) )
            while( (header == 0) && (millis() < exitTime) )
#endif
            { // Waiting to read first payload bytes from packet
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP32
            	yield();
#endif            
                header = readRegister(REG_FIFO_RX_BYTE_ADDR);
                // Condition to avoid an overflow (DO NOT REMOVE)
                //if( millis() < previous )
                //{
                //    previous = millis();
                //}
            }

            if( header != 0 )
            { // Reading first byte of the received packet
#ifdef W_NET_KEY
                // added by C. Pham
                // if we actually wait for an ACK, there is no net key before ACK data
                if (_requestACK==0) {
                    _the_net_key_0 = readRegister(REG_FIFO);
                    _the_net_key_1 = readRegister(REG_FIFO);
                }
#endif
                _destination = readRegister(REG_FIFO);
            }
        }
        else
        {
            forme = false;
            _hreceived = false;
#if (SX1272_debug_mode > 0)
            Serial.println(F("** The timeout has expired **"));
            Serial.println();
#endif
        }
    }
    else
    { // FSK mode
        value = readRegister(REG_IRQ_FLAGS2);
        // Wait to Payload Ready interrupt
        //while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
        while( (bitRead(value, 2) == 0) && (millis() < exitTime) )
        {
            value = readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        }// end while (millis)

        if( bitRead(value, 2) == 1 )	// something received
        {
            _hreceived = true;
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Valid Preamble detected in FSK mode ##"));
#endif
            // Reading first byte of the received packet
            _destination = readRegister(REG_FIFO);
        }
        else
        {
            forme = false;
            _hreceived = false;
#if (SX1272_debug_mode > 0)
            Serial.println(F("** The timeout has expired **"));
            Serial.println();
#endif
        }
    }
    // We use _hreceived because we need to ensure that _destination value is correctly
    // updated and is not the _destination value from the previously packet
    if( _hreceived == true )
    { // Checking destination
#if (SX1272_debug_mode > 0)
        Serial.println(F("## Checking destination ##"));
#endif

        // added by C. Pham
#ifdef W_NET_KEY
        forme=true;

        // if we wait for an ACK, then we do not check for net key
        if (_requestACK==0)
            if (_the_net_key_0!=_my_netkey[0] || _the_net_key_1!=_my_netkey[1]) {
                //#if (SX1272_debug_mode > 0)
                Serial.println(F("## Wrong net key ##"));
                //#endif
                forme=false;
            }
            else
            {
                //#if (SX1272_debug_mode > 0)
                Serial.println(F("## Good net key ##"));
                //#endif
            }


        if( forme && ((_destination == _nodeAddress) || (_destination == BROADCAST_0)) )
#else
        // modified by C. Pham
        // if _rawFormat, accept all
        if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) || _rawFormat)
#endif
        { // LoRa or FSK mode
            forme = true;
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Packet received is for me ##"));
#endif
        }
        else
        {
            forme = false;
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Packet received is not for me ##"));
            Serial.println();
#endif
        }
    }

    // added by C. Pham
    if (_hreceived==false || forme==false) {
        if( _modem == LORA )	// STANDBY PARA MINIMIZAR EL CONSUMO
        { // LoRa mode
            writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
        }
        else
        { //  FSK mode
            writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
        }
    }

    return forme;
}

/*
 Function: It gets and stores a packet if it is received before MAX_TIMEOUT expires.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPacketMAXTimeout()
{
    return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
int8_t SX1272::getPacket()
{
    return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   // added by C. Pham
   state = 5  --> The command has been executed with no errors and an ACK is requested
   state = 3  --> The command has been executed but packet has been incorrectly received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t SX1272::getPacket(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    //unsigned long previous;
    unsigned long exitTime;
    boolean p_received = false;

#if (SX1272_debug_mode > 0)
    Serial.println();
    Serial.println(F("Starting 'getPacket'"));
#endif

    //previous = millis();
    exitTime = millis() + (unsigned long)wait;
    if( _modem == LORA )
    { // LoRa mode
        value = readRegister(REG_IRQ_FLAGS);
        // Wait until the packet is received (RxDone flag) or the timeout expires
        //while( (bitRead(value, 6) == 0) && (millis() - previous < (unsigned long)wait) )
        while( (bitRead(value, 6) == 0) && (millis() < exitTime) )
        {
            value = readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        } // end while (millis)

        // modified by C. Pham
        // RxDone
        if ((bitRead(value, 6) == 1)) {
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Packet received in LoRa mode ##"));
#endif
            //CrcOnPayload?
            if (bitRead(readRegister(REG_HOP_CHANNEL),6)) {

                if ( (bitRead(value, 5) == 0) ) {
                    // packet received & CRC correct
                    p_received = true;	// packet correctly received
                    _reception = CORRECT_PACKET;
#if (SX1272_debug_mode > 0)
                    Serial.println(F("** The CRC is correct **"));
#endif
                }
                else {
                    _reception = INCORRECT_PACKET;
                    state = 3;
#if (SX1272_debug_mode > 0)
                    Serial.println(F("** The CRC is incorrect **"));
#endif
                }
            }
            else {
                  // as CRC is not set we suppose that CRC is correct
                  p_received = true;	// packet correctly received
                  _reception = CORRECT_PACKET;
#if (SX1272_debug_mode > 0)
                  Serial.println(F("## Packet supposed to be correct as CrcOnPayload is off at transmitter ##"));
#endif
             }
        }
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
    }
    else
    { // FSK mode
        value = readRegister(REG_IRQ_FLAGS2);
        //while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
        while( (bitRead(value, 2) == 0) && (millis() < exitTime) )
        {
            value = readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        } // end while (millis)

        if( bitRead(value, 2) == 1 )
        { // packet received
            if( bitRead(value, 1) == 1 )
            { // CRC correct
                _reception = CORRECT_PACKET;
                p_received = true;
#if (SX1272_debug_mode > 0)
                Serial.println(F("## Packet correctly received in FSK mode ##"));
#endif
            }
            else
            { // CRC incorrect
                _reception = INCORRECT_PACKET;
                state = 3;
                p_received = false;
#if (SX1272_debug_mode > 0)
                Serial.println(F("## Packet incorrectly received in FSK mode ##"));
#endif
            }
        }
        else
        {
#if (SX1272_debug_mode > 0)
            Serial.println(F("** The timeout has expired **"));
            Serial.println();
#endif
        }
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
    }
    if( p_received == true )
    {
        // Store the packet
        if( _modem == LORA )
        {
            // comment by C. Pham
            // set the FIFO addr to 0 to read again all the bytes
            writeRegister(REG_FIFO_ADDR_PTR, 0x00);  	// Setting address pointer in FIFO data buffer

#ifdef W_NET_KEY
            // added by C. Pham
            packet_received.netkey[0]=readRegister(REG_FIFO);
            packet_received.netkey[1]=readRegister(REG_FIFO);
#endif
            //modified by C. Pham
            if (!_rawFormat)
                packet_received.dst = readRegister(REG_FIFO);	// Storing first byte of the received packet
            else
                packet_received.dst = 0;
        }
        else
        {
            value = readRegister(REG_PACKET_CONFIG1);
            if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
            {
                packet_received.dst = readRegister(REG_FIFO); // Storing first byte of the received packet
            }
            else
            {
                packet_received.dst = _destination;			// Storing first byte of the received packet
            }
        }

        // modified by C. Pham
        if (!_rawFormat) {
            packet_received.type = readRegister(REG_FIFO);		// Reading second byte of the received packet
            // check packet type to discard unknown packet type
            if ( (packet_received.type & PKT_TYPE_MASK != PKT_TYPE_DATA) && (packet_received.type & PKT_TYPE_MASK != PKT_TYPE_ACK) ) {
                _reception = INCORRECT_PACKET_TYPE;
                state = 3;
#if (SX1272_debug_mode > 0)
                Serial.println(F("** The packet type is incorrect **"));
#endif            	
				return state;	
            }             
            packet_received.src = readRegister(REG_FIFO);		// Reading second byte of the received packet
            packet_received.packnum = readRegister(REG_FIFO);	// Reading third byte of the received packet
            //packet_received.length = readRegister(REG_FIFO);	// Reading fourth byte of the received packet
        }
        else {
            packet_received.type = 0;
            packet_received.src = 0;
            packet_received.packnum = 0;
        }

        packet_received.length = readRegister(REG_RX_NB_BYTES);

        if( _modem == LORA )
        {
            if (_rawFormat) {
                _payloadlength=packet_received.length;
            }
            else
                _payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
        }
        if( packet_received.length > (MAX_LENGTH + 1) )
        {
#if (SX1272_debug_mode > 0)
            Serial.println(F("Corrupted packet, length must be less than 256"));
#endif
        }
        else
        {
            for(unsigned int i = 0; i < _payloadlength; i++)
            {
                packet_received.data[i] = readRegister(REG_FIFO); // Storing payload
            }

            // commented by C. Pham
            //packet_received.retry = readRegister(REG_FIFO);

            // Print the packet if debug_mode
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Packet received:"));
            Serial.print(F("Destination: "));
            Serial.println(packet_received.dst);			 	// Printing destination
            Serial.print(F("Type: "));
            Serial.println(packet_received.type);			 	// Printing type
            Serial.print(F("Source: "));
            Serial.println(packet_received.src);			 	// Printing source
            Serial.print(F("Packet number: "));
            Serial.println(packet_received.packnum);			// Printing packet number
            Serial.print(F("Packet length: "));
            Serial.println(packet_received.length);			// Printing packet length
            Serial.print(F("Data: "));
            for(unsigned int i = 0; i < _payloadlength; i++)
            {
                Serial.print((char)packet_received.data[i]);		// Printing payload
            }
            Serial.println();
            //Serial.print(F("Retry number: "));
            //Serial.println(packet_received.retry);			// Printing number retry
            Serial.println(F("##"));
            Serial.println();
#endif
            state = 0;

#ifdef W_REQUESTED_ACK
            // added by C. Pham
            // need to send an ACK
            if (packet_received.type & PKT_FLAG_ACK_REQ) {
                state = 5;
                _requestACK_indicator=1;
            }
            else
                _requestACK_indicator=0;
#endif
        }
    }
    else
    {
        //state = 1;
        if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) )
        {
            // comment by C. Pham
            // what is the purpose of incrementing retries here?
            // bug? not needed?
            _retries++;
#if (SX1272_debug_mode > 0)
            Serial.println(F("## Retrying to send the last packet ##"));
            Serial.println();
#endif
        }
    }
    if( _modem == LORA )
    {
        writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    }
    clearFlags();	// Initializing flags
    if( wait > MAX_WAIT )
    {
        state = -1;
#if (SX1272_debug_mode > 0)
        Serial.println(F("** The timeout must be smaller than 12.5 seconds **"));
        Serial.println();
#endif
    }

    return state;
}

/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t SX1272::setDestination(uint8_t dest)
{
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setDestination'"));
#endif

    state = 1;
    _destination = dest; // Storing destination in a global variable
    packet_sent.dst = dest;	 // Setting destination in packet structure
    packet_sent.src = _nodeAddress; // Setting source in packet structure
    packet_sent.packnum = _packetNumber;	// Setting packet number in packet structure
    _packetNumber++;
    state = 0;

#if (SX1272_debug_mode > 1)
    Serial.print(F("## Destination "));
    Serial.print(_destination, HEX);
    Serial.println(F(" successfully set ##"));
    Serial.print(F("## Source "));
    Serial.print(packet_sent.src, DEC);
    Serial.println(F(" successfully set ##"));
    Serial.print(F("## Packet number "));
    Serial.print(packet_sent.packnum, DEC);
    Serial.println(F(" successfully set ##"));
    Serial.println();
#endif
    return state;
}

/*
 Function: It sets the timeout according to the configured mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setTimeout()
{
    uint8_t state = 2;
    //uint16_t delay;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setTimeout'"));
#endif

    state = 1;

    // changed by C. Pham
    // we always use MAX_TIMEOUT
    _sendTime = MAX_TIMEOUT;

    /*
    if( _modem == LORA )
    {
        switch(_spreadingFactor)
        {	// Choosing Spreading Factor
        case SF_6:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 335;
                    break;
                case CR_6: _sendTime = 352;
                    break;
                case CR_7: _sendTime = 368;
                    break;
                case CR_8: _sendTime = 386;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 287;
                    break;
                case CR_6: _sendTime = 296;
                    break;
                case CR_7: _sendTime = 305;
                    break;
                case CR_8: _sendTime = 312;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 242;
                    break;
                case CR_6: _sendTime = 267;
                    break;
                case CR_7: _sendTime = 272;
                    break;
                case CR_8: _sendTime = 276;
                    break;
                }
                break;
            }
            break;

        case SF_7:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 408;
                    break;
                case CR_6: _sendTime = 438;
                    break;
                case CR_7: _sendTime = 468;
                    break;
                case CR_8: _sendTime = 497;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 325;
                    break;
                case CR_6: _sendTime = 339;
                    break;
                case CR_7: _sendTime = 355;
                    break;
                case CR_8: _sendTime = 368;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 282;
                    break;
                case CR_6: _sendTime = 290;
                    break;
                case CR_7: _sendTime = 296;
                    break;
                case CR_8: _sendTime = 305;
                    break;
                }
                break;
            }
            break;

        case SF_8:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 537;
                    break;
                case CR_6: _sendTime = 588;
                    break;
                case CR_7: _sendTime = 640;
                    break;
                case CR_8: _sendTime = 691;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 388;
                    break;
                case CR_6: _sendTime = 415;
                    break;
                case CR_7: _sendTime = 440;
                    break;
                case CR_8: _sendTime = 466;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 315;
                    break;
                case CR_6: _sendTime = 326;
                    break;
                case CR_7: _sendTime = 340;
                    break;
                case CR_8: _sendTime = 352;
                    break;
                }
                break;
            }
            break;

        case SF_9:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 774;
                    break;
                case CR_6: _sendTime = 864;
                    break;
                case CR_7: _sendTime = 954;
                    break;
                case CR_8: _sendTime = 1044;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 506;
                    break;
                case CR_6: _sendTime = 552;
                    break;
                case CR_7: _sendTime = 596;
                    break;
                case CR_8: _sendTime = 642;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 374;
                    break;
                case CR_6: _sendTime = 396;
                    break;
                case CR_7: _sendTime = 418;
                    break;
                case CR_8: _sendTime = 441;
                    break;
                }
                break;
            }
            break;

        case SF_10:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1226;
                    break;
                case CR_6: _sendTime = 1388;
                    break;
                case CR_7: _sendTime = 1552;
                    break;
                case CR_8: _sendTime = 1716;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 732;
                    break;
                case CR_6: _sendTime = 815;
                    break;
                case CR_7: _sendTime = 896;
                    break;
                case CR_8: _sendTime = 977;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 486;
                    break;
                case CR_6: _sendTime = 527;
                    break;
                case CR_7: _sendTime = 567;
                    break;
                case CR_8: _sendTime = 608;
                    break;
                }
                break;
            }
            break;

        case SF_11:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 2375;
                    break;
                case CR_6: _sendTime = 2735;
                    break;
                case CR_7: _sendTime = 3095;
                    break;
                case CR_8: _sendTime = 3456;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1144;
                    break;
                case CR_6: _sendTime = 1291;
                    break;
                case CR_7: _sendTime = 1437;
                    break;
                case CR_8: _sendTime = 1586;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 691;
                    break;
                case CR_6: _sendTime = 766;
                    break;
                case CR_7: _sendTime = 838;
                    break;
                case CR_8: _sendTime = 912;
                    break;
                }
                break;
            }
            break;

        case SF_12: switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 4180;
                    break;
                case CR_6: _sendTime = 4836;
                    break;
                case CR_7: _sendTime = 5491;
                    break;
                case CR_8: _sendTime = 6146;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1965;
                    break;
                case CR_6: _sendTime = 2244;
                    break;
                case CR_7: _sendTime = 2521;
                    break;
                case CR_8: _sendTime = 2800;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1102;
                    break;
                case CR_6: _sendTime = 1241;
                    break;
                case CR_7: _sendTime = 1381;
                    break;
                case CR_8: _sendTime = 1520;
                    break;
                }
                break;
            }
            break;
        default: _sendTime = MAX_TIMEOUT;
        }
    }
    else
    {
        _sendTime = MAX_TIMEOUT;
    }
    delay = ((0.1*_sendTime) + 1);
    _sendTime = (uint16_t) ((_sendTime * 1.2) + (rand()%delay));

    */
#if (SX1272_debug_mode > 1)
    Serial.print(F("Timeout to send/receive is: "));
    Serial.println(_sendTime, DEC);
#endif
    state = 0;
    return state;
}

/*
 Function: It sets a char array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPayload(char *payload)
{
    uint8_t state = 2;
    uint8_t state_f = 2;
    uint16_t length16;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPayload'"));
#endif

    state = 1;
    length16 = (uint16_t)strlen(payload);
    state = truncPayload(length16);
    if( state == 0 )
    {
        // fill data field until the end of the string
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            packet_sent.data[i] = payload[i];
        }
    }
    else
    {
        state_f = state;
    }
    if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
    {
        _payloadlength = MAX_PAYLOAD_FSK;
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("In FSK, payload length must be less than 60 bytes."));
        Serial.println();
#endif
    }
    // set length with the actual counter value
    state_f = setPacketLength();	// Setting packet length in packet structure
    return state_f;
}

/*
 Function: It sets an uint8_t array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPayload(uint8_t *payload)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPayload'"));
#endif

    state = 1;
    if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
    {
        _payloadlength = MAX_PAYLOAD_FSK;
        state = 1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("In FSK, payload length must be less than 60 bytes."));
        Serial.println();
#endif
    }
    for(unsigned int i = 0; i < _payloadlength; i++)
    {
        packet_sent.data[i] = payload[i];	// Storing payload in packet structure
    }
    // set length with the actual counter value
    state = setPacketLength();	// Setting packet length in packet structure
    return state;
}

/*
 Function: It sets a packet struct in FIFO in order to send it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPacket(uint8_t dest, char *payload)
{
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPacket'"));
#endif

    // added by C. Pham
    // check for enough remaining ToA
    // when operating under duty-cycle mode
    if (_limitToA) {
        uint16_t length16 = (uint16_t)strlen(payload);

        if (!_rawFormat)
            length16 = length16 + OFFSET_PAYLOADLENGTH;

        if (getRemainingToA() - getToA(length16) < 0) {
            Serial.print(F("## not enough ToA at "));
            Serial.println(millis());
            return SX1272_ERROR_TOA;
        }
    }

    clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    _reception = CORRECT_PACKET;	// Updating incorrect value
    if( _retries == 0 )
    { // Updating this values only if is not going to re-send the last packet
        state = setDestination(dest);	// Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = setPacketLength();
        packet_sent.retry = _retries;
#if (SX1272_debug_mode > 0)
        Serial.print(F("** Retrying to send last packet "));
        Serial.print(_retries, DEC);
        Serial.println(F(" time **"));
#endif
    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;

#ifdef W_REQUESTED_ACK
    // added by C. Pham
    // indicate that an ACK should be sent by the receiver
    if (_requestACK)
        packet_sent.type |= PKT_FLAG_ACK_REQ;
#endif

    writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO
#ifdef W_NET_KEY
        // added by C. Pham
        packet_sent.netkey[0]=_my_netkey[0];
        packet_sent.netkey[1]=_my_netkey[1];
        //#if (SX1272_debug_mode > 0)
        Serial.println(F("## Setting net key ##"));
        //#endif
        writeRegister(REG_FIFO, packet_sent.netkey[0]);
        writeRegister(REG_FIFO, packet_sent.netkey[1]);
#endif
        // added by C. Pham
        // we can skip the header for instance when we want to generate
        // at a higher layer a LoRaWAN packet
        if (!_rawFormat) {
            writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
            // added by C. Pham
            writeRegister(REG_FIFO, packet_sent.type); 		// Writing the packet type in FIFO
            writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
            writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
        state = 0;
#if (SX1272_debug_mode > 0)
        Serial.println(F("## Packet set and written in FIFO ##"));
        // Print the complete packet if debug_mode
        Serial.println(F("## Packet to send: "));
        Serial.print(F("Destination: "));
        Serial.println(packet_sent.dst);			 	// Printing destination
        Serial.print(F("Packet type: "));
        Serial.println(packet_sent.type);			// Printing packet type
        Serial.print(F("Source: "));
        Serial.println(packet_sent.src);			 	// Printing source
        Serial.print(F("Packet number: "));
        Serial.println(packet_sent.packnum);			// Printing packet number
        Serial.print(F("Packet length: "));
        Serial.println(packet_sent.length);			// Printing packet length
        Serial.print(F("Data: "));
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            Serial.print((char)packet_sent.data[i]);		// Printing payload
        }
        Serial.println();
        //Serial.print(F("Retry number: "));
        //Serial.println(packet_sent.retry);			// Printing retry number
        Serial.println(F("##"));
#endif
    }

    return state;
}

/*
 Function: It sets a packet struct in FIFO in order to sent it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPacket(uint8_t dest, uint8_t *payload)
{
    int8_t state = 2;
    byte st0;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPacket'"));
#endif

    // added by C. Pham
    // check for enough remaining ToA
    // when operating under duty-cycle mode
    if (_limitToA) {
        // here truncPayload() should have been called before in
        // sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16)
        uint16_t length16 = _payloadlength;

        if (!_rawFormat)
            length16 = length16 + OFFSET_PAYLOADLENGTH;

        if (getRemainingToA() - getToA(length16) < 0) {
            Serial.print(F("## not enough ToA at "));
            Serial.println(millis());
            return SX1272_ERROR_TOA;
        }
    }

    st0 = readRegister(REG_OP_MODE);	// Save the previous status
    clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    _reception = CORRECT_PACKET;	// Updating incorrect value to send a packet (old or new)
    if( _retries == 0 )
    { // Sending new packet
        state = setDestination(dest);	// Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = setPacketLength();
        packet_sent.retry = _retries;
#if (SX1272_debug_mode > 0)
        Serial.print(F("** Retrying to send last packet "));
        Serial.print(_retries, DEC);
        Serial.println(F(" time **"));
#endif
    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;

#ifdef W_REQUESTED_ACK
    // added by C. Pham
    // indicate that an ACK should be sent by the receiver
    if (_requestACK)
        packet_sent.type |= PKT_FLAG_ACK_REQ;
#endif

    writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO
#ifdef W_NET_KEY
        // added by C. Pham
        packet_sent.netkey[0]=_my_netkey[0];
        packet_sent.netkey[1]=_my_netkey[1];
        //#if (SX1272_debug_mode > 0)
        Serial.println(F("## Setting net key ##"));
        //#endif
        writeRegister(REG_FIFO, packet_sent.netkey[0]);
        writeRegister(REG_FIFO, packet_sent.netkey[1]);
#endif
        // added by C. Pham
        // we can skip the header for instance when we want to generate
        // at a higher layer a LoRaWAN packet
        if (!_rawFormat) {
            writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
            // added by C. Pham
            writeRegister(REG_FIFO, packet_sent.type); 		// Writing the packet type in FIFO
            writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
            writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
        state = 0;
#if (SX1272_debug_mode > 0)
        Serial.println(F("## Packet set and written in FIFO ##"));
        // Print the complete packet if debug_mode
        Serial.println(F("## Packet to send: "));
        Serial.print(F("Destination: "));
        Serial.println(packet_sent.dst);			 	// Printing destination
        Serial.print(F("Packet type: "));
        Serial.println(packet_sent.type);			// Printing packet type
        Serial.print(F("Source: "));
        Serial.println(packet_sent.src);			 	// Printing source
        Serial.print(F("Packet number: "));
        Serial.println(packet_sent.packnum);			// Printing packet number
        Serial.print(F("Packet length: "));
        Serial.println(packet_sent.length);			// Printing packet length
        Serial.print(F("Data: "));
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            Serial.print((char)packet_sent.data[i]);		// Printing payload
        }
        Serial.println();
        //Serial.print(F("Retry number: "));
        //Serial.println(packet_sent.retry);			// Printing retry number
        Serial.println(F("##"));
#endif
    }
    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendWithMAXTimeout()
{
    return sendWithTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendWithTimeout()
{
    setTimeout();
    return sendWithTimeout(_sendTime);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendWithTimeout(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    //unsigned long previous;
    unsigned long exitTime;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendWithTimeout'"));
#endif

    // clearFlags();	// Initializing flags

    // wait to TxDone flag
    //previous = millis();
    exitTime = millis() + (unsigned long)wait;
    if( _modem == LORA )
    { // LoRa mode
        clearFlags();	// Initializing flags

        writeRegister(REG_OP_MODE, LORA_TX_MODE);  // LORA mode - Tx

#if (SX1272_debug_mode > 1)
        value = readRegister(REG_OP_MODE);

        if (value & LORA_TX_MODE == LORA_TX_MODE)
            Serial.println(F("OK"));
        else
            Serial.println(F("ERROR"));
#endif
        value = readRegister(REG_IRQ_FLAGS);
        // Wait until the packet is sent (TX Done flag) or the timeout expires
        //while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
        while ((bitRead(value, 3) == 0) && (millis() < exitTime))
        {
            value = readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        }
        state = 1;
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_TX_MODE);  // FSK mode - Tx

        value = readRegister(REG_IRQ_FLAGS2);
        // Wait until the packet is sent (Packet Sent flag) or the timeout expires
        //while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
        while ((bitRead(value, 3) == 0) && (millis() < exitTime))
        {
            value = readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        }
        state = 1;
    }
    if( bitRead(value, 3) == 1 )
    {
        state = 0;	// Packet successfully sent
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Packet successfully sent ##"));
        Serial.println();
#endif
        // added by C. Pham
        // normally there should be enough remaing ToA as the test has been done earlier
        if (_limitToA)
            removeToA(_currentToA);
    }
    else
    {
        if( state == 1 )
        {
#if (SX1272_debug_mode > 1)
            Serial.println(F("** Timeout has expired **"));
            Serial.println();
#endif
        }
        else
        {
#if (SX1272_debug_mode > 1)
            Serial.println(F("** There has been an error and packet has not been sent **"));
            Serial.println();
#endif
        }
    }

    clearFlags();		// Initializing flags
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeout(uint8_t dest, char *payload)
{
    return sendPacketTimeout(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeout(uint8_t dest,  uint8_t *payload, uint16_t length16)
{
    return sendPacketTimeout(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeout(uint8_t dest, char *payload)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

    state = setPacket(dest, payload);	// Setting a packet with 'dest' destination
    if (state == 0)								// and writing it in FIFO.
    {
        state = sendWithTimeout();	// Sending the packet
    }
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

    state = truncPayload(length16);

    if( state == 0 )
    {
        state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
    }												// and writing it in FIFO.
    else
    {
        state_f = state;
    }
    if( state_f == 0 )
    {
        state_f = sendWithTimeout();	// Sending the packet
    }
    return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeout(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

    state = setPacket(dest, payload);	// Setting a packet with 'dest' destination
    if (state == 0)								// and writing it in FIFO.
    {
        state = sendWithTimeout(wait);	// Sending the packet
    }
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeout'"));
#endif

    state = truncPayload(length16);
    if( state == 0 )
    {
        state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
    }
    else
    {
        state_f = state;
    }
    if( state_f == 0 )								// and writing it in FIFO.
    {
        state_f = sendWithTimeout(wait);	// Sending the packet
    }
    return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeoutACK(uint8_t dest, char *payload)
{
    return sendPacketTimeoutACK(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeoutACK(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    return sendPacketTimeoutACK(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, char *payload)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sendPacketTimeout(dest, payload);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = state;
    }
    if( state == 0 )
    {
        // added by C. Pham
        Serial.println(F("wait for ACK"));

        if( availableData() )
        {
            state_f = getACK();	// Getting ACK
        }
        else
        {
            state_f = SX1272_ERROR_ACK;
            // added by C. Pham
            Serial.println(F("no ACK"));
        }
    }
    else
    {
        state_f = state;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    // Sending packet to 'dest' destination
    state = sendPacketTimeout(dest, payload, length16);

    // Trying to receive the ACK
    if( state == 0 )
    {
        state = receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = state;
    }
    if( state == 0 )
    {
        // added by C. Pham
        Serial.println(F("wait for ACK"));

        if( availableData() )
        {
            state_f = getACK();	// Getting ACK
        }
        else
        {
            state_f = SX1272_ERROR_ACK;
            // added by C. Pham
            Serial.println(F("no ACK"));
        }
    }
    else
    {
        state_f = state;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sendPacketTimeout(dest, payload, wait);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = 1;
    }
    if( state == 0 )
    {
        // added by C. Pham
        Serial.println(F("wait for ACK"));

        if( availableData() )
        {
            state_f = getACK();	// Getting ACK
        }
        else
        {
            state_f = SX1272_ERROR_ACK;
            // added by C. Pham
            Serial.println(F("no ACK"));
        }
    }
    else
    {
        state_f = 1;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACK(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACK'"));
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sendPacketTimeout(dest, payload, length16, wait);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = 1;
    }
    if( state == 0 )
    {
        // added by C. Pham
        Serial.println(F("wait for ACK"));

        if( availableData() )
        {
            state_f = getACK();	// Getting ACK
        }
        else
        {
            state_f = SX1272_ERROR_ACK;
            // added by C. Pham
            Serial.println(F("no ACK"));
        }
    }
    else
    {
        state_f = 1;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: It gets and stores an ACK if it is received.
 Returns:
*/
uint8_t SX1272::getACK()
{
    return getACK(MAX_TIMEOUT);
}

/*
 Function: It gets and stores an ACK if it is received, before ending 'wait' time.
 Returns: Integer that determines if there has been any error
   state = 2  --> The ACK has not been received
   state = 1  --> The N-ACK has been received with no errors
   state = 0  --> The ACK has been received with no errors
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
uint8_t SX1272::getACK(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    //unsigned long previous;
    unsigned long exitTime;
    boolean a_received = false;

    //#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getACK'"));
    //#endif

    //previous = millis();
    exitTime = millis()+(unsigned long)wait;
    if( _modem == LORA )
    { // LoRa mode
        value = readRegister(REG_IRQ_FLAGS);
        // Wait until the ACK is received (RxDone flag) or the timeout expires
        //while ((bitRead(value, 6) == 0) && (millis() - previous < wait))
        while ((bitRead(value, 6) == 0) && (millis() < exitTime))
        {
            value = readRegister(REG_IRQ_FLAGS);
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        }
        if( bitRead(value, 6) == 1 )
        { // ACK received
            // comment by C. Pham
            // not really safe because the received packet may not be an ACK
            // probability is low if using unicast to gateway, but if broadcast
            // can get a packet from another node!!
            a_received = true;
        }
        // Standby para minimizar el consumo
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
    }
    else
    { // FSK mode
        value = readRegister(REG_IRQ_FLAGS2);
        // Wait until the packet is received (RxDone flag) or the timeout expires
        //while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
        while ((bitRead(value, 2) == 0) && (millis() < exitTime))
        {
            value = readRegister(REG_IRQ_FLAGS2);
            //if( millis() < previous )
            //{
            //    previous = millis();
            //}
        }
        if( bitRead(value, 2) == 1 )
        { // ACK received
            a_received = true;
        }
        // Standby para minimizar el consumo
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
    }

    // comment by C. Pham
    // not safe because the received packet may not be an ACK!
    if( a_received )
    {
        // Storing the received ACK
        ACK.dst = _destination;
        ACK.type = readRegister(REG_FIFO);
        ACK.src = readRegister(REG_FIFO);
        ACK.packnum = readRegister(REG_FIFO);
        ACK.length = readRegister(REG_FIFO);
        ACK.data[0] = readRegister(REG_FIFO);
        ACK.data[1] = readRegister(REG_FIFO);

        if (ACK.type == PKT_TYPE_ACK) {

            // Checking the received ACK
            if( ACK.dst == packet_sent.src )
            {
                if( ACK.src == packet_sent.dst )
                {
                    if( ACK.packnum == packet_sent.packnum )
                    {
                        if( ACK.length == 2 )
                        {
                            if( ACK.data[0] == CORRECT_PACKET )
                            {
                                state = 0;
                                //#if (SX1272_debug_mode > 0)
                                // Printing the received ACK
                                Serial.println(F("## ACK received:"));
                                Serial.print(F("Destination: "));
                                Serial.println(ACK.dst);			 	// Printing destination
                                Serial.print(F("Source: "));
                                Serial.println(ACK.src);			 	// Printing source
                                Serial.print(F("ACK number: "));
                                Serial.println(ACK.packnum);			// Printing ACK number
                                Serial.print(F("ACK length: "));
                                Serial.println(ACK.length);				// Printing ACK length
                                Serial.print(F("ACK payload: "));
                                Serial.println(ACK.data[0]);			// Printing ACK payload
                                Serial.print(F("ACK SNR of rcv pkt at gw: "));

                                value = ACK.data[1];

                                if( value & 0x80 ) // The SNR sign bit is 1
                                {
                                    // Invert and divide by 4
                                    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                                    _rcv_snr_in_ack = -value;
                                }
                                else
                                {
                                    // Divide by 4
                                    _rcv_snr_in_ack = ( value & 0xFF ) >> 2;
                                }

                                Serial.println(_rcv_snr_in_ack);
                                Serial.println(F("##"));
                                Serial.println();
                                //#endif
                            }
                            else
                            {
                                state = 1;
                                #if (SX1272_debug_mode > 0)
                                Serial.println(F("** N-ACK received **"));
                                Serial.println();
                                #endif
                            }
                        }
                        else
                        {
                            state = 1;
                            #if (SX1272_debug_mode > 0)
                            Serial.println(F("** ACK length incorrectly received **"));
                            Serial.println();
                            #endif
                        }
                    }
                    else
                    {
                        state = 1;
                        #if (SX1272_debug_mode > 0)
                        Serial.println(F("** ACK number incorrectly received **"));
                        Serial.println();
                        #endif
                    }
                }
                else
                {
                    state = 1;
                    #if (SX1272_debug_mode > 0)
                    Serial.println(F("** ACK source incorrectly received **"));
                    Serial.println();
                    #endif
                }
            }
        }
        else
        {
            state = 1;
            #if (SX1272_debug_mode > 0)
            Serial.println(F("** ACK destination incorrectly received **"));
            Serial.println();
            #endif
        }
    }
    else
    {
        state = 1;
        #if (SX1272_debug_mode > 0)
        Serial.println(F("** ACK lost **"));
        Serial.println();
        #endif
    }
    clearFlags();	// Initializing flags
    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeoutACKRetries(uint8_t dest, char  *payload)
{
    return sendPacketTimeoutACKRetries(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketMAXTimeoutACKRetries(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    return sendPacketTimeoutACKRetries(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACKRetries(uint8_t dest, char *payload)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACKRetries'"));
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while( (state != 0) && (_retries <= _maxRetries) )
    {
        state = sendPacketTimeoutACK(dest, payload);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACKRetries(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACKRetries'"));
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sendPacketTimeoutACK(dest, payload, length16);
        _retries++;

    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACKRetries(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACKRetries'"));
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sendPacketTimeoutACK(dest, payload, wait);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeoutACKRetries(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'sendPacketTimeoutACKRetries'"));
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sendPacketTimeoutACK(dest, payload, length16, wait);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getTemp()
{
    byte st0;
    uint8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getTemp'"));
#endif

    st0 = readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == LORA )
    { // Allowing access to FSK registers while in LoRa standby mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }

    state = 1;
    // Saving temperature value
    _temp = readRegister(REG_TEMP);
    if( _temp & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        _temp = ( ( ~_temp + 1 ) & 0xFF );
    }
    else
    {
        // Divide by 4
        _temp = ( _temp & 0xFF );
    }


#if (SX1272_debug_mode > 1)
    Serial.print(F("## Temperature is: "));
    Serial.print(_temp);
    Serial.println(F(" ##"));
    Serial.println();
#endif

    if( _modem == LORA )
    {
        writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    }

    state = 0;
    return state;
}

//**********************************************************************/
// Added by C. Pham
//**********************************************************************/

void SX1272::setPacketType(uint8_t type)
{
    packet_sent.type=type;

    if (type & PKT_FLAG_ACK_REQ)
        _requestACK=1;
}

/*
 Function: Configures the module to perform CAD.
 Returns: Integer that determines if the number of requested CAD have been successfull
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::doCAD(uint8_t counter)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long startCAD, endCAD, startDoCad, endDoCad;
    //unsigned long previous;
    unsigned long exitTime;
    uint16_t wait = 100;
    bool failedCAD=false;
    uint8_t retryCAD = 3;
    uint8_t save_counter;
    byte st0;
    int rssi_count=0;
    int rssi_mean=0;
    double bw=0.0;
    bool hasRSSI=false;
    unsigned long startRSSI=0;

    bw=(_bandwidth==BW_125)?125e3:((_bandwidth==BW_250)?250e3:500e3);
    // Symbol rate : time for one symbol (usecs)
    double rs = bw / ( 1 << _spreadingFactor);
    double ts = 1 / rs;
    ts = ts * 1000000.0;

    st0 = readRegister(REG_OP_MODE);	// Save the previous status

#ifdef DEBUG_CAD
    Serial.println(F("SX1272::Starting 'doCAD'"));
#endif

    save_counter = counter;

    startDoCad=millis();

    if( _modem == LORA ) { // LoRa mode

        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

        do {

            hasRSSI=false;

            clearFlags();	// Initializing flags

            // wait to CadDone flag
            // previous = millis();
            startCAD = millis();
            exitTime = millis()+(unsigned long)wait;

            writeRegister(REG_OP_MODE, LORA_CAD_MODE);  // LORA mode - Cad

            startRSSI=micros();

            value = readRegister(REG_IRQ_FLAGS);
            // Wait until CAD ends (CAD Done flag) or the timeout expires
            //while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
            while ((bitRead(value, 2) == 0) && (millis() < exitTime))
            {
                // only one reading per CAD
                if (micros()-startRSSI > ts+240 && !hasRSSI) {
                    _RSSI = -(OFFSET_RSSI+(_board==SX1276Chip?18:0)) + readRegister(REG_RSSI_VALUE_LORA);
                    rssi_mean += _RSSI;
                    rssi_count++;
                    hasRSSI=true;
                }

                value = readRegister(REG_IRQ_FLAGS);
                // Condition to avoid an overflow (DO NOT REMOVE)
                //if( millis() < previous )
                //{
                //    previous = millis();
                //}
            }
            state = 1;

            endCAD = millis();

            if( bitRead(value, 2) == 1 )
            {
                state = 0;	// CAD successfully performed
#ifdef DEBUG_CAD				  
                Serial.print(F("SX1272::CAD duration "));
                Serial.println(endCAD-startCAD);
                Serial.println(F("SX1272::CAD successfully performed"));
#endif				  

                value = readRegister(REG_IRQ_FLAGS);

                // look for the CAD detected bit
                if( bitRead(value, 0) == 1 )
                {
                    // we detected activity
                    failedCAD=true;
#ifdef DEBUG_CAD				  		
                    Serial.print(F("SX1272::CAD exits after "));
                    Serial.println(save_counter-counter);
#endif				  		
                }

                counter--;
            }
            else
            {
#ifdef DEBUG_CAD			  	 	
                Serial.print(F("SX1272::CAD duration "));
                Serial.println(endCAD-startCAD);
#endif				  
                if( state == 1 )
                {
#ifdef DEBUG_CAD
                    Serial.println(F("SX1272::Timeout has expired"));
#endif
                }
                else
                {
#ifdef DEBUG_CAD
                    Serial.println(F("SX1272::Error and CAD has not been performed"));
#endif
                }

                retryCAD--;

                // to many errors, so exit by indicating that channel is not free
                if (!retryCAD)
                    failedCAD=true;
            }

        } while (counter && !failedCAD);

        rssi_mean = rssi_mean / rssi_count;
        _RSSI = rssi_mean;
    }

    writeRegister(REG_OP_MODE, st0);

    endDoCad=millis();

    clearFlags();		// Initializing flags

#ifdef DEBUG_CAD	  
    Serial.print(F("SX1272::doCAD duration "));
    Serial.println(endDoCad-startDoCad);
#endif

    if (failedCAD)
        return 2;

    return state;
}

//#define DEBUG_GETTOA

#ifdef DEBUG_GETTOA

void printDouble( double val, byte precision){
    // prints val with number of decimal places determine by precision
    // precision is a number from 0 to 6 indicating the desired decimial places
    // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

    if(val < 0.0){
        Serial.print('-');
        val = -val;
    }

    Serial.print (int(val));  //prints the int part
    if( precision > 0) {
        Serial.print("."); // print the decimal point
        unsigned long frac;
        unsigned long mult = 1;
        byte padding = precision -1;
        while(precision--)
            mult *=10;

        if(val >= 0)
            frac = (val - int(val)) * mult;
        else
            frac = (int(val)- val ) * mult;
        unsigned long frac1 = frac;
        while( frac1 /= 10 )
            padding--;
        while(  padding--)
            Serial.print("0");
        Serial.print(frac,DEC) ;
    }
}

#endif

uint16_t SX1272::getToA(uint8_t pl) {

    uint8_t DE = 0;
    uint32_t airTime = 0;

    double bw=0.0;

    bw=(_bandwidth==BW_125)?125e3:((_bandwidth==BW_250)?250e3:500e3);

#ifdef DEBUG_GETTOA
    Serial.print(F("SX1272::bw is "));
    Serial.println(bw);

    Serial.print(F("SX1272::SF is "));
    Serial.println(_spreadingFactor);
#endif

    //double ts=pow(2,_spreadingFactor)/bw;

    ////// from LoRaMAC SX1272GetTimeOnAir()

    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << _spreadingFactor);
    double ts = 1 / rs;

    // must add 4 to the programmed preamble length to get the effective preamble length
    double tPreamble=((_preamblelength+4)+4.25)*ts;

#ifdef DEBUG_GETTOA	
    Serial.print(F("SX1272::ts is "));
    printDouble(ts,6);
    Serial.println();
    Serial.print(F("SX1272::tPreamble is "));
    printDouble(tPreamble,6);
    Serial.println();
#endif

    // for low data rate optimization
    if ((_bandwidth == BW_125) && _spreadingFactor == 12)
        DE = 1;

    // Symbol length of payload and time
    double tmp = (8*pl - 4*_spreadingFactor + 28 + 16 - 20*_header) /
            (double)(4*(_spreadingFactor-2*DE) );

#ifdef DEBUG_GETTOA                         
    Serial.print(F("SX1272::tmp is "));
    printDouble(tmp,6);
    Serial.println();
#endif

    tmp = ceil(tmp)*(_codingRate + 4);

    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );

#ifdef DEBUG_GETTOA    
    Serial.print(F("SX1272::nPayload is "));
    Serial.println(nPayload);
#endif

    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // in us secs
    airTime = floor( tOnAir * 1e6 + 0.999 );

    //////

#ifdef DEBUG_GETTOA    
    Serial.print(F("SX1272::airTime is "));
    Serial.println(airTime);
#endif
    // return in ms
    _currentToA=ceil(airTime/1000)+1;
    return _currentToA;
}

void SX1272::CarrierSense(uint8_t cs) {
    
    if (cs==1)
    	CarrierSense1();
    
    if (cs==2)
    	CarrierSense2(); 
    	
    if (cs==3)
    	CarrierSense3();     	
}    	  	   	

// need to set _send_cad_number to a value > 0
// we advise using _send_cad_number=3 for a SIFS and _send_cad_number=9 for a DIFS
// prior to send any data
void SX1272::CarrierSense1() {

    int e;
    bool carrierSenseRetry=false;
    uint8_t retries=3;
    uint8_t DIFSretries=8;

  	Serial.print(F("--> CS1\n")); 
  	
    if (_send_cad_number && _enableCarrierSense) {

        do {
            DIFSretries=8;
            do {

                // check for free channel (SIFS/DIFS)
                _startDoCad=millis();
                e = doCAD(_send_cad_number);
                _endDoCad=millis();

                Serial.print(F("--> CAD "));
                Serial.print(_endDoCad-_startDoCad);
                Serial.println();

                if (!e) {
                    Serial.print(F("OK1\n"));

                    if (_extendedIFS)  {
                        // wait for random number of CAD
                        uint8_t w = random(1,8);

                        Serial.print(F("--> wait for "));
                        Serial.print(w);
                        Serial.print(F(" CAD = "));
                        Serial.print(sx1272_CAD_value[_loraMode]*w);
                        Serial.println();

                        delay(sx1272_CAD_value[_loraMode]*w);

                        // check for free channel (SIFS/DIFS) once again
                        _startDoCad=millis();
                        e = doCAD(_send_cad_number);
                        _endDoCad=millis();

                        Serial.print(F("--> CAD "));
                        Serial.print(_endDoCad-_startDoCad);
                        Serial.println();

                        if (!e)
                            Serial.print(F("OK2"));
                        else
                            Serial.print(F("#2"));

                        Serial.println();
                    }
                }
                else {
                    Serial.print(F("#1\n"));

                    // wait for random number of DIFS
                    uint8_t w = random(1,8);

                    Serial.print(F("--> wait for "));
                    Serial.print(w);
                    Serial.print(F(" DIFS=3SIFS= "));
                    Serial.print(sx1272_SIFS_value[_loraMode]*3*w);
                    Serial.println();

                    delay(sx1272_SIFS_value[_loraMode]*3*w);

                    Serial.print(F("--> retry\n"));
                }

            } while (e && --DIFSretries);
		
            // CAD is OK, but need to check RSSI
            if (_RSSIonSend) {

                e=getRSSI();
                uint8_t rssi_retry_count=8;

                if (!e) {

                    do {
                        getRSSI();
                        Serial.print(F("--> RSSI "));
                        Serial.print(_RSSI);
                        Serial.println();
                        rssi_retry_count--;
                        delay(1);
                    } while (_RSSI > -90 && rssi_retry_count);
                }
                else
                    Serial.print(F("--> RSSI error\n"));

                if (!rssi_retry_count)
                    carrierSenseRetry=true;
                else
                    carrierSenseRetry=false;
            }
        } while (carrierSenseRetry && --retries);
    }
}

void SX1272::CarrierSense2() {

	int e;
	bool carrierSenseRetry=false;  
	uint8_t foundBusyDuringDIFSafterBusyState=0;
    uint8_t retries=3;
    uint8_t DIFSretries=8;
	uint8_t n_collision=0;
	// upper bound of the random backoff timer
	uint8_t W=2;
	uint32_t max_toa = sx1272.getToA(MAX_LENGTH);

	// do CAD for DIFS=9CAD
	Serial.print(F("--> CS2\n")); 
  
	if (_send_cad_number && _enableCarrierSense) {
    	  
		do { 
            DIFSretries=8;
			do {
                //D f W
                //2 2 4
                //3 3 8
                //4 4 16
                //5 5 16
                //6 6 16
                //...

                if (foundBusyDuringDIFSafterBusyState>1 && foundBusyDuringDIFSafterBusyState<5)
                    W=W*2;

                // check for free channel (SIFS/DIFS)
                _startDoCad=millis();
                e = sx1272.doCAD(_send_cad_number);
                _endDoCad=millis();

                Serial.print(F("--> DIFS "));
                Serial.print(_endDoCad-_startDoCad);
                Serial.println();

                // successull SIFS/DIFS
                if (!e) {

                    // previous collision detected
                    if (n_collision) {

                        Serial.print(F("--> count for "));
                        // count for random number of CAD/SIFS/DIFS?
                        // SIFS=3CAD
                        // DIFS=9CAD
                        uint8_t w = random(0,W*_send_cad_number);

                        Serial.println(w);

                        int busyCount=0;
                        bool nowBusy=false;

                        do {

                            if (nowBusy)
                                e = sx1272.doCAD(_send_cad_number);
                            else
                                e = sx1272.doCAD(1);

                            if (nowBusy && e) {
                                Serial.print(F("#"));
                                busyCount++;
                            }
                            else if (nowBusy && !e) {
                                Serial.print(F("|"));
                                nowBusy=false;
                            }
                            else if (!e) {
                                w--;
                                Serial.print(F("-"));
                            }
                            else {
                                Serial.print(F("*"));
                                nowBusy=true;
                                busyCount++;
                            }

                        } while (w);

                        // if w==0 then we exit and
                        // the packet will be sent
                        Serial.println();
                        Serial.print(F("--> busy during "));
                        Serial.println(busyCount);
                    }
                    else {
                        Serial.println(F("OK1"));

                        if (_extendedIFS)  {
                            // wait for random number of CAD
                            uint8_t w = random(1,8);

                            Serial.print(F("--> extended wait for "));
                            Serial.println(w);
                            Serial.print(F(" CAD = "));
                            Serial.println(sx1272_CAD_value[_loraMode]*w);

                            delay(sx1272_CAD_value[_loraMode]*w);

                            // check for free channel (SIFS/DIFS) once again
                            _startDoCad=millis();
                            e = sx1272.doCAD(_send_cad_number);
                            _endDoCad=millis();

                            Serial.print(F("--> CAD "));
                            Serial.println(_endDoCad-_startDoCad);

                            if (!e)
                                Serial.println("OK2");
                            else
                                Serial.println("#2");
                        }
                    }
                }
                else {
                    n_collision++;
                    foundBusyDuringDIFSafterBusyState++;
                    Serial.print(F("###"));
                    Serial.println(n_collision);

                    Serial.println(F("--> CAD until clear"));

                    int busyCount=0;

                    _startDoCad=millis();
                    do {

                        e = sx1272.doCAD(1);

                        if (e) {
                            Serial.print(F("R"));
                            busyCount++;
                        }
                    } while (e && (millis()-_startDoCad < 2*max_toa));

                    _endDoCad=millis();

                    Serial.println();
                    Serial.print(F("--> busy during "));
                    Serial.println(busyCount);

                    Serial.print(F("--> wait "));
                    Serial.println(_endDoCad-_startDoCad);

                    // to perform a new DIFS
                    Serial.println(F("--> retry"));
                    e=1;
                }
			} while (e && --DIFSretries);
	
			// CAD is OK, but need to check RSSI
			if (_RSSIonSend) {

				e=getRSSI();
				uint8_t rssi_retry_count=8;

				if (!e) {

                    do {
						getRSSI();
						Serial.print(F("--> RSSI "));
						Serial.print(_RSSI);
						Serial.println();
						rssi_retry_count--;
                        delay(1);
                    } while (_RSSI > -90 && rssi_retry_count);
				}
				else
					Serial.print(F("--> RSSI error\n"));

				if (!rssi_retry_count)
					carrierSenseRetry=true;
				else
					carrierSenseRetry=false;
			}
		} while (carrierSenseRetry && --retries);  
  	}
}

void SX1272::CarrierSense3() {

    int e;
    bool carrierSenseRetry=false;
    uint8_t n_collision=0;
    uint8_t retries=3;
    uint8_t n_cad=9;
    uint32_t max_toa = sx1272.getToA(MAX_LENGTH);

    Serial.println(F("--> CS3"));

    //unsigned long end_carrier_sense=0;

    if (_send_cad_number && _enableCarrierSense) {
        do {
            Serial.print(F("--> CAD for MaxToa="));
            Serial.println(max_toa);

            //end_carrier_sense=millis()+(max_toa/n_cad)*(n_cad-1);

            for (int i=0; i<n_cad; i++) {
                _startDoCad=millis();
                e = sx1272.doCAD(1);
                _endDoCad=millis();

                if (!e) {
                    Serial.print(_endDoCad);
                    Serial.print(F(" 0 "));
                    Serial.print(sx1272._RSSI);
                    Serial.print(F(" "));
                    Serial.println(_endDoCad-_startDoCad);
                }
                else
                    continue;

                // wait in order to have n_cad CAD operations during max_toa
                delay(max_toa/(n_cad-1)-(millis()-_startDoCad));
            }

            if (e) {
                n_collision++;
                Serial.print(F("#"));
                Serial.println(n_collision);

                Serial.print(F("Busy. Wait MaxToA="));
                Serial.println(max_toa);
                delay(max_toa);
                // to perform a new max_toa waiting
                Serial.println(F("--> retry"));
                carrierSenseRetry=true;
            }
            else
                carrierSenseRetry=false;

        } while (carrierSenseRetry && --retries);
    }
}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1272::getSyncWord()
{
    int8_t state = 2;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'getSyncWord'"));
#endif

    if( _modem == FSK )
    {
        state = -1;		// sync word is not available in FSK mode
#if (SX1272_debug_mode > 1)
        Serial.println(F("** FSK mode hasn't sync word **"));
        Serial.println();
#endif
    }
    else
    {
        _syncWord = readRegister(REG_SYNC_WORD);

        state = 0;

#if (SX1272_debug_mode > 1)
        Serial.print(F("## Sync word is "));
        Serial.print(_syncWord, HEX);
        Serial.println(F(" ##"));
        Serial.println();
#endif
    }
    return state;
}

/*
 Function: Sets the sync word in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: sw is sync word value to set in LoRa modem configuration.
*/
int8_t	SX1272::setSyncWord(uint8_t sw)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setSyncWord'"));
#endif

    st0 = readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Notice that FSK hasn't sync word parameter, "));
        Serial.println(F("so you are configuring it in LoRa mode ##"));
#endif
        state = setLORA();
    }
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

    writeRegister(REG_SYNC_WORD, sw);

    delay(100);

    config1 = readRegister(REG_SYNC_WORD);

    if (config1==sw) {
        state=0;
        _syncWord = sw;
#if (SX1272_debug_mode > 1)
        Serial.print(F("## Sync Word "));
        Serial.print(sw, HEX);
        Serial.println(F(" has been successfully set ##"));
        Serial.println();
#endif
    }
    else {
        state=1;
#if (SX1272_debug_mode > 1)
        Serial.println(F("** There has been an error while configuring Sync Word parameter **"));
        Serial.println();
#endif
    }

    writeRegister(REG_OP_MODE,st0);	// Getting back to previous status
    delay(100);
    return state;
}


int8_t SX1272::setSleepMode() {

    int8_t state = 2;
    byte value;

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    // proposed by escyes
    // https://github.com/CongducPham/LowCostLoRaGw/issues/53#issuecomment-289237532
    //
    // inserted to avoid REG_OP_MODE stay = 0x40 (no sleep mode)
    delay(100);
    writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
	
	//delay(50);
	
    value = readRegister(REG_OP_MODE);

	//Serial.print(F("## REG_OP_MODE 0x"));
	//Serial.println(value, HEX);
	
    if (value == LORA_SLEEP_MODE)
        state=0;
    else
        state=1;

    return state;
}

int8_t SX1272::setPowerDBM(uint8_t dbm) {
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

    byte RegPaDacReg=(_board==SX1272Chip)?0x5A:0x4D;

#if (SX1272_debug_mode > 1)
    Serial.println();
    Serial.println(F("Starting 'setPowerDBM'"));
#endif

    st0 = readRegister(REG_OP_MODE);	  // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

	if (dbm == 20) {
		return setPower('X');
	}
	
    if (dbm > 14)
        return state;
      	
	// disable high power output in all other cases
	writeRegister(RegPaDacReg, 0x84);

    if (dbm > 10)
        // set RegOcp for OcpOn and OcpTrim
        // 130mA
        setMaxCurrent(0x10);
    else
        // 100mA
        setMaxCurrent(0x0B);

    if (_board==SX1272Chip) {
        // Pout = -1 + _power[3:0] on RFO
        // Pout = 2 + _power[3:0] on PA_BOOST
        if (_needPABOOST) {
            value = dbm - 2;
            // we set the PA_BOOST pin
            value = value | B10000000;
        }
        else
            value = dbm + 1;

        writeRegister(REG_PA_CONFIG, value);	// Setting output power value
    }
    else {
        // for the SX1276
        uint8_t pmax=15;

        // then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +14dBm)
        // so L=3dBm; H=7dBm; M=15dBm (but should be limited to 14dBm by RFO pin)

        // and Pout = 17-(15-_power[3:0]) if  PaSelect=1 (PA_BOOST pin for +14dBm)
        // so x= 14dBm (PA);
        // when p=='X' for 20dBm, value is 0x0F and RegPaDacReg=0x87 so 20dBm is enabled

        if (_needPABOOST) {
            value = dbm - 17 + 15;
            // we set the PA_BOOST pin
            value = value | B10000000;
        }
        else
            value = dbm - pmax + 15;

        // set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
        value = value | B01110000;

        writeRegister(REG_PA_CONFIG, value);
    }

    _power=value;

    value = readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
#if (SX1272_debug_mode > 1)
        Serial.println(F("## Output power has been successfully set ##"));
        Serial.println();
#endif
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay(100);
    return state;
}

long SX1272::limitToA() {

    // first time we set limitToA?
    // in this design, once you set _limitToA to true
    // it is not possible to set it back to false
    if (_limitToA==false) {
        _startToAcycle=millis();
        _remainingToA=MAX_DUTY_CYCLE_PER_HOUR;
        // we are handling millis() rollover by calculating the end of cycle time
        _endToAcycle=_startToAcycle+DUTYCYCLE_DURATION;
    }

    _limitToA=true;
    return getRemainingToA();
}

long SX1272::getRemainingToA() {

    if (_limitToA==false)
        return MAX_DUTY_CYCLE_PER_HOUR;

    // we compare to the end of cycle so that millis() rollover is taken into account
    // using unsigned long modulo operation
    if ( (millis() > _endToAcycle ) ) {
        _startToAcycle=_endToAcycle;
        _remainingToA=MAX_DUTY_CYCLE_PER_HOUR;
        _endToAcycle=_startToAcycle+DUTYCYCLE_DURATION;

        Serial.println(F("## new cycle for ToA ##"));
        Serial.print(F("cycle begins at "));
        Serial.print(_startToAcycle);
        Serial.print(F(" cycle ends at "));
        Serial.print(_endToAcycle);
        Serial.print(F(" remaining ToA is "));
        Serial.print(_remainingToA);
        Serial.println();
    }

    return _remainingToA;
}

long SX1272::removeToA(uint16_t toa) {

    // first, update _remainingToA
    getRemainingToA();

    if (_limitToA) {
        _remainingToA-=toa;
    }

    return _remainingToA;
}

int8_t SX1272::setFreqHopOn() {
    
    double bw=0.0;
    bw=(_bandwidth==BW_125)?125e3:((_bandwidth==BW_250)?250e3:500e3);
    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << _spreadingFactor);
    double ts = 1 / rs;
    
    return 0;        
}

void SX1272::setCSPin(uint8_t cs) {
	//need to call this function before the ON() function
	_SX1272_SS=cs;
}

SX1272 sx1272 = SX1272();
