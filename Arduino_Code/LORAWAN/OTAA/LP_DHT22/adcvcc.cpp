#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h> 
#include "Arduino.h"


volatile uint8_t  _adc_irq_cnt;


// add this to the INO of you project.
///* ======================================================================
//Function: ADC_vect
//Purpose : IRQ Handler for ADC 
//Input   : - 
//Output  : - 
//Comments: used for measuring 8 samples low power mode, ADC is then in 
//          free running mode for 8 samples
//====================================================================== */
//ISR(ADC_vect)  
//{
//  // Increment ADC counter
//  _adc_irq_cnt++;
//}


/* ======================================================================
Function: readADCLowNoise
Purpose : Read Analog Value with reducing noise
Input   : true return the average value, false return only the sum
Output  : average value read
Comments: hard coded to read 8 samples each time
          AD MUX Channel and ADC must have been set before this call
====================================================================== */
uint16_t readADCLowNoise(boolean average)
{
  uint8_t low, high;
  uint16_t sum = 0;
  
  // Start 1st Conversion, but ignore it, can be hazardous
  ADCSRA |= _BV(ADSC); 
  
  // wait for first dummy conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Init our measure counter
  _adc_irq_cnt = 0;

  // We want to have a interrupt when the conversion is done
  ADCSRA |= _BV( ADIE );

  // Loop thru samples
  // 8 samples (we don't take the 1st one)
  do {
    // Enable Noise Reduction Sleep Mode
    set_sleep_mode( SLEEP_MODE_ADC );
    sleep_enable();

    // Wait until conversion is finished 
    do {
      // enabled IRQ before sleeping
      sei();
      sleep_cpu();
      cli();
    }
    // Check is done with interrupts disabled to avoid a race condition
    while (bit_is_set(ADCSRA,ADSC));

    // No more sleeping
    sleep_disable();
    sei();
    
    // read low first
    low  = ADCL;
    high = ADCH;
    
    // Sum the total
    sum += ((high << 8) | low);
  }
  while (_adc_irq_cnt<8);
  
  // No more interrupts needed for this
  // we finished the job
  ADCSRA &= ~ _BV( ADIE );
  
  // Return the average divided by 8 (8 samples)
  return ( average ? sum >> 3 : sum );
}

/* ======================================================================
Function: readVcc
Purpose : Read and Calculate V powered, the Voltage on Arduino VCC pin
Input   : -
Output  : value readed
Comments: ADC Channel input is modified
====================================================================== */
uint16_t readVcc() 
{
  uint16_t value, _vcc; 

  // Enable ADC (just in case going out of low power)
  power_adc_enable();
  ADCSRA |= _BV(ADEN)  ;

  // Read 1.1V reference against AVcc
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

  // Take care, changing reference from VCC to 1.1V bandgap can take some time, this is due
  // to the fact that the capacitor on aref pin need to discharge
  // or to charge when we're just leaving power down mode
  // power down does not hurt and 15ms strong enough for ADC setup
  delay(15);  

  // read value
  value = readADCLowNoise(true);

  // Vcc reference in millivolts
  _vcc =  ( 1023L * 1100L /*config.aRef*/) / value ; 
  
  // Operating range of ATMega
  if (_vcc < 1800 ) _vcc = 1800 ;
  if (_vcc > 5500 ) _vcc = 5500 ;
    
  // Vcc in millivolts
  return ( _vcc ); 
}
