// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance

#define portRead 36
float ICAL = 90.90909090909091;
double sampleI, offsetI, filteredI, sumI, sqI;
int SupplyVoltage;
double irms;
float I_RATIO;

double PASSA_BAIXA1 = 5.50;
double PASSA_BAIXA2 = 22.47;

void setup()
{  
  Serial.begin(115200);
  emon1.current(36, ICAL);             // Current: input pin, calibration.
}

void loop()
{
  irms = calcIrms(4000);
  double irmsLib = calcIrmsLib();  // Calculate Irms only

  Serial.print(" Wats: ");
  Serial.print(irms * 127);	       // Apparent power
  Serial.print(" IRMS: ");
  Serial.print(irms);		       // Irms

  Serial.print(" ICAL: ");
  Serial.print(ICAL);

  Serial.print(" I_RATIO: ");
  Serial.print(I_RATIO);

  Serial.print(" Supply: ");
  Serial.print(SupplyVoltage);

  Serial.print(" ADC_COUNTS: ");
  Serial.print(4096);

  Serial.print(" Irms LIB: ");
  Serial.print(irmsLib);

  Serial.print(" Wats LIB: ");
  Serial.println(irmsLib * 127);
}

double calcIrmsLib() {
    double irmsCalc = (emon1.calcIrms(1480) - PASSA_BAIXA2);
    if (irmsCalc < 0) {
      return 0;
    } else { return irmsCalc; }
  }

double calcIrms(unsigned int Number_of_Samples) {
   double irmsCalc;
  
   #if defined emonTxV3
   SupplyVoltage=3300;
   #else 
   SupplyVoltage = readVcc();
   #endif

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(portRead);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
   //  then subtract this - signal is now centered on 0 counts.
   offsetI = (offsetI + (sampleI-offsetI)/1024);
   filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum 
    sumI += sqI;
  }
  
  I_RATIO = ICAL * ((SupplyVoltage/1000.0) / (4096));
  irmsCalc = (I_RATIO * sqrt(sumI / Number_of_Samples)) - 5.50; 

  //Reset accumulators
  sumI = 0;
//--------------------------------------------------------------------------------------       
  if (irmsCalc < 0) { return 0; }
  else {  return irmsCalc; }
}

long readVcc() {
  long result;
  
  //not used on emonTx V3 - as Vcc is always 3.3V - eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #endif
  
  #if defined(__AVR__) 
  delay(2);                                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
 #elif defined(__arm__)
  return (3300);                                  //Arduino Due
 #else 
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
 #endif
}

// Wats: 2896.58 IRMS: 22.81
// Wats: 2866.66 IRMS: 22.57
// Wats: 2729.61 IRMS: 21.49
// Wats: 2733.16 IRMS: 21.52
// Wats: 2770.35 IRMS: 21.81
// Wats: 2910.66 IRMS: 22.92
// Wats: 2888.11 IRMS: 22.74
