#include "esp32-hal-adc.h"
#include "Arduino.h"
#include "ACS712.h"
#include "EmonLib.h" 

#define SERIAL_SPEED 115200

const int portRead = 36;

EnergyMonitor emon1; 

#define   SAMPLING_TIME     0.0001668649    // intervalo de amostragem 166,86us
#define   LINE_FREQUENCY    60              // frequencia 60Hz Brasil

#define   VOLTAGE_AC        127.00          // 127 Volts
#define   ACS_MPY           15.41           // ganho/calibracao da corrente

double Irms = 0;
double current;

void setup() { 
 Serial.begin(115200);
 
 analogReadResolution(10);
 analogSetAttenuation(ADC_6db); // Alterando resolucao da porta de leitura
 
 pinMode(portRead, INPUT);

 emon1.current(portRead, ACS_MPY); 
}

void  loop () {
 getACS712();
 delay(2500);
}

void getACS712() {  // for AC
  Irms = emon1.calcIrms(1996);

  Serial.print("Port: ");
  Serial.print(analogRead(portRead));
  Serial.print("\tIrms: ");
  Serial.println(Irms);
}
