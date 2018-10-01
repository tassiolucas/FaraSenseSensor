#include "Arduino.h"

#include "EmonLib.h"                   // Inclui EmonLib (Biblioteca padrão do cálculo RMS)   
#include <WiFi.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>

EnergyMonitor emon1; // Cria uma instância
WiFiManager wifiManager;
boolean shouldSaveConfig;

#define SERIAL_SPEED 115200
int ID_SENSOR = 1;

#define LEDC_CHANNEL_0     0
#define LEDC_TIMER_13_BIT  13
#define LEDC_BASE_FREQ     5000
#define LED_PIN            2

int brightness = 0;
int fadeAmount = 5;
unsigned long times;

#define portRead 34
float ICAL = 9.090909090909090;
double sampleI, offsetI, filteredI, sumI, sqI;
int SupplyVoltage;
double irms;
float I_RATIO;

int freq = 5000;
int ledChannel = 0;
int resolution = 8;

// END POINT POST AWS
#define apiUrlPOST "https://p4b2zvd5pi.execute-api.us-east-1.amazonaws.com/dev/current_sensor"
HTTPClient http;

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup()
{ 
  Serial.begin(SERIAL_SPEED);
   ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
   ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
   ledcAnalogWrite(LEDC_CHANNEL_0, 255);  
    emon1.current(portRead, ICAL);
    wifiManager.setAPCallback(configModeCallback);
  
  if (!wifiManager.autoConnect("FARASENSE")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }
  
  delay(50);
   ledcAnalogWrite(LEDC_CHANNEL_0, 0);
}

void loop()
{
  if(millis() > 5000) {
      irms = calcIrms(4000);
      double irmsLib = calcIrmsLib();  // Calculate Irms only
      //debugValuesSensor(irms, irmsLib);
      if (irms != 0) {
        apiSendData(irms);
      } else { Serial.println("Aguardando sensores...");  }
  } else {
    irms = calcIrms(4000);
  }
  
  delay(1000);
}

void apiSendData(double amper) {
  ledcAnalogWrite(LEDC_CHANNEL_0, 255);
  if(WiFi.status() == WL_CONNECTED) {
    http.begin(apiUrlPOST);
    http.addHeader("Content-Type", "text/plain");

    String dataPost = (String) "{\"id\":" + ID_SENSOR + ", \"amper\":" + amper + ", \"milliamper\":" + (amper*1000) + ", \"power\":" + (amper * 127) + "}";
    
    int httpResponseCode = http.POST(dataPost);
    
    if(httpResponseCode>0){
      String response = http.getString();
      Serial.println(httpResponseCode); 
      Serial.println(response);          
    }else{
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
    }
    
    http.end();
  } else {
    Serial.print("Error in WiFi connection");
    bool shouldSaveConfig = false ;
  }
  ledcAnalogWrite(LEDC_CHANNEL_0, 255);
}

double calcIrmsLib() {
    double irmsCalc = (emon1.calcIrms(1480));
    if (irmsCalc < 0) {
      return 0;
    } else { return irmsCalc; }
}

void debugValuesSensor(double irms, double irmsLib) {

//  Serial.print("ICAL: ");
//  Serial.print(ICAL);
//
//  Serial.print(" I_RATIO: ");
//  Serial.print(I_RATIO);
//
//  Serial.print(" Supply: ");
//  Serial.print(SupplyVoltage);
//
//  Serial.print(" ADC_COUNTS: ");
//  Serial.print(4096);

  String dataPost = (String) "{\"id\":" + ID_SENSOR + ", \"amper\":" + irms + ", \"milliamper\":" + (irms*1000) + ", \"power\":" + (irms * 127) + "}";
  Serial.print(dataPost);
  
  Serial.print(" Entrada: ");
  Serial.print(portRead);
  Serial.print(" R:");
  Serial.print(analogRead(portRead));
  
  Serial.print(" IRMS: ");
  Serial.print(irms);          // Irms

  Serial.print(" MILLIAMPER: ");
  Serial.print(irms * 1000);          // Irms
  
  Serial.print(" Wats: ");
  Serial.print(irms * 127);         // Apparent power

  Serial.print(" Irms LIB: ");
  Serial.print(irmsLib);
  Serial.print(" Wats LIB: ");
  Serial.println(irmsLib * 127);
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

    delay(0.05);
  }
  
  I_RATIO = ICAL * ((SupplyVoltage/1000.0) / (4096));
  irmsCalc = (I_RATIO * sqrt(sumI / Number_of_Samples)); 

//Reseta acumuladores
  sumI = 0;
//--------------------------------------------------------------------------------------  
// VALOR PURO
//  return irmsCalc;
//-------------------------------------------------------------------------------------- 
// VALOR TRATADO
// Caso a corente aparente seja menor que o valor mínimo que o sensor lê, retorna 0
  if (irmsCalc < 0.50) { return 0; }  
  else {  return irmsCalc; }
}

void blinkLedFade(int fadeAmount, int brightness) {
  // set the brightness on LEDC channel 0
  ledcAnalogWrite(LEDC_CHANNEL_0, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}

// callback notificando-nos da necessidade de salvar config 
void  saveConfigCallback () {
  Serial. println ( " Deve salvar config " );
  shouldSaveConfig = true ;
}

void  configModeCallback (WiFiManager * myWiFiManager) {
  Serial. println ("Modo de configuração entrado: " );
  Serial. println (WiFi. softAPIP ());

  Serial. println (myWiFiManager-> getConfigPortalSSID ());
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
