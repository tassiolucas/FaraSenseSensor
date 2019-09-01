// Programa: FaraSenseSensor
// Autor: Tassio Lucas
// Descricao & Documentacao: No README.md

// Bibliotecas do Sistema
#include "esp32-hal-adc.h"
#include "Arduino.h"
#include "ACS712.h"
// Bibliotecas da Captação de Dados
#include "EmonLib.h"
// Bibliotecas de Comunicação
#include <WiFi.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h> 
// Bibliotecas do BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Define config da plataforma Arduino
#define SERIAL_SPEED  115200
#define TX_PIN  1
bool TX_IS_ON = false;

#if TX_IS_ON
#undef STATUS
#define STATUS 1

#else
#undef STATUS
#define STATUS 2
#endif

#define LEDC_TIMER_13_BIT  13 // Tempo LED
#define LEDC_BASE_FREQ  5000 // Tempo FREQ
boolean DEBUG_MODE = false;
boolean BLE_DEBUG_MODE = false;
const int TRIGGER_PIN = 0;

// Instancias das bibliotecas utilizadas
HTTPClient http;
EnergyMonitor emon1; 
WiFiManager wifiManager;

// Configuracoes do sensor de corrente
int ID_SENSOR = 1;
#define ACS_MPY  15.41  
float I_RATIO;
float ICAL = 9.090909090909090;
const int portRead = 35;
double sampleI, offsetI, filteredI, sumI, sqI;
int SupplyVoltage;
double irms;
double totalAmper;
int countTotal;

// Configurações dos Loops
unsigned long loops = 0;
long lastMillis = 0;
// Led alert
boolean blinkWaiting = false;

// Configuracoes de Brilho do LED de Status
#define UP 0
#define DOWN 1
// Constantes para minimo e maximo do PWM
const int minPWM = 0;
const int maxPWM = 255;
// Estado variavel para direcao do face
byte fadeDirection = UP;
// Valor de fade global
// mas seja maior que byte e assinado, para rollover
int fadeValue = 0;
// Suavidade do desaparecer do fade
byte fadeIncrement = 5;
// Em millis() timing Variable, just for fading
unsigned long previousFadeMillis;
// How fast to increment?
int fadeInterval = 50;

// Configuracoes de funcionamento da API AWS
// END POINT POST AWS
#define apiUrlPOST "https://p4b2zvd5pi.execute-api.us-east-1.amazonaws.com/dev/current_sensor"
// #define batteryUrlPOST "https://p4b2zvd5pi.execute-api.us-east-1.amazonaws.com/dev/current_sensor/battery"
String dataPost;

// Configuracoes do Bluetooth
BLEServer* bleServer = NULL;
BLECharacteristic *bleCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
#define SERVICE_UUID "129fecfc-3f58-11e9-b210-d663bd873d93" // Servico UART de UUID
#define CHARACTERISTIC_UUID "129fefae-3f58-11e9-b210-d663bd873d93"

class BleServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* bleServer) {
    deviceConnected = true;
    
    Serial.println("BLE Conectado!");  
  };

  void onDisconnect(BLEServer* bleServer) {
    deviceConnected = false;
    Serial.println("BLE Desconectado!");    
  }
};

// Configurando a porta analogica para escrita de 0 atual o valor maximo de 4096
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup() {
  // DEBUG MODE ? 
  TX_IS_ON = digitalRead(TX_PIN);
  
  Serial.begin(SERIAL_SPEED);
  Serial.println(digitalRead(TX_PIN));
  Serial.println(TX_IS_ON);

  // Definição do led de status
  ledcSetup(STATUS, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(STATUS, STATUS);
  ledcAnalogWrite(STATUS, 255); 

  pinMode(portRead, INPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  adcAttachPin(portRead);
  
  analogReadResolution(10);
  analogSetAttenuation(ADC_6db); // Alterando resolucao da porta de leitura

  emon1.current(portRead, ACS_MPY); 

  wifiManager.setAPCallback(configModeCallback);

  if (!wifiManager.autoConnect("FARASENSE")) {
    delay(1000);
    if (DEBUG_MODE) {
      Serial.println("FALHA NA CONEXAO, TEMPO MAXIMO ATINGIDO.");
    }
    // Redefine e tenta novamente
    ESP.restart();
  }

  totalAmper = 0;
  countTotal = 0;

    // Configuração inicial Bluetooth
  BLEDevice::init("FaraSense Sensor 1"); // Cria dispositivo BLE, dando um nome
  bleServer = BLEDevice::createServer(); // Configura dispositivo como servidor BLE
  bleServer -> setCallbacks(new BleServerCallbacks());
  BLEService *bleService = bleServer -> createService(SERVICE_UUID);
  bleCharacteristic = bleService -> createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_WRITE
                                          );
  bleCharacteristic ->addDescriptor(new BLE2902());
  bleService -> start(); 
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  if (DEBUG_MODE) {
    Serial.println("BLE pronto!");
  }
  delay(1000);
  ledcAnalogWrite(STATUS, 0);
}

// Loop do programa //
void loop()
{
    unsigned long currentMillis = millis();
    loops++;
    
    handleButton();
    
    if (currentMillis > 5000) {

      irms = emon1.calcIrms(1996);
      
      // irms = calcIrms(4000); // Deprecado

      // Calcula IRMS somente (Biblioteca padrão do sensor)
      // double irmsLib = calcIrmsLib();     
      // debugValuesSensor(irms, irmsLib);

      if (irms != 0) {
        if (deviceConnected) {   
          char bleValue[10];
          sprintf(bleValue, "%4.4f", irms);

          if (BLE_DEBUG_MODE && DEBUG_MODE) {
            Serial.print("BLE VALUE: ");
            Serial.print(bleValue);
          }
                                      
          bleCharacteristic -> setValue(bleValue); 
          bleCharacteristic -> notify();
        }

        if (!deviceConnected && oldDeviceConnected) {
          delay(500);
          bleServer->startAdvertising(); // restart advertising
          if (DEBUG_MODE) {
            Serial.println("start advertising");
          }
          oldDeviceConnected = deviceConnected;
        }

        if (deviceConnected && !oldDeviceConnected) {
          oldDeviceConnected = deviceConnected;
        }
      }

      if (DEBUG_MODE) {
        Serial.print("Porta ");
        Serial.print(portRead);
        Serial.print(": ");
        Serial.print(analogRead(portRead));
        Serial.print("  - Ultima medida (amper): ");
        Serial.println(irms);
        delay(2500);
      }
            
      if (irms != 0) {    
        
        totalAmper = totalAmper + irms;
        countTotal++;
        
        if (currentMillis - lastMillis > 60000) {
          double dataSend = (totalAmper / countTotal);
             
          apiSendData(dataSend);
          
          if (DEBUG_MODE) {
              Serial.print(" | Contagem: ");
              Serial.print(countTotal);
              Serial.print(" | Total: ");
              Serial.println(totalAmper);
          }
  
          countTotal = 0;
          totalAmper = 0;
          dataSend = 0;
          
          lastMillis = currentMillis;
          loops = 0;
        }
        doTheFade(currentMillis);
      } else {
        if (DEBUG_MODE) {      
          Serial.println("Aguardando leitura dos sensores...");
        }
        doBlinkWaiting();
        delay(1000);
      }
    } else {
      irms = calcIrms(4000);
    }
}

// FUNCOES DE FUNCIONAMENTO
void handleButton(){
  int debounce = 50;
  if (digitalRead(TRIGGER_PIN) == LOW){
    delay(debounce);
    if(digitalRead(TRIGGER_PIN) == LOW){
      WiFi.disconnect(false,true);
      delay(1000);
      ESP.restart();
    }
  }
}

// FUNCOES DE STATUS
void doTheFade(unsigned long thisMillis) {
  if (thisMillis - previousFadeMillis >= fadeInterval) {
    if (fadeDirection == UP) {
      fadeValue = fadeValue + fadeIncrement;
      if (fadeValue >= maxPWM) {
        fadeValue = maxPWM;
        fadeDirection = DOWN;
      }
    } else {
      fadeValue = fadeValue - fadeIncrement;
      if (fadeValue <= minPWM) {
        fadeValue = minPWM;
        fadeDirection = UP;
      }
    }
    ledcAnalogWrite(STATUS, fadeValue);
    previousFadeMillis = thisMillis;
  }
}

void doBlinkWaiting() {
  if (blinkWaiting) {
    ledcAnalogWrite(STATUS, 0);
    blinkWaiting = false;
  } else {
    ledcAnalogWrite(STATUS, 255);
    blinkWaiting = true;
  }
}

void doBlinkError() {
  for (int i = 0; i < 3; i++) {
    if (blinkWaiting) {
      ledcAnalogWrite(STATUS, 0);
      blinkWaiting = false;
    } else {
      ledcAnalogWrite(STATUS, 255);
      blinkWaiting = true;
    }
  }
}

// FUNCOES DE COMUNICACAO WIFI
void apiSendData(double amper) {
  if (WiFi.status() == WL_CONNECTED) {
    http.begin(apiUrlPOST);
    http.addHeader("Content-Type", "text/plain");

    String dataPost = (String) "{\"id\":" + ID_SENSOR + ", \"amper\":" + amper + ", \"power\":" + (amper * 127) + "}";

    if (DEBUG_MODE) {
      Serial.print("DATAPOST: ");
      Serial.println(dataPost);
    }
    
    int httpResponseCode = http.POST(dataPost);

    if (httpResponseCode > 0) {
      String response = http.getString();
      if (DEBUG_MODE) {
        Serial.print("POST: ");
        Serial.print(httpResponseCode);
        Serial.print("  ");
        Serial.println(response);
      }
    } else {
      if (DEBUG_MODE) {
        Serial.print("ERRO AO ENVIAR O POST...");
        Serial.println(httpResponseCode);
      }
      forceNetworkRestart();
    }

    http.end();
  } else {
    if (DEBUG_MODE) {
      Serial.println("ERROR NA CONEXAO WIFI...");
    }
    doBlinkError();
  }
}

void  configModeCallback (WiFiManager * myWiFiManager) {
  if (DEBUG_MODE) {
    Serial.println ("Em modo de configuracao..." );
    Serial.println (WiFi. softAPIP ());
    Serial.println (myWiFiManager-> getConfigPortalSSID ());
  }
}

void forceNetworkRestart() {
    wifiManager.autoConnect("FARASENSE");
    delay(1000);
    if (DEBUG_MODE) {
      Serial.println("FALHA NO ENVIO PARA API, REINICIADO A CONEXAO...");
    }
    ESP.restart();
}

// FUNCOES DE CALCULO DO AFERIMENTO DE CORRENTE (deprecado)
double calcIrms(unsigned int Number_of_Samples) { 
  double irmsCalc;

#if defined emonTxV3
  SupplyVoltage = 3300;
#else
  SupplyVoltage = readVcc();
#endif

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(portRead);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;

    // delay(0.05);
  }

  I_RATIO = ICAL * ((SupplyVoltage / 1000.0) / (4096));
  irmsCalc = (I_RATIO * sqrt(sumI / Number_of_Samples));

  //Reseta acumuladores
  sumI = 0;
  //--------------------------------------------------------------------------------------
  // VALOR PURO
  //  return irmsCalc;
  //--------------------------------------------------------------------------------------
  // VALOR TRATADO
  // Caso a corente aparente seja menor que o valor que o sensor detecta, retorna 0
  if (irmsCalc < 0.50) {
    return 0;
  }
  else {
    return irmsCalc;
  }
}

// CALIBRACAO VCC (deprecado)
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
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
#elif defined(__arm__)
  return (3300);                                  //Arduino Due
#else
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
#endif
}

// FUNCOES DEBUG
void debugValuesSensor(double irms, double irmsLib) {
  if (DEBUG_MODE) {
    Serial.print(" Entrada: ");
    Serial.print(portRead);
    Serial.print(" R:");
    Serial.print(analogRead(portRead));
  
    Serial.print(" IRMS: ");
    Serial.print(irms);          // Irms
  
    Serial.print(" Wats: ");
    Serial.print(irms * 127);         // Apparent power
  
    Serial.print(" Irms LIB: ");
    Serial.print(irmsLib);
    Serial.print(" Wats LIB: ");
    Serial.println(irmsLib * 127);
  }
}
