
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Configuracoes do sensor de corrente
int ID_SENSOR = 1;
const int portRead = 36;
float ICAL = 9.090909090909090;
double sampleI, offsetI, filteredI, sumI, sqI;
int SupplyVoltage;
double irms;
float I_RATIO;
double totalAmper;
int countTotal;

unsigned long loops = 0;
long lastMillis = 0;

// Configuracoes do Bluetooth
BLECharacteristic *bleCharacteristic;
BLEService *bleService;
bool deviceConnected = false;
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

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  // Configuração inicial Bluetooth
  BLEDevice::init("FaraSense Sensor 1"); // Cria dispositivo BLE, dando um nome
  BLEServer *bleServer = BLEDevice::createServer(); // Configura dispositivo como servidor BLE
  bleServer -> setCallbacks(new BleServerCallbacks());
  BLEService *bleService = bleServer -> createService(SERVICE_UUID);
  BLECharacteristic *bleCharacteristic = bleService -> createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_NOTIFY
                                          );
  bleCharacteristic -> addDescriptor(new BLE2902());
  bleService -> start(); 
  bleServer -> getAdvertising() -> start();
 
}

void loop() {

  irms = calcIrms(4000);

  if (irms != 0) {
    if (deviceConnected) {   
      char bleValue[8];
      dtostrf(irms, 1, 2, bleValue);
  
      Serial.print("BLE VALUE: ");
      Serial.println(bleValue);
                                  
      bleCharacteristic -> setValue(bleValue); 
      bleCharacteristic -> notify();
    }
  }

  delay(100);
}

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
