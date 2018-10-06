
#define B0    34
#define B1    35
#define B2    32
#define B3    33
#define B4    25
#define B5    26
#define B6    27
#define B7    14
#define B8    12
#define B9    13

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void ledDigitalConfig() {
  //pinMode(B0, OUTPUT);
  //pinMode(B1, OUTPUT);
  ledcSetup(B0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcSetup(B1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(B0, B0);
  ledcAttachPin(B1, B1);
  pinMode(B2, OUTPUT);
  pinMode(B3, OUTPUT);
  pinMode(B4, OUTPUT);
  pinMode(B5, OUTPUT);
  pinMode(B6, OUTPUT);
  pinMode(B7, OUTPUT);
  pinMode(B8, OUTPUT);
  pinMode(B9, OUTPUT);
}

void testeLed() {
  if (!blinkWaiting) {
    ledcAnalogWrite(B0, 255);
    ledcAnalogWrite(B1, 255);
    digitalWrite(B2, HIGH);
    digitalWrite(B3, HIGH);
    digitalWrite(B4, HIGH);
    digitalWrite(B5, HIGH);
    digitalWrite(B6, HIGH);
    digitalWrite(B7, HIGH);
    digitalWrite(B8, HIGH);
    digitalWrite(B9, HIGH);
    blinkWaiting = false;
  } else {
    ledcAnalogWrite(B0, 0);
    ledcAnalogWrite(B1, 0);
    digitalWrite(B2, LOW);
    digitalWrite(B3, LOW);
    digitalWrite(B4, LOW);
    digitalWrite(B5, LOW);
    digitalWrite(B6, LOW);
    digitalWrite(B7, LOW);
    digitalWrite(B8, LOW);
    digitalWrite(B9, LOW);
    blinkWaiting = true;
  }
}
