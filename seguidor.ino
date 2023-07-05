#include <QTRSensors.h>

#define motorE 3 //velocidade motor A - de 0 a 255

#define dirE 2 //direcao do motor A - HIGH ou LOW

#define dirE2 4 //direcao do motor A - HIGH ou LOW

#define motorD 6 //velocidade motor B - de 0 a 255

#define dirD 5 //direcao do motor B - HIGH ou LOW  

#define dirD2 7 //direcao do motor B - HIGH ou LOW  

int P;

int I;

int D;

float Kp = 0.1; // .13

float Ki = 0.00001;

float Kd = 0.18; //1.08

int maxspeedA = 255;

int maxspeedB = 255;

int velBaseA = 100;

int velBaseB = 100;

int lastError = 0;

QTRSensors qtr;

const uint8_t SensorCount = 6;

uint16_t sensorValues[SensorCount];

void setup()

{
  Serial.begin(9600);

  // configure the sensors

  qtr.setTypeAnalog();

  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  pinMode(dirE, OUTPUT);

  pinMode(dirE2, OUTPUT);

  pinMode(motorE, OUTPUT);

  pinMode(dirD, OUTPUT);

  pinMode(dirD2, OUTPUT);

  pinMode(motorD, OUTPUT);

  digitalWrite(dirD, HIGH);

  digitalWrite(dirD2, LOW);

  digitalWrite(dirE, HIGH);

  digitalWrite(dirE2, LOW);

  //analogWrite(motorE, 255);

  //analogWrite(motorD, 255);

  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  analogWrite(motorE, 0);

  analogWrite(motorD, 0);

  delay(1000);
}

void loop()

{
  PID_control();
}

void PID_control() {

  uint16_t positionLine = qtr.readLineWhite(sensorValues);
  int error = 2500 - positionLine;

  P = error;

  I = error + I;

  D = error - lastError;

  lastError = error;

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = constrain(velBaseA + motorSpeedChange,0,maxspeedA);

  int motorSpeedB = constrain(velBaseB - motorSpeedChange,0,maxspeedB);

  digitalWrite(dirD, HIGH);
  digitalWrite(dirD2, LOW);
  digitalWrite(dirE, HIGH);
  digitalWrite(dirE2, LOW);

  if (motorSpeedA<120&&motorSpeedA>0) motorSpeedA=120;

  if (motorSpeedB<120&&motorSpeedB>0) motorSpeedB=120;

  analogWrite(motorE, motorSpeedA);
  analogWrite(motorD, motorSpeedB);
  
  if(error<=300&&error>=-300){
    motorSpeedA=maxspeedA;
    motorSpeedB=maxspeedB;
  }
  
  analogWrite(motorE, motorSpeedA);
  analogWrite(motorD, motorSpeedB);
}
