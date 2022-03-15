#include <Arduino.h>

// #include "Motor.h"

// #define mot_en_pin 8
// #define mot_in1_pin 2
// #define mot_in2_pin 3

// control motor_control(mot_en_pin, mot_in1_pin, mot_in2_pin);

int pA = 21; //interrupt pin 2
int pB = 20; //interrupt pin 3
int pZ = 19; //interrupt pin 4

volatile signed long cnt = 0;
volatile signed char dir = 1;


void encoderCount()
{
  dir = (digitalRead(pB) == HIGH)? 1:-1;
  cnt += dir;
}
void encoderReset()
{
  cnt = 0;
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pA), encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pZ), encoderReset, FALLING);
}

void loop()
{ 
  Serial.print(micros());
  Serial.print(',');
  Serial.println(cnt);
  // motor_control.run(180);
}


