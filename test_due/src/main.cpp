#include <Arduino.h>


#include "Motor.h"
#include "Calculates.h"

#define pwm_pin 7
#define motor_pin_a 2
#define motor_pin_b 3
#define servo_pin 6

#define pA 10
#define pB 11
#define pZ 12

#define Count_per_Revolution 12000

volatile signed long cnt_ = 0;
volatile signed char dir_ = 1;
int vel_ = 0;
int temp_ = 0;
bool flag_ = false;

unsigned long prev_count_time = 0;
unsigned long prev_count_tick = 0;

// control motor(pwm_pin, motor_pin_a, motor_pin_b, servo_pin);

void encoderCount()
{
  dir_ = (digitalRead(pB) == HIGH)? 1:-1;
  cnt_ += dir_;
}
void encoderReset()
{
  cnt_ = 0;
}
long returnCount()
{
  return cnt_;
}
float getRPM()
{
  long current_tick = returnCount();
  unsigned long current_time = micros();
  unsigned long dt = current_time - prev_count_time;

  double dtm = (double)dt / 60000000;
  double delta_tick = current_tick - prev_count_tick;

  prev_count_time = current_time;
  prev_count_tick = current_tick;

  return (delta_tick / Count_per_Revolution) / dtm ;
}


void setup() {
  Serial.begin(115200);
  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB,INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);
      pinMode(motor_pin_a, OUTPUT);
    pinMode(motor_pin_b, OUTPUT);
    pinMode(7, OUTPUT); 
}

void loop() {
  
  if (Serial.available())
  {
    temp_ = Serial.parseInt();
    flag_ = true;
  }
  if(flag_)
  {
    switch (temp_) 
    {
      case 1:
        vel_ += 10;
        flag_ = false;
      break;
      case 2:
        vel_ -= 10;
        flag_ = false;
      break;
      case 3:
        Serial.print("encoder : ");
        Serial.println(cnt_);
        Serial.print("no map : ");
        Serial.println(vel_);
        Serial.print("map : ");
        Serial.println(map(vel_, 0, 1023, 0, 4095));
        flag_ = false;
      break;
    }
  }
  //   Serial.print("Motor : ");
  //   Serial.println(vel);
  //   motor.run(vel);
  digitalWrite(motor_pin_a, HIGH);
  digitalWrite(motor_pin_b, LOW );
  analogWriteResolution(12);
  analogWrite(7, map(vel_, 0, 1023, 0, 4095));
}