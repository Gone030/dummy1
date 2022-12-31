#include <Arduino.h>

#include "Motor.h"
#include "Calculates.h"
#include "PID.h"

#define motor_pin_F 2
#define motor_pin_R 3
#define motor_pin_F_EN 5
#define motor_pin_R_EN 6
#define servo_pin 4

#define pA 10
#define pB 11
#define pZ 12

#define Count_per_Revolution 12000

// Calculates calc(int 최대rpm, float 바퀴치수, float 바퀴x거리, float 바퀴y거리)
Calculates calc(18000, 7.30 ,18.3);

volatile signed long cnt_ = 0;
volatile signed char dir_ = 1;
float vel_ = 0.0;
int temp_ = 0;
bool flag_ = false;
float pi = 3.14;

unsigned long prev_count_time = 0;
unsigned long prev_count_tick = 0;


#define kp 20.0
#define ki 5.0
#define kd 0.0
int min_val = -255;
int max_val = 255;
PID pid(min_val, max_val, kp, ki, kd);

control motor(motor_pin_F, motor_pin_R, motor_pin_F_EN, motor_pin_R_EN, servo_pin);

void encoderCount()
{
  dir_ = (digitalRead(pB) == HIGH) ? -1 : 1;
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
float getRPM() // 엔코더 rpm 계산
{
  long current_tick = returnCount();
  unsigned long current_time = micros();
  unsigned long dt = current_time - prev_count_time;

  double dtm = (double)dt / 60000000;
  double delta_tick = current_tick - prev_count_tick;

  prev_count_time = current_time;
  prev_count_tick = current_tick;

  return (delta_tick / Count_per_Revolution) / dtm;
}



void setup()
{
  Serial.begin(115200);
  // Wire.begin();
  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);
}

void loop()
{

  if (Serial.available())
  {
    temp_ = Serial.parseInt();
    flag_ = true;
  }
  if (flag_)
  {
    switch (temp_)
    {
    case 1:
      vel_ += 1.0;
      flag_ = false;
      break;
    case 2:
      vel_ = vel_ - 1.0;
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
    case 4:
      while(1)
      {}
    }
  }
  motor.steer(vel_ * 180/pi);
  // motor.run(vel_);
  // float calcRPM = calc.CalculateRpm(vel_); //모터에 적용되는 값을 rpm으로 변환
  // float ecdRPM = getRPM(); //엔코더로 얻은 실제 rpm
  // double pidvel = pidcompute(calcRPM, ecdRPM);
  // motor.run(pidvel);
  // Serial.println(pidvel);
  // Serial.print(calcRPM);
  // Serial.print(',');
  // Serial.println(ecdRPM);
  // Serial.print(',');
  // Serial.println(pidvel);
}
