#include <Arduino.h>

#include "Motor.h"

#define motor_pin_F 2
#define motor_pin_R 3
#define motor_pin_F_EN 5
#define motor_pin_R_EN 6
#define servo_pin 4

// #define pA 10
// #define pB 11
// #define pZ 12

// #define Count_per_Revolution 12000

// // Calculates calc(int 최대rpm, float 바퀴치수, float 바퀴x거리, float 바퀴y거리)
// volatile signed long cnt_ = 0;
// volatile signed char dir_ = 1;
float vel_ = 0.0;
int temp_ = 0;
bool flag_ = false;
float pi = 3.14;

control motor(motor_pin_F, motor_pin_R, motor_pin_F_EN, motor_pin_R_EN, servo_pin);


void setup()
{
  Serial.begin(115200);
  // Wire.begin();
  // pinMode(2, OUTPUT); //전진
  // pinMode(3, OUTPUT); //후진
  // pinMode(5, OUTPUT); //전진EN
  // pinMode(6, OUTPUT); //후진EN
  // digitalWrite(9,HIGH);
  // digitalWrite(8,HIGH);

  // pinMode(13, OUTPUT);
}

void loop()
{
  // for(int i = 0 ; i<256 ; i++)
  // {
  //   digitalWrite(6,LOW);
  //   analogWrite(5, i);
  //   delay(100);
  // }
  // for(int i = 1 ; i < 3 ; i++)
  // {
  //   digitalWrite(13, HIGH);
  //   delay(300);
  //   digitalWrite(13,LOW);
  //   delay(300);
  // }
  // for(int i = 255 ; i >= 0 ; i--)
  // {
  //   digitalWrite(6,LOW);
  //   analogWrite(5, i);
  //   delay(100);
  // }
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
      vel_ += 10.0;
      flag_ = false;
      Serial.println(vel_);
      break;
    case 2:
      vel_ = vel_ - 10.0;
      flag_ = false;
      Serial.println(vel_);
      break;
    case 3:
      // Serial.print("no map : ");
      Serial.println("vel reset");
      vel_ = 0;
      // Serial.print("map : ");
      flag_ = false;
      break;
    case 4:{
      int temp = motor.velreturn();
      Serial.println(temp);
      flag_ = false;
      break;}
    case 5:
      while(1)
      {}
    }
  }
  motor.run(vel_);
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
  // for(double i = 0 ; i <256 ; i++)
  // {
  //   motor.run(i);
  //   delay(100);
  // }
}
