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

// Calculates calc(int 최대rpm, float 최대rpm비, float 바퀴치수, float 바퀴x거리, float 바퀴y거리)
Calculates calc(21900, 10.5, 15.0);

volatile signed long cnt_ = 0;
volatile signed char dir_ = 1;
float vel_ = 0.0;
int temp_ = 0;
bool flag_ = false;

unsigned long prev_count_time = 0;
unsigned long prev_count_tick = 0;

double integral; // pid variable
double derivative;
double prev_error;
float kp = 0; //모터 교체 완료 후 진행
float ki = 0;
float kd = 0;
int min_val = 0;
int max_val = 1024;

control motor(pwm_pin, motor_pin_a, motor_pin_b, servo_pin);

void encoderCount()
{
  dir_ = (digitalRead(pB) == HIGH) ? 1 : -1;
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

double pidcompute(float setpoint, float measured_value) // (목표 rpm , 실제 rpm)
{
  double error;
  double pid;

  error = setpoint - measured_value;
  integral += error;
  derivative = error - prev_error;

  if (setpoint == 0 && error == 0)
  {
    integral = 0;
    derivative = 0;
  }
  pid = (kp * error) + (ki * integral) + (kd * derivative);
  prev_error = error;

  return constrain(pid, min_val, max_val);
}

void setup()
{
  Serial.begin(115200);
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
      vel_ += 10.0;
      flag_ = false;
      break;
    case 2:
      vel_ -= 10.0;
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
  // motor.run(vel_);
  float calcRPM = calc.CalculateRpm(vel_); //모터에 적용되는 값을 rpm으로 변환
  float ecdRPM = getRPM(); //엔코더로 얻은 실제 rpm
  double pidvel = pidcompute(calcRPM, ecdRPM);
  motor.run(pidvel);
  // Serial.println(pidvel);
  Serial.println(calcRPM);
  Serial.println(ecdRPM);
}
