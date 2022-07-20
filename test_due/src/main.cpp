/*#include <Arduino.h>

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
*/

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}