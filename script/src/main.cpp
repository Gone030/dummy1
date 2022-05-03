#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

#include "Calculates.h"
#include "Motor.h"

rcl_subscription_t twist_sub;
rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;
rcl_timer_t control_timer;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;

// #define max_rpm 330
// #define max_rpm_ratio
//temperary value
int max_rpm = 0; 
float wheel_diameter = 0.07;
float wheel_distence_x = 0.2;

double integral; // pid variable
double derivative;
double prev_error;
float kp = 0;
float ki = 0;
float kd = 0;
int min_val = 0;
int max_val = 1024;


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

unsigned long prev_count_time = 0;
unsigned long prev_count_tick = 0;

control motor(pwm_pin, motor_pin_a, motor_pin_b, servo_pin);
Calculates calculates(max_rpm, wheel_diameter, wheel_distence_x);

unsigned long prev_cmdvel_time = 0;



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
}

void encoderCount()
{
  dir_ = (digitalRead(pB) == HIGH)? -1: 1; //reverse
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
  unsigned long dt = current_time - prev_count_tick;

  double dtm = (double)dt / 60000000;
  double delta_tick = current_tick - prev_count_tick;

  prev_count_tick = current_tick;
  prev_count_time = current_time;

  return (delta_tick / Count_per_Revolution) / dtm; 
}

double pidcompute(float setpoint, float measured_value)
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

void subcmdvel_callback(const void *msgin)
{
  digitalWrite(13, !digitalRead(13));
  prev_cmdvel_time = millis();
}

void controlcallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if(timer != NULL)
  {
    move();

  }
}

void move()
{
  if((millis() - prev_cmdvel_time) >= 200)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
  }
  calculates.dcmotor_rpm = calculates.CalculateRpm(twist_msg.linear.x);
  float calc_dc_rpm = calculates.dcmotor_rpm;
  float ecd_rpm = getRPM();
  double pidvel = pidcompute(calc_dc_rpm, ecd_rpm);
  float req_anguler_vel_z = twist_msg.angular.z; 
  motor.run(pidvel);
  float current_steering_angle = motor.steer(req_anguler_vel_z);

  Calculates::vel current_vel = calculates.get_velocities(current_steering_angle, twist_msg.linear.x);
  //temperary value
}

void setup()
{ 
  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "Arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_best_effort(
    &twist_sub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "cmd_vel"));
  RCCHECK(rclc_publisher_init_best_effort(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom_velo"));
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_pub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
     "imu/data"));

  // For actuating the motor at 20 KHz (temp)
  const unsigned int timeout = 0.05;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(timeout),
    controlcallback
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &twist_sub,
    &twist_msg,
    &subcmdvel_callback,
    ON_NEW_DATA
  ));
}

void loop()
{ 
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  // motor_control.run(180);
}


