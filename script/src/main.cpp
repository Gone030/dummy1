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
float max_rpm_ratio = 0.0;
float wheel_diameter = 0.07;
float wheel_distence_x = 0.2;
float wheel_distance_y = 0.15;
int pwm_pin = 6;
int motor_pin_a = 7;
int motor_pin_b = 8;
int servo_pin = 9;

control motor(pwm_pin, motor_pin_a, motor_pin_b, servo_pin);
Calculates calculates(max_rpm, max_rpm_ratio, wheel_diameter, wheel_distence_x,wheel_distance_y);

unsigned long prev_cmdvel_time = 0;



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
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
    Serial.print("임시");
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
  float req_anguler_vel_z = twist_msg.angular.z; 
  float current_steering_angle = motor.steer(req_anguler_vel_z);

  Calculates::vel current_vel = calculates.get_velocities(current_steering_angle,twist_msg.linear.x);
  //temperary value
}

void setup()
{
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
    &imu_pub, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
     "imu/data"));

  const unsigned int timeout = 20;
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

  // motor_control.run(180);
}


