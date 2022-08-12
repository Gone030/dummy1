#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "Calculates.h"
#include "Motor.h"
#include "Imu.h"

rcl_subscription_t twist_sub;
rcl_publisher_t odom_velo_pub;
rcl_publisher_t imu_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;

geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist odom_velo;
sensor_msgs__msg__Imu imu_msg;


enum states
{
  Waiting_agent,
  agent_available,
  agent_connected,
  agent_disconnected
} state;


#define max_rpm 18000

float wheel_diameter = 7.3;
float wheel_distence_x = 18.3;

double integral; // pid variable
double derivative;
double prev_error;
float kp = 20;
float ki = 0;
float kd = 0;
int min_val = -255;
int max_val = 255;


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
Imu imu;

unsigned long prev_cmdvel_time = 0;


void error_loop(){
  while(1){
    for(int i=0; i<2; i++)
    {
      digitalWrite(13, HIGH);
      delay(150);
      digitalWrite(13, LOW);
      delay(150);
    }
    delay(1000);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define EXECUTE_EVERY_N_NS(MS, X) do{ \
 static volatile int64_t init = -1; \
 if (init == -1) { init = uxr_millis();} \
 if (uxr_millis() - init > MS) {X; init = uxr_millis();} \
} while (0)



void encoderCount()
{
  dir_ = (digitalRead(pB) == HIGH)? -1: 1;
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
  pid = (kp * error); //+ (ki * integral) + (kd * derivative);
  prev_error = error;

  return constrain(pid, min_val, max_val);
}
void subcmdvel_callback(const void *msgin)
{
  digitalWrite(13, !digitalRead(13));
  prev_cmdvel_time = millis();
}


void syncTime()
{
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}
struct timespec getTime()
{
  struct  timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 100000;

  return tp;
}
void publishData()
{
  imu_msg = imu.getdata();

  struct timespec time_stamp = getTime();

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  RCSOFTCHECK(rcl_publish(&odom_velo_pub, &odom_velo, NULL));
}

void controlcallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if(timer != NULL)
  {
    move();
    publishData();
  }
}


void move()
{
  if((millis() - prev_cmdvel_time) >= 200)
  {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
  }
  float calc_dc_rpm = calculates.CalculateRpm(twist_msg.linear.x);
  float ecd_rpm = getRPM();
  double pidvel = pidcompute(calc_dc_rpm, ecd_rpm);
  float req_anguler_vel_z = twist_msg.angular.z;
  motor.run(pidvel);
  float current_steering_angle = motor.steer(req_anguler_vel_z);

  Calculates::vel current_vel = calculates.get_velocities(current_steering_angle, twist_msg.linear.x);
  //temperary value
  odom_velo.linear.x = current_vel.linear_x;
  odom_velo.angular.x = current_vel.anguler_z;
}


bool createEntities()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 7;
  RCCHECK(rclc_node_init_with_options(&node, "Dummy1_Due_node", "", &support, &node_ops));

  RCCHECK(rclc_subscription_init_best_effort(
    &twist_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  RCCHECK(rclc_publisher_init_best_effort(
    &odom_velo_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "odom_velo"));


  RCCHECK(rclc_publisher_init_best_effort(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_due"
  ));

  // For actuating the motor at 20 KHz (temp) 0.05
  const unsigned int timeout = 0.05;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(timeout),
    controlcallback
  ));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &twist_sub,
    &twist_msg,
    &subcmdvel_callback,
    ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  syncTime();
  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_pub, &node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  rcl_timer_fini(&control_timer);
  //나머지 채워놓을것
  return true;
}
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);

  bool imu_test = imu.init();

  if(!imu_test)
  {
    while(1)
    {
      for(int i=0; i<4; i++)
      {
      digitalWrite(13,HIGH);
      delay(150);
      digitalWrite(13,LOW);
      delay(150);
      }
      delay(1000);
    }
  }
  pinMode(13,OUTPUT);
  set_microros_transports();
}

void loop()
{
  switch (state)
  {
    case Waiting_agent:
      EXECUTE_EVERY_N_NS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? agent_available : Waiting_agent;);
      break;
    case agent_available:
      state = (true == createEntities()) ? agent_connected : Waiting_agent;
      if ( state == Waiting_agent)
      {
        destroyEntities();
      }
      break;
    case agent_connected:
      EXECUTE_EVERY_N_NS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? agent_connected : agent_disconnected;);
      if (state == agent_connected)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case agent_disconnected:
      destroyEntities();
      state = Waiting_agent;
      break;
    default:
      break;
  }
}
