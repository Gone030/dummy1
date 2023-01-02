#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float64.h>

#include "Calculates.h"
#include "Motor.h"
#include "PID.h"

rcl_subscription_t twist_sub;
rcl_subscription_t p_sub;
rcl_subscription_t i_sub;
rcl_subscription_t d_sub;
rcl_publisher_t odom_velo_pub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmdvel_time = 0;
unsigned long prev_odom_update = 0;

geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist odom_velo_msg;
std_msgs__msg__Float64 p_msg;
std_msgs__msg__Float64 i_msg;
std_msgs__msg__Float64 d_msg;


enum states
{
  Waiting_agent,
  agent_available,
  agent_connected,
  agent_disconnected
} state;


#define max_rpm 18000

float wheel_diameter = 0.073;
float wheel_distence_x = 0.183;


//motor config
#define motor_pin_F 2
#define motor_pin_R 3
#define motor_pin_F_EN 5
#define motor_pin_R_EN 6
#define servo_pin 4

//encoder config
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

//PID config
#define kp 0.3
#define ki 0.5
#define kd 0.0

int min_val = -255;
int max_val = 255;

PID pid(min_val, max_val, kp, ki, kd);
control motor(motor_pin_F, motor_pin_R, motor_pin_F_EN, motor_pin_R_EN, servo_pin);
Calculates calculates(max_rpm, wheel_diameter, wheel_distence_x);


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

void subcmdvel_callback(const void *msgin)
{
  digitalWrite(13, !digitalRead(13));
  prev_cmdvel_time = millis();
}

void subpvel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  msg->data;
  pid.updatepvel(p_msg.data);
}
void subivel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  msg->data;
  pid.updateivel(i_msg.data);
}
void subdvel_callback(const void *msgin)
{
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  msg->data;
  pid.updatedvel(d_msg.data);
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
  double pidvel = pid.pidcompute(calc_dc_rpm, ecd_rpm);
  float req_anguler_vel_z = twist_msg.angular.z;
  motor.run(pidvel);
  float current_steering_angle = motor.steer(req_anguler_vel_z);
  Calculates::vel current_vel = calculates.get_velocities(current_steering_angle, ecd_rpm);

  odom_velo_msg.linear.x = current_vel.linear_x;
  odom_velo_msg.angular.z = current_vel.anguler_z;

  // unsigned long now = millis();
  // float dt = (now  - prev_odom_update) / 1000.0;

  // prev_odom_update = now;
}

void publishData()
{
  RCSOFTCHECK(rcl_publish(&odom_velo_pub, &odom_velo_msg, NULL));
  // struct timespec time_stamp = getTime();

  // odom_msg.header.stamp.sec = time_stamp.tv_sec;
  // odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

 // RCSOFTCHECK(rcl_publish(&odom_velo_pub, &odom_msg, NULL));
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

  RCCHECK(rclc_subscription_init_best_effort(
    &p_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "p_vel"
  ));
  RCCHECK(rclc_subscription_init_best_effort(
    &i_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "i_vel"
  ));
  RCCHECK(rclc_subscription_init_best_effort(
    &d_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "d_vel"
  ));
  RCCHECK(rclc_publisher_init_best_effort(
    &odom_velo_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "odom_velo"
  ));


  const unsigned int timeout = 20;
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

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &p_sub,
    &p_msg,
    &subpvel_callback,
    ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &i_sub,
    &i_msg,
    &subivel_callback,
    ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &d_sub,
    &d_msg,
    &subdvel_callback,
    ON_NEW_DATA
  ));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_velo_pub, &node);
  rcl_subscription_fini(&twist_sub, &node);
  rcl_subscription_fini(&p_sub, &node);
  rcl_subscription_fini(&i_sub, &node);
  rcl_subscription_fini(&d_sub, &node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  rcl_timer_fini(&control_timer);
  rcl_node_fini(&node);
  return true;
}
void setup()
{
  Serial.begin(115200);
  set_microros_transports();
  pinMode(13, OUTPUT);


  attachInterrupt(pA, encoderCount, FALLING);
  pinMode(pB, INPUT);
  attachInterrupt(pZ, encoderReset, FALLING);

  state = Waiting_agent;
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
