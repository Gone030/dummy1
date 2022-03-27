#include "Motor.h"
#include <DuePWM.h>
#include <Servo.h>

#define PWM_DC 20000
#define PWM_SV 1000
#define MAX_SERVO_ANGLE 2.0944 //rad
#define MIN_SERVO_ANGLE 1.0472 //rad

DuePWM pwm(PWM_DC, PWM_SV);
Servo steering_servo;
control::control(int pwm_pin, int motor_pin_a, int motor_pin_b, int servo_pin )
{
    pwm_pin_ = pwm_pin;
    motor_pin_A_ = motor_pin_a;
    motor_pin_B_ = motor_pin_b;
    servo_pin_ = servo_pin;

    pinMode(motor_pin_A_, OUTPUT);
    pinMode(motor_pin_B_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT); 

    pwm.pinFreq1(pwm_pin_);
    pwm.setFreq1(PWM_DC);
    
    pwm.pinDuty(pwm_pin_, 0);
    steering_servo.attach(servo_pin_);

}

void control::run(int pwm_duty)
{   
    bool dir;
    if(pwm_duty >= 0)
        dir = true;
    else
        dir = false;

    pwm_duty_ = pwm_duty;
    pwm_duty_ = myMap(pwm_duty_, 0, 255);
    if(dir)
    {
        digitalWrite(motor_pin_A_, HIGH);
        digitalWrite(motor_pin_B_, LOW);
    }
    else
    {
        digitalWrite(motor_pin_A_, LOW);
        digitalWrite(motor_pin_B_, HIGH);
    }
    pwm.pinDuty(pwm_pin_, pwm_duty_);
}

float control::steer(float steering_angle)
{
    float servo_control_angle;
    steering_angle_ = constrain(steering_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servo_control_angle = mapFloat(steering_angle_, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 0, PI) * (180/PI);
    steering_servo.write(servo_control_angle);

    return steering_angle_;
}

float control::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
