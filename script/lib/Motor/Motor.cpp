#include "Motor.h"
#include "Servo.h"

#define MAX_SERVO_ANGLE 2.0944 //rad
#define MIN_SERVO_ANGLE 1.0472 //rad

Servo steering_servo;
control::control(int motor_pin_F, int motor_pin_R, int motor_pin_F_EN, int motor_pin_R_EN, int servo_pin ):
    motor_pin_F_(motor_pin_F),
    motor_pin_R_(motor_pin_R),
    motor_pin_F_EN_(motor_pin_F_EN),
    motor_pin_R_EN_(motor_pin_R_EN),
    servo_pin_(servo_pin)
{
    pinMode(motor_pin_F_, OUTPUT);
    pinMode(motor_pin_R_, OUTPUT);
    pinMode(motor_pin_F_EN_, OUTPUT);
    pinMode(motor_pin_R_EN_, OUTPUT);

    steering_servo.attach(servo_pin_);
    steering_servo.write(90);
}

void control::run(double pwm_duty)
{
    pwm_duty_ = (int)abs(pwm_duty);
    if(pwm_duty > 0)
    {
        digitalWrite(motor_pin_F_EN_, HIGH);
        digitalWrite(motor_pin_R_EN_, HIGH);
        analogWrite(motor_pin_F_,pwm_duty_);
        analogWrite(motor_pin_R_,0);
    }
    else if(pwm_duty < 0)
    {
        digitalWrite(motor_pin_F_EN_, HIGH);
        digitalWrite(motor_pin_R_EN_, HIGH);
        analogWrite(motor_pin_F_,0);
        analogWrite(motor_pin_R_,pwm_duty_);
    }
    else
    {
        digitalWrite(motor_pin_F_EN_, HIGH);
        digitalWrite(motor_pin_R_EN_, HIGH);
        analogWrite(motor_pin_F_,0);
        analogWrite(motor_pin_R_,0);
        digitalWrite(motor_pin_F_EN_, LOW);
        digitalWrite(motor_pin_R_EN_, LOW);
    }
}

float control::steer(float steering_angle)
{
    temp_ = mapFloat(-steering_angle, -1.0, 1.0, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE) * 180/PI;
    servo_control_angle = (int)temp_;
    servo_control_angle = constrain(servo_control_angle, 75, 105);
    steering_servo.write(servo_control_angle);

    return steering_angle;
}

float control::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
