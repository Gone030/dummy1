#include "Motor.h"
#include "Servo.h"

#define MAX_SERVO_ANGLE 2.0944 //rad
#define MIN_SERVO_ANGLE 1.0472 //rad

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

    steering_servo.attach(servo_pin_);
    steering_servo.write(90);
}

void control::run(double pwm_duty)
{
    pwm_duty_ = pwm_duty;
    if(pwm_duty_ > 0)
    {
        digitalWrite(motor_pin_A_, HIGH);
        digitalWrite(motor_pin_B_, LOW);
    }
    else
    {
        digitalWrite(motor_pin_A_, LOW);
        digitalWrite(motor_pin_B_, HIGH);
    }
    pwm_duty_ = constrain(pwm_duty_,-255,255);
    analogWrite(pwm_pin_, pwm_duty_);
}

float control::steer(float steering_angle)
{
    temp_ = mapFloat(steering_angle, -1.0, 1.0, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE) * 180/PI;
    servo_control_angle = (int)temp_;
    servo_control_angle = constrain(servo_control_angle, 75, 105);
    steering_servo.write(servo_control_angle);

    return steering_angle;
}

float control::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
