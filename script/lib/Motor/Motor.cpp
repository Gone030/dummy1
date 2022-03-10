#include "Motor.h"

control::control(int pwm_pin, int motor_pin_a, int motor_pin_b)
{
    pwm_pin_ = pwm_pin;
    motor_pin_A_ = motor_pin_a;
    motor_pin_B_ = motor_pin_b;

    pinMode(motor_pin_A_, OUTPUT);
    pinMode(motor_pin_B_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT); 
    
    analogWrite(pwm_pin_, abs(0));
}

void control::run(int pwm)
{
    if(pwm > 0)
    {
        digitalWrite(motor_pin_A_, HIGH);
        digitalWrite(motor_pin_B_, LOW);
    }
    else if(pwm < 0)
    {
        digitalWrite(motor_pin_A_, LOW);
        digitalWrite(motor_pin_B_, HIGH);
    }
    analogWrite(pwm_pin_, abs(pwm));
}
