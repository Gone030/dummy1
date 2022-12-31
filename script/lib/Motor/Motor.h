#include <Arduino.h>

class control
{
private:
    int pwm_duty_;
    int motor_pin_F_;
    int motor_pin_R_;
    int motor_pin_F_EN_;
    int motor_pin_R_EN_;
    int servo_pin_;
    float steering_angle_;
    int servo_control_angle;
    float temp_;


public:
    control(int motor_pin_F, int motor_pin_R, int motor_pin_F_EN, int motor_pin_R_EN, int servo_pin);
    void run(double pwm);
    float steer(float steering_angle);
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};
