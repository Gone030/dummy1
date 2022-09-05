#include <Arduino.h>

class control
{
private:
    double pwm_duty_;
    int pwm_pin_;
    int motor_pin_A_;
    int motor_pin_B_;
    int servo_pin_;
    float steering_angle_;
    int servo_control_angle;
    float temp_;


public:
    control(int pwm_pin, int motor_pin_a, int motor_pin_b, int servo_pin);
    void run(double pwm);
    float steer(float steering_angle);
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};
