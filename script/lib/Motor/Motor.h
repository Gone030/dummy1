#include <Arduino.h>

class control
{
private:
    int pwm_pin_;
    int motor_pin_A_;
    int motor_pin_B_;
    int servo_pin_;
    float steering_angle_;
    double pwm_duty_;
    float temp_;
    int servo_control_angle;

public:
    control(int pwm_pin, int motor_pin_a, int motor_pin_b, int servo_pin);
    void run(double pwm);
    float steer(float steering_angle);
    void servoSet();
    int steerNow();
    float returnVel();

    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};
