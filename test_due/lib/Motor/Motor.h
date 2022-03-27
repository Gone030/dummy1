#include <Arduino.h>

class control
{
private:
    int pwm_duty_;
    int pwm_pin_;
    int motor_pin_A_;
    int motor_pin_B_;
    int servo_pin_;
    float steering_angle_;
     unsigned int myMap(float val, float minin, float maxin) {
        return min((unsigned int) max(0., (val - minin) * (float)4095 / (maxin - minin)), 4095);
    }

public:
    control(int pwm_pin, int motor_pin_a, int motor_pin_b, int servo_pin);
    void run(int pwm);
    float steer(float steering_angle);
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};
