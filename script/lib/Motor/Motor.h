#include <Arduino.h>

class control
{
private:
    int pwm_pin_;
    int motor_pin_A_;
    int motor_pin_B_;

public:
    control(int pwm_pin, int motor_pin_a, int motor_pin_b);
    ~control();
    void run(int pwm);
};
