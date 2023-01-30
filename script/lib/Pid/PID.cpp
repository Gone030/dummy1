#include "Arduino.h"
#include "PID.h"

PID::PID(int min_value, int max_value, double kp, double ki, double kd):
    min_value_(min_value),
    max_value_(max_value),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
}

double PID::pidcompute(double setpoint, float measured_value)
{
    double error;
    double pid;

    // unsigned long dt = prev_time_ - current_time_;
    error = setpoint - (double)measured_value;
    integral_ += error; // (double)dt;
    derivative_ = (error - prev_error_) ; //(double)dt;

    if (setpoint == 0 && error == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }
    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_value_, max_value_);
}

void PID::updatepvel(double kp)
{
    kp_ = kp;
}
void PID::updateivel(double ki)
{
    ki_ = ki;
}
void PID::updatedvel(double kd)
{
    kd_ = kd;
}
