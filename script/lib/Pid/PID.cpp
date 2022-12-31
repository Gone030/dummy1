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

double PID::pidcompute(float setpoint, float measured_value)
{
    double error;
    double pid;

    unsigned long current_time_ = millis();
    unsigned long dt = prev_time_ - current_time_;
    error = setpoint - measured_value;
    integral_ += ki_*error*(double)dt;
    derivative_ = (error - prev_error_)/(double)dt;

    if (setpoint == 0 && error == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }
    pid = (kp_ * error) + integral_ + (kd_ * derivative_);
    prev_error_ = error;
    prev_time_ = current_time_;
    if(pid == 0)
        { return pid;}
    else
        return constrain(pid, min_value_, max_value_);
}

void PID::updateparam(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
