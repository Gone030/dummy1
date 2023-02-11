#include "Calculates.h"

Calculates::Calculates(int max_rpm, double gear_ratio, double encoder_gear_ratio, double wheel_diameter, double wheel_distance_x)
{
    max_rpm_ = max_rpm;
    gear_ratio_ = gear_ratio;
    encoder_gear_ratio_ = encoder_gear_ratio;
    wheel_round_ = PI * wheel_diameter;
    wheel_distance_x_ = wheel_distance_x;
}

double Calculates::CalculateRpm(double linear_x)
{
    double linear_x_mins = linear_x * 60.0 / gear_ratio_; // Gear ratio (temp)
    double x_rpm = linear_x_mins / wheel_round_;

    dcmotor_rpm = constrain(x_rpm, -max_rpm_, max_rpm_ );

    return dcmotor_rpm;
}

Calculates::vel Calculates::get_velocities(float steer_angle, double rpm)
{
    Calculates::vel velo;
    double average_rps;
    average_rps = (double)rpm / 60.0;
    velo.linear_x = average_rps * wheel_round_ * encoder_gear_ratio_; // Gear ratio (temp)
    velo.angular_z = (velo.linear_x * tan(steer_angle)) / wheel_distance_x_;
    if(fabs(velo.angular_z) < 1e-3) {velo.angular_z = 0;}

    return velo;
}

