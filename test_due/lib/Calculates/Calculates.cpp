#include "Calculates.h"

Calculates::Calculates(int max_rpm, float wheel_diameter, float wheel_distance_x)
{
    max_rpm_ = max_rpm;
    wheel_round_ = PI * wheel_diameter;
    wheel_distance_x_ = wheel_distance_x;
}

float Calculates::CalculateRpm(float linear_x)
{   
    float linear_x_mins = linear_x * 60;
    float x_rpm = linear_x_mins / wheel_round_;

    dcmotor_rpm = x_rpm;
    
    return dcmotor_rpm;
}

Calculates::vel Calculates::get_velocities(float steer_angle, float rpm)
{
    Calculates::vel velo;
    float average_rps;
    average_rps = (float)rpm / 60;
    velo.linear_x = average_rps * wheel_round_;
    velo.anguler_z = (velo.linear_x * tan(steer_angle)) / wheel_distance_x_;

    return velo;
}

