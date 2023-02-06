#include <Arduino.h>

class Calculates
{
    private:

        double wheel_distance_x_;
        double max_rpm_;
        double wheel_round_;
        double gear_ratio_;
        double encoder_gear_ratio_;
    public:
        double dcmotor_rpm;

        struct vel
        {
            double linear_x;
            double angular_z;
        };
        Calculates(int max_rpm, double gear_ratio, double encoder_gear_ratio, double wheel_diameter, double wheel_distance_x);
        vel get_velocities(float steer_angle, double rpm);
        double CalculateRpm(double linear_x);
};
