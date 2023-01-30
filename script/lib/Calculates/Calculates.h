#include <Arduino.h>

class Calculates
{
    private:

        double wheel_distance_x_;
        double max_rpm_;
        double wheel_round_;

    public:
        double dcmotor_rpm;

        struct vel
        {
            double linear_x;
            double angular_z;
        };
        Calculates(int max_rpm, double wheel_diameter, double wheel_distance_x);
        vel get_velocities(float steer_angle, float rpm);
        double CalculateRpm(double linear_x);
};
