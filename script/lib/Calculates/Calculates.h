#include <Arduino.h>

class Calculates
{
    private:

        float wheel_distance_x_;
        float max_rpm_;
        float wheel_round_;

    public:
        float dcmotor_rpm;

        struct vel
        {
            double linear_x;
            double angular_z;
        };
        Calculates(int max_rpm, float wheel_diameter, float wheel_distance_x);
        vel get_velocities(float steer_angle, float rpm);
        float CalculateRpm(float linear_x);
};
