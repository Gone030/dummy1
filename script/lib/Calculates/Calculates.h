#include <Arduino.h>

class Calculates
{
    private:

        float wheel_distance_x_;
        float wheel_distance_y_;
        float max_rpm_ratio_;
        float max_rpm_;
        float wheel_round_;

    public:
        float dcmotor_rpm;

        struct vel
        {
            float linear_x;
            float anguler_z;
        };
        Calculates(int max_rpm, float max_rpm_ratio, float wheel_diameter, float wheel_distance_x, float wheel_distance_y);
        vel get_velocities(float steer_angle, float rpm);
        float CalculateRpm(float linear_x);
        
};