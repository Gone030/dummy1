#include <Arduino.h>

#include <nav_msgs/msg/odometry.h>
class Odom
{
    private:
        float linear_vel_x_;
        float angular_vel_z_;
        nav_msgs__msg__Odometry odometry_msg_;
        float x_pose_;
        float y_pose_;
        float theta_;

        const void euler_to_qurternion(float roll, float pitch, float yaw, float* q);
    public:
        nav_msgs__msg__Odometry getData();
        Odom();
        void update(float dt, float linear_vel_x, float angular_vel_z);

};
