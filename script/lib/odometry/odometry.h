#include <Arduino.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
class Odom
{
    private:
        nav_msgs__msg__Odometry odometry_msg_;
        geometry_msgs__msg__TransformStamped odom_trans_;
        double linear_;
        double angular_;
        double x_pose_;
        double y_pose_;
        double theta_;

        const void euler_to_qurternion(float roll, float pitch, float yaw, float* q);
    public:
        Odom();
        void integrate(double linear, double angular);
        void integrateRK2(double linear, double angular);
        void update(float dt, double linear_vel_x, double angular_vel_z);
        nav_msgs__msg__Odometry getOdomData() { return odometry_msg_; };
        geometry_msgs__msg__TransformStamped getOdomtfData() { return odom_trans_; };

};
