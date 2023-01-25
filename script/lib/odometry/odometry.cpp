#include "odometry.h"

Odom::Odom():
    x_pose_(0.0),
    y_pose_(0.0),
    theta_(0.0)
{
    char string1[] = "odom";
    odometry_msg_.header.frame_id.capacity = sizeof(string1);
    odometry_msg_.header.frame_id.data = (char*)malloc(odometry_msg_.header.frame_id.capacity * sizeof(char));
    odometry_msg_.header.frame_id.size = sizeof(string1);
    strcpy(odometry_msg_.header.frame_id.data, "odom");

    char string2[] = "base_footprint";
    odometry_msg_.child_frame_id.capacity = sizeof(string2);
    odometry_msg_.child_frame_id.data = (char*)malloc(odometry_msg_.child_frame_id.capacity * sizeof(char));
    odometry_msg_.child_frame_id.size = sizeof(string2);
    strcpy(odometry_msg_.child_frame_id.data, "base_footprint");
}

void Odom::update(float dt, float linear_vel_x, float angular_vel_z)
{

    float dtheta = angular_vel_z * dt;

    float conth = cos(theta_);
    float sinth = sin(theta_);
    float delta_x = (linear_vel_x * conth) * dt;
    float delta_y = (linear_vel_x * sinth) * dt;

    x_pose_ += delta_x;
    y_pose_ += delta_y;
    theta_ += dtheta;

    float q[4];
    euler_to_qurternion(0, 0, theta_, q);

    odometry_msg_.pose.pose.position.x = x_pose_;
    odometry_msg_.pose.pose.position.y = y_pose_;
    odometry_msg_.pose.pose.position.z = 0.0;

    odometry_msg_.pose.pose.orientation.x = (double) q[1];
    odometry_msg_.pose.pose.orientation.y = (double) q[2];
    odometry_msg_.pose.pose.orientation.z = (double) q[3];
    odometry_msg_.pose.pose.orientation.w = (double) q[0];

    odometry_msg_.pose.covariance[0] = 0.001;
    odometry_msg_.pose.covariance[7] = 0.001;
    odometry_msg_.pose.covariance[35] = 0.001;

    odometry_msg_.twist.twist.linear.x = linear_vel_x_;
    odometry_msg_.twist.twist.linear.y = 0.0;
    odometry_msg_.twist.twist.linear.z = 0.0;

    odometry_msg_.twist.twist.angular.x = 0.0;
    odometry_msg_.twist.twist.angular.y = 0.0;
    odometry_msg_.twist.twist.angular.z = angular_vel_z_;

    odometry_msg_.twist.covariance[0] = 0.0001;
    odometry_msg_.twist.covariance[7] = 0.0001;
    odometry_msg_.twist.covariance[35] = 0.0001;
}

const void Odom::euler_to_qurternion(float roll, float pitch, float yaw, float* q)
{
    float cosyaw = cos(yaw * 0.5);
    float sinyaw = sin(yaw * 0.5);
    float cospitch = cos(pitch * 0.5);
    float sinpitch = sin(pitch * 0.5);
    float cosroll = cos(roll * 0.5);
    float sinroll = sin(roll * 0.5);

    q[0] = cosyaw * cospitch * cosroll + sinyaw * sinpitch * sinroll;
    q[1] = cosyaw * cospitch * sinroll - sinyaw * sinpitch * cosroll;
    q[2] = sinyaw * cospitch * sinroll + cosyaw * sinpitch * cosroll;
    q[3] = sinyaw * cospitch * cosroll - cosyaw * sinpitch * sinroll;
}

nav_msgs__msg__Odometry Odom::getData()
{
    return odometry_msg_;
}
