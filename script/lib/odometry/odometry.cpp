#include "odometry.h"

Odom::Odom():
    x_pose_(0.0),
    y_pose_(0.0),
    theta_(0.0)
{
    odometry_msg_.header.frame_id.data = (char*)malloc(100*sizeof(char));
    char string1[] = "odom";
    memcpy(odometry_msg_.header.frame_id.data, string1, strlen(string1)+1);
    odometry_msg_.header.frame_id.size = strlen(odometry_msg_.header.frame_id.data);
    odometry_msg_.header.frame_id.capacity = 100;

    odometry_msg_.child_frame_id.data = (char*)malloc(100*sizeof(char));
    char string2[] = "base_footprint";
    memcpy(odometry_msg_.child_frame_id.data, string2, strlen(string2)+1);
    odometry_msg_.child_frame_id.size = strlen(odometry_msg_.child_frame_id.data);
    odometry_msg_.child_frame_id.capacity = 100;
}

void Odom::update(float dt, float linear_vel_x, float angular_vel_z)
{
    linear_vel_x_ = linear_vel_x;
    angular_vel_z_ = angular_vel_z;
    float dtheta = angular_vel_z_ * dt;

    float conth = cos(theta_);
    float sinth = sin(theta_);
    float delta_x = (linear_vel_x_ * conth) * dt;
    float delta_y = (linear_vel_x_ * sinth) * dt;

    x_pose_ += delta_x;
    y_pose_ += delta_y;
    theta_ += dtheta;

    float q[4];
    euler_to_qurternion(0, 0, theta_, q);

    odometry_msg_.pose.pose.position.x = x_pose_;
    odometry_msg_.pose.pose.position.y = y_pose_;
    odometry_msg_.pose.pose.position.z = 0.0;

    odometry_msg_.pose.pose.orientation.x = q[1];
    odometry_msg_.pose.pose.orientation.y = q[2];
    odometry_msg_.pose.pose.orientation.z = q[3];
    odometry_msg_.pose.pose.orientation.w = q[0];

    odometry_msg_.pose.covariance[0] = 0.001;
    odometry_msg_.pose.covariance[7] = 0.001;
    odometry_msg_.pose.covariance[35] = 0.001;

    odometry_msg_.twist.twist.linear.x = linear_vel_x_;
    odometry_msg_.twist.twist.linear.y = 0;
    odometry_msg_.twist.twist.linear.z = 0;

    odometry_msg_.twist.twist.angular.x = 0;
    odometry_msg_.twist.twist.angular.y = 0;
    odometry_msg_.twist.twist.angular.z = angular_vel_z_;

    odometry_msg_.twist.covariance[0] = 0.001;
    odometry_msg_.twist.covariance[7] = 0.001;
    odometry_msg_.twist.covariance[35] = 0.001;
}

const void Odom::euler_to_qurternion(float roll, float pitch, float yaw, float* q)
{
    float cosyaw = cos(yaw * 0.5);
    float sinyaw = sin(yaw * 0.5);
    float cospitch = cos(pitch * 0.5);
    float sinpitch = sin(pitch * 0.5);
    float cosroll = cos(roll * 0.5);
    float sinroll = sin(roll *0.5);

    q[0] = cosyaw * cospitch * cosroll + sinyaw * sinpitch * sinroll;
    q[1] = sinyaw * cospitch * cosroll - cosyaw * sinpitch * sinroll;
    q[2] = sinyaw * cospitch * sinroll + cosyaw * sinpitch * cosroll;
    q[3] = cosyaw * cospitch * sinroll - sinyaw * sinpitch * cosroll;
}
