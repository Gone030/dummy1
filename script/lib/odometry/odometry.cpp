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

    char string2[] = "base_link";
    odometry_msg_.child_frame_id.capacity = sizeof(string2);
    odometry_msg_.child_frame_id.data = (char*)malloc(odometry_msg_.child_frame_id.capacity * sizeof(char));
    odometry_msg_.child_frame_id.size = sizeof(string2);
    strcpy(odometry_msg_.child_frame_id.data, "base_link");

}


void Odom::integrateRK2(double linear, double angular)
{
    const double direction = theta_ + angular * 0.5;

    x_pose_ += linear * cos(direction);
    y_pose_ += linear * sin(direction);
    theta_ += angular;
}

void Odom::integrate(double linear, double angular)
{
    if( angular == 0 || fabs(angular) < 1e-3 )
    {
        integrateRK2(linear, angular);
    }
    else if(linear == 0)
    {
        x_pose_ += 0;
        y_pose_ += 0;
        theta_ += 0;
    }
    else
    {
        const double theta_old = theta_;
        const double r = linear / angular;
        theta_ += angular;
        x_pose_ += r * (sin(theta_) - sin(theta_old));
        y_pose_ += -r * (cos(theta_) - cos(theta_old));
    }
}


void Odom::update(float dt, double linear_vel_x, double angular_vel_z)
{
    linear_ = linear_vel_x;
    angular_ = angular_vel_z;

    integrate(linear_ * dt, angular_ * dt);

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

    odometry_msg_.twist.twist.linear.x = linear_;
    odometry_msg_.twist.twist.linear.y = 0.0;
    odometry_msg_.twist.twist.linear.z = 0.0;

    odometry_msg_.twist.twist.angular.x = 0.0;
    odometry_msg_.twist.twist.angular.y = 0.0;
    odometry_msg_.twist.twist.angular.z = angular_;

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

