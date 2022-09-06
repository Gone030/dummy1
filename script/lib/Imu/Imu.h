#include <Arduino.h>
#include "MPU9250.h"

#include <sensor_msgs/msg/imu.h>


class Imu
{
private:
    const float accel_scale_ = 1 / 16384.0;
    const float gyro_scale_ = 1 / 131.0;

    MPU9250 accelerometer_;
    MPU9250 gyroscope_;
    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;


protected:
    sensor_msgs__msg__Imu imu_msg_;
    geometry_msgs__msg__Vector3 gyro_cal_;
    const int sample_size_ = 40;
    const float g_to_accel_ = 9.81;

    float accel_cov_ = 0.00001;
    float gyro_cov_ = 0.00001;

    void calibrateGyro()
    {
        geometry_msgs__msg__Vector3 gyro;
        for(int i=0; i<sample_size_; i++)
        {
            gyro = readgyroscope();
            gyro_cal_.x += gyro.x;
            gyro_cal_.y += gyro.y;
            gyro_cal_.z += gyro.z;

            delay(50);
        }
        gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
        gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
        gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
    }

public:
    Imu()
    {
        imu_msg_.header.frame_id.data = (char*)malloc(100*sizeof(char));
        char string1[] = "imu_base";
        memcpy(imu_msg_.header.frame_id.data, string1, strlen(string1) + 1);
        imu_msg_.header.frame_id.size = strlen(imu_msg_.header.frame_id.data);
        imu_msg_.header.frame_id.capacity = 100;
    }
    ~Imu()
    {
    }
    bool startSensor()
    {
        Wire.begin();
        accelerometer_.initialize();
        if(!accelerometer_.testConnection())
        {
            return false;
        }
        gyroscope_.initialize();
        if(!gyroscope_.testConnection())
        {
            return false;
        }
        return true;
    }
    geometry_msgs__msg__Vector3 readgyroscope()
    {
        int16_t gx, gy, gz;

        gyroscope_.getRotation(&gx, &gy, &gz);

        gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
        gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
        gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

        return gyro_;
    }
    geometry_msgs__msg__Vector3 readaccelerometer()
    {
        int16_t ax, ay, az;

        accelerometer_.getAcceleration(&ax, &ay, &az);

        accel_.x = ax * (double) accel_scale_ * g_to_accel_;
        accel_.y = ay * (double) accel_scale_ * g_to_accel_;
        accel_.z = az * (double) accel_scale_ * g_to_accel_;

        return accel_;
    }

    bool init()
    {
        bool sensor_ready = startSensor();
        if(sensor_ready)
        {
            calibrateGyro();
        }
        return sensor_ready;
    }
    sensor_msgs__msg__Imu getdata()
    {
        imu_msg_.angular_velocity = readgyroscope();
        imu_msg_.angular_velocity.x -= gyro_cal_.x;
        imu_msg_.angular_velocity.y -= gyro_cal_.y;
        imu_msg_.angular_velocity.z -= gyro_cal_.z;

        if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01)
            imu_msg_.angular_velocity.x = 0;

        if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01)
            imu_msg_.angular_velocity.y = 0;

        if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01)
            imu_msg_.angular_velocity.z = 0;

        imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

        imu_msg_.linear_acceleration = readaccelerometer();
        imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

        return imu_msg_;
    }

};

