#include <Arduino.h>
#include <MPU9250.h>

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

public:
    Imu()
    {
    }
    bool startSensor()
    {
        Wire.begin();
        if(!accelerometer_.setup(0x68))
        {
            return false;
        }
        if(!gyroscope_.setup(0x68))
        {
            return false;
        }
        return true;
    }
    geometry_msgs__msg__Vector3 readgyroscope()
    {
        uint8_t gx, gy, gz;
        gx = gyroscope_.getGyroX();
        gy = gyroscope_.getGyroY();
        gz = gyroscope_.getGyroZ();

        gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
        gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
        gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

        return gyro_;
    }
    geometry_msgs__msg__Vector3 readaccelerometer()
    {
        uint8_t ax, ay, az;

        ax = accelerometer_.getAccX();
        ay = accelerometer_.getAccY();
        az = accelerometer_.getAccZ();

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

    ~Imu();
};
