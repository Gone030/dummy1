#include <Arduino.h>
#include <Wire.h>

#include <sensor_msgs/msg/imu.h>

class Imu
{
    private:
        uint8_t cnt_;
        int16_t acc_raw_[3]={0,}, gyro_raw_[3]={0,};
        double gyro_rate_[3];
        double acc_rate_[3];
        float accel_cov_ = 0.00001f;
        float gyro_cov_ = 0.00001f;
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        int16_t offset_[3];
        sensor_msgs__msg__Imu imu_msg_;
        geometry_msgs__msg__Vector3 gyro_cal_;
        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;
            for(int i=0; i<40; i++)
            {
                gyro = getgyro();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }
            gyro_cal_.x = gyro_cal_.x / 40.0;
            gyro_cal_.y = gyro_cal_.y / 40.0;
            gyro_cal_.z = gyro_cal_.z / 40.0;

            offset_[0] = gyro_cal_.x;
            offset_[1] = gyro_cal_.y;
            offset_[2] = gyro_cal_.z;
        }
    public:
        Imu()
        {
            char frame_name[] = "imu_link";
            imu_msg_.header.frame_id.capacity = sizeof(frame_name);
            imu_msg_.header.frame_id.data = (char*)malloc(imu_msg_.header.frame_id.capacity * sizeof(char));
            imu_msg_.header.frame_id.size = sizeof(frame_name);
            strcpy(imu_msg_.header.frame_id.data, "imu_link");

        }
        ~Imu()
        {}
        void init()
        {
            Wire.begin();
            Wire.beginTransmission(0x68);
            Wire.write(107);
            Wire.write(0);
            Wire.endTransmission();
            // Register 26 DLPF
            for(uint8_t i = 2; i <= 7; i++)
            {
                Wire.beginTransmission(0x68);
                Wire.write(26);
                Wire.write(i << 3 | 0x03);
                Wire.endTransmission();
            }
            // Register 27 Gyro, Full Scale Range = +-2000 ˚/s
            Wire.beginTransmission(0x68);
            Wire.write(27);
            Wire.write(3 << 3);
            Wire.endTransmission();
            // Register 28 Accel
            Wire.beginTransmission(0x68);
            Wire.write(28);
            Wire.write(0);
            Wire.endTransmission();

            calibrateGyro();
        }

        geometry_msgs__msg__Vector3 getacc()
        {
            Wire.beginTransmission(0x68);
            Wire.write(59);
            Wire.endTransmission();
            Wire.requestFrom(0x68, 6);
            for(cnt_ = 0; cnt_ < 3; cnt_++) acc_raw_[cnt_] = (Wire.read() << 8) | Wire.read();

            for(cnt_ = 0; cnt_ < 3; cnt_++) acc_rate_[cnt_] = (acc_raw_[cnt_] / 16384.0) * 9.80665; // m/s^2

            accel_.x = acc_rate_[0];
            accel_.y = acc_rate_[1];
            accel_.z = acc_rate_[2];

            return accel_;
        }

        geometry_msgs__msg__Vector3 getgyro()
        {
            Wire.beginTransmission(0x68);
            Wire.write(67);
            Wire.endTransmission();
            Wire.requestFrom(0x68, 6);
            for(cnt_ = 0; cnt_ < 3; cnt_++)
                gyro_raw_[cnt_] = gyro_raw_[cnt_] * 0.8 + 0.2 * (((Wire.read() << 8) | Wire.read()) - offset_[cnt_]);

            for(cnt_ = 0; cnt_ < 3; cnt_++) gyro_rate_[cnt_] = gyro_raw_[cnt_] / 16.4; // rad/sec

            gyro_.x = gyro_rate_[0];
            gyro_.y = gyro_rate_[1];
            gyro_.z = gyro_rate_[2];

            return gyro_;
        }

        sensor_msgs__msg__Imu operation()
        {
            geometry_msgs__msg__Vector3 acc__ = getacc();
            geometry_msgs__msg__Vector3 gyro__ = getgyro();


            imu_msg_.angular_velocity = gyro__;

            imu_msg_.linear_acceleration = acc__;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            return imu_msg_;
        }
};
