#include <Arduino.h>
#include <MPU9250.h>

#include <sensor_msgs/msg/imu.h>


class Imu
{
private:
    
    MPU9250 accelerometer_;
    MPU9250 gyroscope_;
    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;

public:
    Imu()
    {}
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
    ~Imu();
};

Imu::Imu(/* args */)
{
}

Imu::~Imu()
{
}
