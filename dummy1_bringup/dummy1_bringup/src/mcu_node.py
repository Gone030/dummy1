from sqlite3 import Timestamp
import rclpy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

import math
import numpy as np

global prv_num
global missing_num

prv_num = 0
missing_num = []

class ros22mcu(Node):

    def __init__(self):
        super().__init__('mcu_node')
        qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info('waiting')
        # self.subvel = self.create_subscription(Twist, 'cmd_vel', self.decision ,qos_profile)
        # self.subOdovel = self.create_subscription(Odometry, 'odom_velo',self.odometry, qos_profile)
        self.subImu = self.create_subscription(Imu, 'imu_due', self.imucontrol, qos_profile)
        self.pubImu = self.create_publisher(Imu, 'imu/data', qos_profile)
        # self.pubOdo = self.create_publisher(Odometry, 'odom', qos_profile)
        self.subtest = self.create_subscription(Int32, 'test_publisher', self.testcallback, qos_profile)

    def imucontrol(self, imu):
        timestamp_now = self.get_clock().now().to_msg()
        imu_msg = imu
        imu_msg.header.frame_id = 'imu_base'
        imu_msg.header.stamp = timestamp_now
        self.pubImu.publish(imu_msg)
    def testcallback(self, msg):
        global prv_num
        global missing_num

        if msg.data != prv_num + 1:
            missing_num.extend(list(range(prv_num + 1, msg.data)))
            print("              missed: {0}".format(missing_num))
        prv_num = msg.data
        print("              Receive: {0:>3}".format(msg.data))

    # def odometry(self, vel):
    #     linear_vel_x = vel.linear_x
    #     angular_vel_z = vel.angular_vel_z
    #     odom_msg = Odometry()
    #     self.x_pose = 0
    #     self.y_pose = 0
    #     self.theta = 0
    #     self.now_time = self.get_clock().now()
    #     self.dt = (s elf.now_time - self.prev_time).nanosecond * 1e-9
    #     self.prev_time = self.now_time
    #     self.dtheta = angular_vel_z * self.dt
    #     self.costh = math.cos(self.theta)
    #     self.sinth = math.sin(self.theta)
    #     self.delta_x = (linear_vel_x * self.costh) * self.dt
    #     self.delta_y = (linear_vel_x * self.sinth) * self.dt

    #     self.x_pose += self.delta_x
    #     self.y_pose += self.delta_y
    #     self.theta += self.dtheta

    #     q = self.euler_to_quaternion(0, 0, angular_vel_z)

    #     odom_msg.header.frame_id = 'odom'
    #     odom_msg.child_frame_id = 'base_footprint'
    #     odom_msg.pose.pose.position.x = self.x_pose
    #     odom_msg.pose.pose.position.y = self.y_pose
    #     odom_msg.pose.pose.position.z = 0.0

    #     odom_msg.pose.pose.orientation.x = q[1]
    #     odom_msg.pose.pose.orientation.y = q[2]
    #     odom_msg.pose.pose.orientation.z = q[3]
    #     odom_msg.pose.pose.orientation.w = q[0]

    #     odom_msg.pose.covariance[0] = 0.001
    #     odom_msg.pose.covariance[7] = 0.001
    #     odom_msg.pose.covariance[35] = 0.001

    #     odom_msg.twist.twist.linear.x = linear_vel_x
    #     odom_msg.twist.twist.linear.y = 0
    #     odom_msg.twist.twist.linear.z = 0

    #     odom_msg.twist.twist.angular.x = 0
    #     odom_msg.twist.twist.angular.y = 0
    #     odom_msg.twist.twist.angular.z = angular_vel_z

    #     odom_msg.twist.covariance[0] = 0.001
    #     odom_msg.twist.covariance[7] = 0.001
    #     odom_msg.twist.covariance[35] = 0.001

    #     self.pubOdo.publish(odom_msg)

    # def euler_to_quaternion(self, roll, pitch, yaw):
    #     cy = math.cos(yaw * 0.5)
    #     sy = math.sin(yaw * 0.5)
    #     cp = math.cos(pitch * 0.5)
    #     sp = math.sin(pitch * 0.5)
    #     cr = math.cos(roll * 0.5)
    #     sr = math.sin(roll * 0.5)
    #     q = [0] * 4
    #     q[0] = cy * cp * sr - sy * sp * cr
    #     q[1] = sy * cp * sr + cy * sp * cr
    #     q[2] = sy * cp * cr - cy * sp * sr
    #     q[3] = cy * cp * cr + sy * sp * sr
    #     return q

    # def decision(self, vel):
    #     self.target_linear = int(vel.linear.x * 1000) #Dc motor vel
    #     self.target_angular = int(vel.angular.z * 1000) #Servo motor ang

    #     self.write_data(str(self.target_angular +'!'+self.target_linear))

def main(args=None):
    rclpy.init(args=args)
    node_= ros22mcu()
    try:
        rclpy.spin(node_)
    except KeyboardInterrupt:
        node_.get_logger().info('bye')
    finally:
        node_.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
