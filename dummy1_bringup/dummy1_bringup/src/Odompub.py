import datetime
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class odompub(Node):

    def __init__(self):
        super().__init__('mcu_node')
        qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info('waiting')
        self.subOdovel = self.create_subscription(Twist, 'odom_velo',self.odometry, qos_profile)
        self.pubOdo = self.create_publisher(Odometry, 'odom/unfiltered', qos_profile)

    def odometry(self, vel):
        current_time = datetime.datetime.now().timestamp()
        linear_vel_x = float(vel.linear.x)
        angular_vel_z = float(vel.angular.z)

        odom_msg = Odometry()
        self.x_pose = 0
        self.y_pose = 0
        self.theta = 0
        self.costh = math.cos(self.theta)
        self.sinth = math.sin(self.theta)
        last_vel_time = datetime.datetime.now().timestamp()
        dt = float(current_time - last_vel_time) * 100000
        self.dtheta = angular_vel_z * dt
        self.delta_x = (linear_vel_x * self.costh) * dt
        self.delta_y = (linear_vel_x * self.sinth) * dt

        self.x_pose += self.delta_x
        self.y_pose += self.delta_y
        self.theta += self.dtheta

        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        self.pubOdo.publish(odom_msg)

        q = self.euler_to_quaternion(0, 0, angular_vel_z)

        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x_pose
        odom_msg.pose.pose.position.y = self.y_pose
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]

        odom_msg.pose.covariance[0] = 0.001
        odom_msg.pose.covariance[7] = 0.001
        odom_msg.pose.covariance[35] = 0.001

        odom_msg.twist.twist.linear.x = linear_vel_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel_z

        odom_msg.twist.covariance[0] = 0.001
        odom_msg.twist.covariance[7] = 0.001
        odom_msg.twist.covariance[35] = 0.001

        self.pubOdo.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cosyaw = math.cos(yaw * 0.5)
        sinyaw = math.sin(yaw * 0.5)
        cospitch = math.cos(pitch * 0.5)
        sinpitch = math.sin(pitch * 0.5)
        cosroll = math.cos(roll * 0.5)
        sinroll = math.sin(roll * 0.5)

        q = [0] * 4

        q[0] = cosyaw * cospitch * cosroll + sinyaw * sinpitch * sinroll
        q[1] = sinyaw * cospitch * cosroll - cosyaw * sinpitch * sinroll
        q[2] = sinyaw * cospitch * sinroll + cosyaw * sinpitch * cosroll
        q[3] = cosyaw * cospitch * sinroll - sinyaw * sinpitch * cosroll

        return q
def main(args=None):
    rclpy.init(args=args)
    node_= odompub()
    try:
        rclpy.spin(node_)
    except KeyboardInterrupt:
        node_.get_logger().info('bye')
    finally:
        node_.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
