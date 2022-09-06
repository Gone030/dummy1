import rclpy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class ros22mcu(Node):

    def __init__(self):
        super().__init__('mcu_node')
        qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info('waiting')
        # self.subvel = self.create_subscription(Twist, 'cmd_vel', self.decision ,qos_profile)
        self.subOdovel = self.create_subscription(Odometry, 'odom_velo',self.odometry, qos_profile)
        self.subImu = self.create_subscription(Imu, 'imu_due', self.imucontrol, qos_profile)
        self.pubImu = self.create_publisher(Imu, 'imu/data', qos_profile)
        self.pubOdo = self.create_publisher(Odometry, 'odom/unfiltered', qos_profile)

    def imucontrol(self, imu):
        imu_msg = imu
        imu_msg.header.frame_id = 'imu_base'
        self.pubImu.publish(imu_msg)

    def odometry(self, odom):
        odom_msg = odom
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        self.pubOdo.publish(odom_msg)


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
