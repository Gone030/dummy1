from rclpy.node import Node
from rclpy.qos import QoSProfile

import rclpy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class jointstatepub(Node):
    def __init__(self):
        super().__init__('dummy1_joint_states')
        qos_profile = QoSProfile(depth = 10)

        self.subOdom = self.create_subscription(
            Odometry, 'odom', self.sub_callback_odom, qos_profile
        )
        self.subCmd = self.create_subscription(
            Twist, 'cmd_vel', self.sub_callback_cmd, qos_profile
        )
        self.pubjs = self.create_publisher(JointState, 'joint_states', qos_profile)

    def sub_callback_odom(self, msg):
        joint_states_velo = JointState()
        joint_states_velo.header.frame_id = "base_link"
        timestamp_now = self.get_clock().now().to_msg()
        joint_states_velo.header.stamp = timestamp_now

        linear_velo = msg.twist.twist.linear.x

        joint_states_velo.name = ["driveRwhl_l_joint", "driveRwhl_R_joint"]
        joint_states_velo.velocity = [linear_velo, linear_velo]

    def sub_callback_cmd(self, msg):
        joint_states_pos = JointState()
        joint_states_pos.header.frame_id = "base_link"
        timestamp_now = self.get_clock().now().to_msg()
        joint_states_pos.header.stamp = timestamp_now

        anguler_steer = msg.angular.z

        joint_states_pos.name = ["front_L_wheel_joint", "front_R_wheel_joint"]
        joint_states_pos.position = [anguler_steer, anguler_steer]
def main(args = None):
    rclpy.init(args=args)
    node = jointstatepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
