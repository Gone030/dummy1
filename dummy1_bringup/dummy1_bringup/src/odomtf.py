import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster


class odomtfpub(Node):
    def __init__(self):
        super().__init__('odometry_tf_publisher')
        qos_profile = QoSProfile(depth = 10)

        self.subOdom = self.create_subscription(
            Odometry, 'odom', self.odom_calc, qos_profile
        )
        self.pubOdomTF = TransformBroadcaster(self)
    def odom_calc(self, msg):
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = msg.header.frame_id
        odom_tf.header.stamp = msg.header.stamp
        odom_tf.child_frame_id = msg.child_frame_id

        odom_tf.transform.translation.x = msg.pose.pose.position.x
        odom_tf.transform.translation.y = msg.pose.pose.position.y
        odom_tf.transform.translation.z = msg.pose.pose.position.z
        odom_tf.transform.rotation = msg.pose.pose.orientation
        self.pubOdomTF.sendTransform(odom_tf)
def main(args = None):
    rclpy.init(args=args)
    node = odomtfpub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
