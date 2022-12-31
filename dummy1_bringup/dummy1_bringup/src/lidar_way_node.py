
import rclpy
from msg_srv_act_interface.msg import LidarinfoMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def Rad2Deg(rad):
    return rad*180/3.141592


class LIDARWAY(Node):
    def __init__(self):
        super().__init__('lidar_way_node')
        self.LIDAR_ERR_min = 0.123
        self.LIDAR_ERR_max = 0.20
        self.once = 1
        self.lidardirection()

    def lidardirection(self):
        self.get_logger().info('lidardirection')
        qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sublaserscan = self.create_subscription(
            LaserScan, '/scan', self.checkaround, qos_profile)
        self.pubdetected = self.create_publisher(
            LidarinfoMsg, 'detected', qos_profile)

    def checkaround(self, msg):
        if self.once < 2:
            self.get_logger().info('check around')
            self.once += 1
        data = LidarinfoMsg()
        self.count = int(msg.scan_time / msg.time_increment)
        for i in range(self.count):
            self.degree = Rad2Deg(msg.angle_min + msg.angle_increment * i)
            data.angle = self.degree
            data.distance = round(msg.ranges[i] * 100 , 3)
            if 0 < data.angle < 30:
                data.data = '1'
                if 12 < data.distance < 30:
                    self.pubdetected.publish(data)
            elif 30 < data.angle < 60:
                data.data = '2'
                if 12 < data.distance < 30:
                    self.pubdetected.publish(data)
            elif -30 < data.angle <= 0:
                data.data = '-1'
                if 12 < data.distance < 30:
                    self.pubdetected.publish(data)
            elif -60 < data.angle < -30:
                data.data = '-2'
                if 12 < data.distance < 30:
                    self.pubdetected.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = LIDARWAY()
    try:
        node.get_logger().info('Hello!!')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Good bye')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
