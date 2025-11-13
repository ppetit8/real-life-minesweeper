import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.treasurePublisher = self.create_publisher(Marker, '/minesweeper/treasure_marker', 10)
        self.metalDetector = self.create_subscription(
            Float32,
            '/vrep/metalDetector',
            self.listener_callback,
            10)
        self.metalDetector

    def listener_callback(self, msg):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "treasure"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.84
        self.marker.color.b = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = 'Hello World: %d' % self.i
        self.treasurePublisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()