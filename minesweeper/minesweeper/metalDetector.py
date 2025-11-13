import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('metal_detector_node')
        self.treasurePublisher = self.create_publisher(Marker, '/minesweeper/treasure_marker', 10)
        self._last_tool_rotate_state: JointState | None = None
        self.toolRotateSub = self.create_subscription(
            JointState,
            '/VSV/ToolRotate/state',
            self._tool_rotate_callback,
            10
        )
        self.metalDetectorSub = self.create_subscription(
            Float32,
            '/vrep/metalDetector',
            self._metal_detector_callback,
            10
        )

    def _tool_rotate_callback(self, msg: JointState):
        """Store the latest JointState message from /VSV/ToolRotate/state."""
        self._last_tool_rotate_state = msg

    def get_last_tool_positions(self):
        """Return the last joint positions list or None if not yet received."""
        return None if self._last_tool_rotate_state is None else list(self._last_tool_rotate_state.position)

    def _metal_detector_callback(self, msg: Float32):
        """Handle metal detector reading: fetch last tool joint positions and optionally publish a marker."""
        positions = self.get_last_tool_positions()
        if positions is None:
            self.get_logger().warn('Metal detector triggered but no JointState received yet from /VSV/ToolRotate/state.')
            return

        # Log the positions for now; adapt as needed.
        self.get_logger().info(f'Metal detector value={msg.data:.3f} | tool joint positions={positions}')

        if msg.data > 0.9:  # arbitrary threshold
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "treasure"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # Positioning could use robot/tool pose; placeholder at origin
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.84
            marker.color.b = 0.0
            self.treasurePublisher.publish(marker)
            self.get_logger().info('Treasure marker published.')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()