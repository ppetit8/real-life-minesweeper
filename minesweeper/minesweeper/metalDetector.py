import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
import math
import time


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('metal_detector_node')
        self.treasureArrayPublisher = self.create_publisher(MarkerArray, '/minesweeper/treasure_markers', 10)
        self._last_tool_rotate_state: JointState | None = None
        # Frames config - use the sensor frame as source for marker placement; z will be forced to 0
        self._source_frame = 'VSV/Kision_sensor' 
        self.world_frame = 'world'

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.metalDetectorSub = self.create_subscription(
            Float32,
            '/vrep/metalDetector',
            self._metal_detector_callback,
            10
        )

        # clustering state (persistent markers — no expiry)
        self._cluster_distance_threshold = 4.0  # meters
        self._clusters = {}  # id -> {centroid: (x,y,z), n: int, sum: [sx,sy,sz]}
        self._next_cluster_id = 1

    def _metal_detector_callback(self, msg: Float32):
        """Handle metal detector reading: fetch last tool joint positions and optionally publish a marker."""
        positions = self.get_frame_positions()
        if positions is None:
            self.get_logger().warn('Metal detector triggered but no frame coordinates available for source frame.')
            return

        # Try to get the current position of the source frame (search common world frames)
        result = self.get_frame_coordinates(self._source_frame)

        # Log reading and resolved frame position
        if result is not None:
            xyz, frame_used = (result[0], result[1], result[2]), result[3]
            self.get_logger().info(f'Metal detector value={msg.data:.3f} | frame {self._source_frame} in {frame_used} = {xyz} | joints={positions}')
        else:
            self.get_logger().warn(f'Could not resolve TF for {self._source_frame}; using origin as fallback. Joints={positions}')
            xyz, frame_used = (0.0, 0.0, 0.0), self.world_frame

        if msg.data > 0.2:  # arbitrary threshold
            # Interpret msg.data as strength in [0..1] where higher = closer.
            # Map strength -> estimated distance (meters). We assume a simple inverse mapping:
            # distance = max_distance * (1 - strength). This is an assumption and can be tuned.
            strength = float(msg.data)
            est_distance = (1.0 - strength)

            # Try to get full transform including rotation so we can project along the sensor's forward axis.
            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, self._source_frame, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                sensor_pos = (t.x, t.y, t.z)
                # assume sensor forward is +X in the sensor frame
                forward_sensor = (1.0, 0.0, 0.0)
                forward_world = self._rotate_vector_by_quaternion(forward_sensor, q)
                detect_x = sensor_pos[0] + forward_world[0] * est_distance
                detect_y = sensor_pos[1] + forward_world[1] * est_distance
                detect_z = 0.0
            except TransformException:
                # fallback to using the translation-only xyz we resolved earlier (already handled)
                self.get_logger().warn('TF lookup for rotation failed; using translation-only position for detection point.')
                detect_x, detect_y, detect_z = float(xyz[0]), float(xyz[1]), 0.0

            # Add detection to clustering (persistent clusters), pass precision as estimated distance
            cluster_id = self._add_detection((detect_x, detect_y, detect_z), precision=est_distance)

            # Build MarkerArray with one center-sphere per cluster (no labels)
            marker_array = MarkerArray()
            now_msg = self.get_clock().now().to_msg()
            for cid, c in self._clusters.items():
                # Use marker_center (best-known location) for display; fall back to centroid
                mx, my, mz = c.get('marker_center', c['centroid'])
                # marker_precision is an estimated radius that should contain the detected point.
                # We'll scale the sphere to 2x precision (diameter) to increase visual safety.
                precision = c.get('marker_precision', 0.2)
                diameter = max(0.5, precision * 4.0)

                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp = now_msg
                m.ns = 'treasure'
                m.id = cid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(mx)
                m.pose.position.y = float(my)
                m.pose.position.z = float(mz)
                m.pose.orientation.w = 1.0
                m.scale.x = diameter
                m.scale.y = diameter
                m.scale.z = diameter
                m.color.a = 0.6
                m.color.r = 1.0
                m.color.g = 0.84
                m.color.b = 0.0
                marker_array.markers.append(m)

            self.treasureArrayPublisher.publish(marker_array)
            self.get_logger().info(f'Detection added to cluster {cluster_id}; published {len(marker_array.markers)} persistent markers.')

    def _rotate_vector_by_quaternion(self, v, q):
        """Rotate vector v (tuple) by quaternion q (geometry_msgs Quaternion-like with x,y,z,w)."""
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # t = 2 * cross(q_vec, v)
        tx = 2.0 * (qy * v[2] - qz * v[1])
        ty = 2.0 * (qz * v[0] - qx * v[2])
        tz = 2.0 * (qx * v[1] - qy * v[0])
        # v' = v + qw * t + cross(q_vec, t)
        vpx = v[0] + qw * tx + (qy * tz - qz * ty)
        vpy = v[1] + qw * ty + (qz * tx - qx * tz)
        vpz = v[2] + qw * tz + (qx * ty - qy * tx)
        # normalize to unit vector to be safe
        norm = math.sqrt(vpx * vpx + vpy * vpy + vpz * vpz)
        if norm == 0.0:
            return (1.0, 0.0, 0.0)
        return (vpx / norm, vpy / norm, vpz / norm)

    def _add_detection(self, p, precision: float = 0.5):
        """Add a detection point p=(x,y,z) with an estimated precision (radius in meters) to nearest cluster or create a new one.

        If the detection is assigned to an existing cluster, update centroid as before.
        Also maintain per-cluster 'marker_center' and 'marker_precision' where marker_center is only updated when
        the new detection has a better (smaller) precision than the stored marker_precision.

        Returns the cluster id.
        """
        px, py, pz = p
        best_id = None
        best_d = None
        for cid, c in self._clusters.items():
            cx, cy, cz = c['centroid']
            d = math.hypot(px - cx, py - cy)
            if best_d is None or d < best_d:
                best_d = d
                best_id = cid

        if best_id is not None and best_d is not None and best_d <= self._cluster_distance_threshold:
            # assign to existing cluster
            c = self._clusters[best_id]
            n = c['n']
            # update running sums and centroid
            c['sum'][0] += px
            c['sum'][1] += py
            c['sum'][2] += pz
            c['n'] = n + 1
            c['centroid'] = (c['sum'][0] / c['n'], c['sum'][1] / c['n'], c['sum'][2] / c['n'])

            # Update marker precision/center only if the new detection has better precision (smaller radius)
            existing_prec = c.get('marker_precision')
            # Update when new precision is better (smaller) or equal — treat equal as an update to keep previous behavior
            if existing_prec is None or precision <= existing_prec:
                c['marker_precision'] = precision
                c['marker_center'] = (px, py, pz)

            return best_id
        else:
            # create new cluster
            cid = self._next_cluster_id
            self._next_cluster_id += 1
            # Initialize cluster with marker_precision == precision and marker_center == detection
            self._clusters[cid] = {
                'centroid': (px, py, pz),
                'n': 1,
                'sum': [px, py, pz],
                'marker_center': (px, py, pz),
                'marker_precision': float(precision),
            }
            return cid

    def get_frame_coordinates(self, source_frame: str):
        """Try to resolve source_frame into a common world frame and return (x,y,z,frame_used).

        It attempts common frames in order: 'map', 'world', 'odom'. Returns None if none available.
        """
        try:
            if not self.tf_buffer.can_transform(self.world_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.3)):
                return None
            tf = self.tf_buffer.lookup_transform(self.world_frame, source_frame, rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z, self.world_frame)
        except TransformException:
            return None

    def get_frame_positions(self):
        """Return (x,y,z) position of the configured source frame in a common world frame, or None.

        This is a small convenience wrapper around get_frame_coordinates.
        """
        res = self.get_frame_coordinates(self._source_frame)
        if res is None:
            return None
        x, y, z, _ = res
        return (x, y, z)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()