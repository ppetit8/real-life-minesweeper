#!/usr/bin/env python3

import tensorflow_models_base.venv_hack

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

import tensorflow as tf
import numpy as np

import cv2
# cv_bridge2 is a local version of cv_bridge compiled with numpy 2
from cv_bridge2 import CvBridge, CvBridgeError

import struct
from sensor_msgs_py import point_cloud2  # ROS2 built-in parser

class ShoreFollowerDrive(Node):
    def __init__(self):
        super().__init__("classify")

        self.declare_parameter('~/model_dir', ".")
        self.declare_parameter('~/linear_vel', 1.0)
        self.model_dir_ = self.get_parameter('~/model_dir').get_parameter_value().string_value
        self.linear_vel_ = self.get_parameter('~/linear_vel').get_parameter_value().double_value

        self.load_model()

        self.br = CvBridge()
        self.twist_pub_ = self.create_publisher(Twist,"~/twist",1)
        self.image_sub_ = self.create_subscription(Image,"~/image", self.image_callback, 1)
        # store last received PointCloud2 for use during decision making
        self.last_points_msg = None
        self.points_sub_ = self.create_subscription(PointCloud2, "/points", self.points_callback, 1)


    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_)

    def image_callback(self, data):
        # Image call back and resize to proper shape
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.br.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0)
        self.twist_pub_.publish(self.image_to_rot(processed_))


    def get_ground_stats(self):
        """
        Returns (min_z, max_z) extracted from the last pointcloud.
        Returns None if no valid pointcloud received yet.
        """
        if self.last_points_msg is None:
            return None

        # point_cloud2.read_points returns a generator over the points
        points = point_cloud2.read_points(
            self.last_points_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        min_z = float('inf')
        max_z = -float('inf')

        for (_, _, z) in points:
            if z < min_z:
                min_z = z
            if z > max_z:
                max_z = z

        if min_z == float('inf'):
            return None  # no points

        return (min_z, max_z)


    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.model(img, training=False)[0]
        # Makes sure that the shape of the network matches the required shape
        assert(res.shape[0] == 3)
        print("%5.2f %5.2f %5.2f" %(res[0],res[1],res[2]))
        #TODO: Use the network output so the robot can drive around the lake
        # returns a geometry_msgs.Twist
        if res[0] > res[1] and res[0] > res[2]:
            out.linear.x = -self.linear_vel_
        elif res[2] > res[1] and res[2] > res[0]:
            out.linear.x = self.linear_vel_
        
        stats = self.get_ground_stats()
        if stats is not None:
            min_z, max_z = stats

            # Example strategy:
            # If ground is too close (min_z < threshold), go up
            # If ground is far (max_z > threshold), go down

            ground_close_threshold = 0.3
            ground_far_threshold = 0.4

            if min_z < ground_close_threshold:
                out.linear.z = 0.3   # go up
            elif max_z > ground_far_threshold:
                out.linear.z = -0.3   # go down
            else:
                out.linear.z = 0.0
            print("min_z: %5.2f, max_z: %5.2f" % (min_z, max_z))
        print("Driving with linear.x = %5.2f, linear.z = %5.2f" % (out.linear.x, out.linear.z))
        return out

    def points_callback(self, msg: PointCloud2):
        """Simple callback to store the last received PointCloud2 message.

        Keep the raw message so callers can parse it when needed. We intentionally avoid
        expensive conversion here; do that lazily in decision code if required.
        """
        self.last_points_msg = msg


def main(args=None):
    rclpy.init(args=args)

    drive = ShoreFollowerDrive()

    rclpy.spin(drive)

    face_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
