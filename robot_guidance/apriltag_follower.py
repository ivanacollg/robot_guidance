
#!/usr/bin/env python3

import sys
import numpy as np

# OpenCV
from cv_bridge import CvBridge
import cv2

# April Tags
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R

# Ros
import rclpy
from rclpy.node import Node

# Ros Messages
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class AprilTagFollower(Node):

    def __init__(self):
        super().__init__('apriltag_follower_node')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('image_sub')
        image_topic = self.get_parameter('image_sub').get_parameter_value().string_value

        # Publishers and Subscribers
        self.image_sub = self.create_subscription(Image, image_topic, self.rgb_callback, 1)
        self.image_pub = self.create_publisher(Image, '/apriltag_follower/image', 1)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 1)

        # April Tag Params
        self.tag_size = 0.17
        # Camera Params
        fx, fy = 1424.476643, 1419.605906
        cx, cy = 878.117318, 597.690257
        self.camera_params = [fx, fy, cx, cy]

        # Initialize April Tag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Define the rotation matrix that maps tag detector axis to robot axis
        # Robot axis
        # x --> Forward
        # y -> Right
        # z -> Up
        self.R_map = np.array([
            [0, 0, 1],  # X' = Y
            [1, 0, 0],  # Y' = -Z
            [0, -1, 0]  # Z' = X
        ])

    def detect_tag(self, image, stamp):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect Tags
        detections = self.detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'camera'

        # Display results
        for detection in detections:
            # Rotate pose into new frame
            R_new = detection.pose_R
            t_new = self.R_map @ detection.pose_t

            # Convert rotation matrix to Euler angles (in degrees, optional)
            rot = R.from_matrix(R_new)
            quat = rot.as_quat()  # [x, y, z, w]
            rpy = rot.as_euler('xyz', degrees=True)  # or 'zyx' depending on your convention

            pitch, yaw, roll = rpy

            self.get_logger().info(f"Tag ID: {detection.tag_id}")
            self.get_logger().info(f"Pose (translation): {t_new.flatten()}")
            self.get_logger().info(f"Pose (RPY): Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

            pose_msg.pose.position.x = float(t_new[0]) - 0.5
            pose_msg.pose.position.y = float(t_new[1])
            pose_msg.pose.position.z = float(t_new[2])

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # Draw detection outline
            for i in range(4):
                pt1 = tuple(detection.corners[i - 1, :].astype(int))
                pt2 = tuple(detection.corners[i, :].astype(int))
                cv2.line(image, pt1, pt2, (255, 0, 0), 2)

            # Draw center
            center = tuple(detection.center.astype(int))
            cv2.circle(image, center, 5, (0, 0, 255), -1)

        # Publish
        self.waypoint_pub.publish(pose_msg)
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        img_msg.header = pose_msg.header
        self.image_pub.publish(img_msg)

    def rgb_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_tag(image, msg.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
