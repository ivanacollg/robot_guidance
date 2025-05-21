#!/usr/bin/env python3

# Python libs
import sys, time
import threading
import struct
import os

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import ros_numpy

# OpenCV
import cv2
import cv_bridge

# Ros libraries
import roslib
import rospy
import rospkg

# Apriltag
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R

# Ros Messages
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image
from message_filters import Subscriber
from std_msgs.msg import Header

import matplotlib.pyplot as plt


class apriltagFollower:

    def __init__(self, ns="~"):
        self.BridgeInstance = cv_bridge.CvBridge()
        # Publishers and Subscribers
        #self.image_sub = rospy.Subscriber(rospy.get_param(ns + "image_sub"), Image, self.rgb_callback,  queue_size = 1)
        self.image_sub = rospy.Subscriber(rospy.get_param(ns + "image_sub"), Image, self.rgb_callback,  queue_size = 1)
        self.image_pub = rospy.Publisher("/apriltag_follower/image", Image, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        self.tag_size = 0.16
        #fx, fy = 1048.9128, 1044.4716  # Focal lengths in pixels
        #cx, cy = 938.53824, 495.39543  # Principal point (image center)
        fx, fy = 1424.476643, 1419.605906  # Focal lengths in pixels
        cx, cy = 878.117318, 597.690257  # Principal point (image center)
        self.camera_params = [fx, fy, cx, cy]

        # Initialize detector
        self.detector = Detector(families='tag36h11',
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
        # Define the rotation matrix that maps tag detector axis to robot axis
        # Robot axis
        # x --> Forward
        # y -> Right
        # z -> Up
        self.R_map = np.array([
            [0, 0, 1],    # X' = Y
            [1, 0, 0],   # Y' = -Z
            [0, -1, 0]     # Z' = X
        ])


    def detect_tag(self, image, stamp):
        #cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect tags
        detections = self.detector.detect(gray_image, estimate_tag_pose=True,
                                    camera_params=self.camera_params, tag_size=self.tag_size)
       # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "camera"  # <-- set this appropriately

       
        # Display results
        for detection in detections:
            # Rotate pose into new frame
            R_new = detection.pose_R #self.R_map @ detection.pose_R
            t_new = self.R_map @ detection.pose_t

            # Convert rotation matrix to Euler angles (in degrees, optional)
            rot = R.from_matrix(R_new)
            quat = rot.as_quat()  # [x, y, z, w]
            rpy = rot.as_euler('xyz', degrees=True)  # or 'zyx' depending on your convention
            #change axis
            pitch, yaw, roll = rpy

            print(f"Tag ID: {detection.tag_id}")
            print("Pose (translation):", t_new)
            print(f"Pose (RPY): Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
            #print("Pose (rotation matrix):\n", R_new)

            pose_msg.pose.position.x = t_new[0]-0.5
            pose_msg.pose.position.y = t_new[1]
            pose_msg.pose.position.z = t_new[2]

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
        self.image_pub.publish(ros_numpy.msgify(Image, image, encoding="bgr8"))

    
    def rgb_callback(self, msgImg):
        # Uncompressed Image
        image = self.BridgeInstance.imgmsg_to_cv2(msgImg, "bgr8") 
        #image = self.BridgeInstance.compressed_imgmsg_to_cv2(msgImg, "bgr8")
        self.detect_tag(image, msgImg.header.stamp)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('apriltag_follower_node', anonymous=True)
    node = apriltagFollower()
    rospy.loginfo("Start apriltag_follower node...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down apriltag follower node")
 
if __name__ == '__main__':
    main(sys.argv)