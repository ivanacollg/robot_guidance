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

# YOLO
from ultralytics import YOLO

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
from message_filters import Subscriber
from std_msgs.msg import Header

import matplotlib.pyplot as plt


class yoloDetect:

    def __init__(self, ns="~"):
        self.BridgeInstance = cv_bridge.CvBridge()
        # Publishers and Subscribers
        #self.image_sub = rospy.Subscriber(rospy.get_param(ns + "image_sub"), Image, self.rgb_callback,  queue_size = 1)
        self.image_sub = rospy.Subscriber(rospy.get_param(ns + "image_sub"), CompressedImage, self.rgb_callback,  queue_size = 1)
        self.image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=1)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("robot_guidance")  # Replace with your package name
        model_path = os.path.join(pkg_path, "models", "best.pt")     
        
        self.yolo_model = YOLO(model_path)

        self.stamp = None
        self.odom = None
        self.odom_new = None
        self.img = np.zeros(0)
        self.img_new = None

        # the threading lock
        self.lock = threading.RLock()


    def detect_fence(self, image, stamp):
        if (image is not None): 
            # Get positions
            det_result = self.yolo_model(image)
            det_annotated = det_result[0].plot(show=False)
            self.image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="bgr8"))


    
    def rgb_callback(self, msgImg):
        # Uncompressed Image
        #self.img_new = self.BridgeInstance.imgmsg_to_cv2(msgImg, "bgr8") 
        image = self.BridgeInstance.compressed_imgmsg_to_cv2(msgImg, "bgr8")
        self.detect_fence(image, msgImg.header.stamp)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('yolo_detect_node', anonymous=True)
    node = yoloDetect()
    rospy.loginfo("Start yolo detect node...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down yolo_detect node")
 
if __name__ == '__main__':
    main(sys.argv)