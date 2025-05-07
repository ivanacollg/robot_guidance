#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import time

class VelocityIntegrator:
    def __init__(self):
        rospy.init_node('velocity_integrator')

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.last_time = rospy.Time.now()
        self.current_velocity = Twist()

        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.update_odometry)  # 20 Hz

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def update_odometry(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        vx = self.current_velocity.linear.x
        vy = self.current_velocity.linear.y
        vz = self.current_velocity.linear.z
        vyaw = self.current_velocity.angular.z

        # Compute delta position in global frame
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        dz = vz * dt
        dyaw = vyaw * dt

        self.x += dx
        self.y += dy
        self.z += dz
        self.yaw += dyaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Normalize

        # Build odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        q = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist = self.current_velocity  # optionally include raw velocities

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        VelocityIntegrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass