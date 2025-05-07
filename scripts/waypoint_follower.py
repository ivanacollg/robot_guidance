#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')

        # Velocity gains
        self.vx = rospy.get_param('~vx', 0.5)
        self.vy = rospy.get_param('~vy', 0.5)
        self.vz = rospy.get_param('~vz', 0.5)
        self.vyaw = rospy.get_param('~vyaw', 0.3)

        # Max speeds
        self.max_vx = rospy.get_param('~max_vx', 1.0)
        self.max_vy = rospy.get_param('~max_vy', 1.0)
        self.max_vz = rospy.get_param('~max_vz', 0.5)
        self.max_vyaw = rospy.get_param('~max_vyaw', 1.0)

        # Thresholds
        self.pos_threshold = rospy.get_param('~pos_threshold', 0.1)
        self.yaw_threshold = rospy.get_param('~yaw_threshold', 0.05)

        # State 
        self.current_position = None
        self.current_yaw =0.0
        self.target_position = None
        self.target_yaw = 0.0

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.waypoint_callback)
        rospy.Subscriber('/bruce/slam/slam/odom', Odometry, self.odom_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def waypoint_callback(self, msg):
        self.target_position = msg.pose.position

        # Extract desired yaw from orientation
        orientation_q = msg.pose.orientation
        _, _, self.target_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def control_loop(self, event):
        if self.current_position is None or self.target_position is None:
            return

        # Compute position difference
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        dz = self.target_position.z - self.current_position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # Compute yaw difference
        dyaw = self.target_yaw - self.current_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # Normalize

        # Check thresholds
        if dist < self.pos_threshold and abs(dyaw) < self.yaw_threshold:
            cmd = Twist()
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.z = 0
            self.cmd_pub.publish(cmd)  # Stop
            rospy.loginfo_throttle(2.0, "Reached waypoint and desired yaw.")
            return

        # Compute clipped velocities
        vx_cmd = max(-self.max_vx, min(self.vx * dx, self.max_vx))
        vy_cmd = max(-self.max_vy, min(self.vy * dy, self.max_vy))
        vz_cmd = max(-self.max_vz, min(self.vz * dz, self.max_vz))
        vyaw_cmd = max(-self.max_vyaw, min(self.vyaw * dyaw, self.max_vyaw))

        # Publish command
        cmd = Twist()
        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd
        cmd.linear.z = vz_cmd
        cmd.angular.z = vyaw_cmd
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass