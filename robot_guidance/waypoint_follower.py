#!/usr/bin/env python3
import math
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Velocity gains (declare parameters with defaults)
        self.declare_parameter('vx', 0.5)
        self.declare_parameter('vy', 0.5)
        self.declare_parameter('vz', 0.5)
        self.declare_parameter('vyaw', 0.3)

        self.declare_parameter('max_vx', 1.0)
        self.declare_parameter('max_vy', 1.0)
        self.declare_parameter('max_vz', 0.5)
        self.declare_parameter('max_vyaw', 1.0)

        self.declare_parameter('pos_threshold', 0.1)
        self.declare_parameter('yaw_threshold', 0.05)

        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.vz = self.get_parameter('vz').value
        self.vyaw = self.get_parameter('vyaw').value

        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_vz = self.get_parameter('max_vz').value
        self.max_vyaw = self.get_parameter('max_vyaw').value

        self.pos_threshold = self.get_parameter('pos_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value

        # State variables
        self.current_position = None
        self.current_yaw = 0.0
        self.target_position = None
        self.target_yaw = 0.0

        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.waypoint_callback, 10)
        self.create_subscription(Odometry, '/bruce/slam/slam/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop, 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def waypoint_callback(self, msg: PoseStamped):
        self.target_position = msg.pose.position

        orientation_q = msg.pose.orientation
        r = R.from_quat([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        _, _, self.target_yaw = r.as_euler('xyz')

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position

        orientation_q = msg.pose.pose.orientation
        r = R.from_quat([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        _, _, self.current_yaw = r.as_euler('xyz')

    def control_loop(self):
        if self.current_position is None or self.target_position is None:
            return

        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        dz = self.target_position.z - self.current_position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        dyaw = self.target_yaw - self.current_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # Normalize angle

        if dist < self.pos_threshold and abs(dyaw) < self.yaw_threshold:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Reached waypoint and desired yaw.")
            return

        vx_cmd = max(-self.max_vx, min(self.vx * dx, self.max_vx))
        vy_cmd = max(-self.max_vy, min(self.vy * dy, self.max_vy))
        vz_cmd = max(-self.max_vz, min(self.vz * dz, self.max_vz))
        vyaw_cmd = max(-self.max_vyaw, min(self.vyaw * dyaw, self.max_vyaw))

        cmd = Twist()
        cmd.linear.x = vx_cmd
        cmd.linear.y = vy_cmd
        cmd.linear.z = vz_cmd
        cmd.angular.z = vyaw_cmd

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
