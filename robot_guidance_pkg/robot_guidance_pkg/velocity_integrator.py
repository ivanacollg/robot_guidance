#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster

class VelocityIntegrator(Node):
    def __init__(self):
        super().__init__('velocity_integrator')

         # Declare topics with defaults
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(Twist, cmd_vel_topic, self.velocity_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()
        self.current_velocity = Twist()

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer at 20 Hz
        self.create_timer(0.05, self.update_odometry)

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        vx = self.current_velocity.linear.x
        vy = self.current_velocity.linear.y
        vz = self.current_velocity.linear.z
        vyaw = self.current_velocity.angular.z

        # Integrate in world frame
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
        odom.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        quat = R.from_euler('z', self.yaw).as_quat()  # x, y, z, w
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        odom.twist.twist = self.current_velocity

        self.odom_pub.publish(odom)

        # Publish transform from odom to base_link
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()