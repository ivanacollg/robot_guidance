import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_guidance_interfaces.action import GoToSide

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math

class StrafeControlServer(Node):
    def __init__(self):
        super().__init__('strafe_control_server')

        # Declare parameters with defaults
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('max_velocity', 0.25)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        # Declare topics with defaults
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        # Publishers and Subcribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        # Action Server
        self._action_server = ActionServer(
            self,
            GoToSide,
            'go_to_side',
            goal_callback=self.goal_callback,   
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.current_y = None  # z from odometry
        self._active = False

    def odom_callback(self, msg):
        self.current_y = msg.pose.pose.position.y

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    # ros2 action send_goal /go_to_side robot_guidance_interfaces/action/GoToSide "{target_y: 1.5}"  - sending a goal
    # ros2 service call /go_to_side/_action/cancel_goal action_msgs/srv/CancelGoal "{}"  - Cancels all goals

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.stop()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_y = float(goal_handle.request.target_y)

        rate = self.create_rate(10)
        self.active = True

        while rclpy.ok() and self.active:
            if self.current_y is None:
                self.get_logger().warn('Waiting for valid y from odometry...')
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    self.stop()
                    goal_handle.canceled()
                    return GoToSide.Result(target_reached=False)
                
                rate.sleep()
                continue

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                self.stop()
                goal_handle.canceled()
                return GoToSide.Result(target_reached=False)

            error = target_y - self.current_y
            self.get_logger().info(f'Current Z: {self.current_y:.2f}, Target: {target_y}, Error: {error:.2f}')

            if abs(error) < self.tolerance:
                self.stop()
                self.get_logger().info('Y Reached.')
                goal_handle.succeed()
                result = GoToSide.Result()
                result.target_reached = True
                return result

            feedback_msg = GoToSide.Feedback()
            feedback_msg.current_y = self.current_y
            goal_handle.publish_feedback(feedback_msg)

            cmd = Twist()
            cmd.linear.y = max(min(self.Kp * error, self.max_velocity), -self.max_velocity)
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop()
        goal_handle.abort()
        return GoToSide.Result(target_reached=False)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Stopping.')

def main(args=None):
    rclpy.init(args=args)
    server = StrafeControlServer()
    try:
        rclpy.spin(server, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
