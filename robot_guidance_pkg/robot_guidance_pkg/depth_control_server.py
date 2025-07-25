import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_guidance_interfaces.action import GoToDepth

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.clock import Clock

import numpy as np

import math

# ros2 action send_goal /go_to_depth robot_guidance_interfaces/action/GoToDepth "{target_depth: 1.5}"  - sending a goal
# ros2 service call /go_to_depth/_action/cancel_goal action_msgs/srv/CancelGoal "{}"  - Cancels all goals

class DepthControlServer(Node):
    def __init__(self):
        super().__init__('depth_control_server')

        # Declare parameters with defaults
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('max_velocity', 0.25)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('error_buffer', 0.02)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.error_buffer = self.get_parameter('error_buffer').get_parameter_value().double_value
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
            GoToDepth,
            'go_to_depth',
            goal_callback=self.goal_callback,   
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            #callback_group=ReentrantCallbackGroup() # threading allows for cancelling requests
        )

        self.current_depth = None  # z from odometry
        self._active = False

    def odom_callback(self, msg):
        #self.get_logger().info('Getting Odometry...')
        self.current_depth = msg.pose.pose.position.z

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')

        if goal_request.target_depth > 0:
            self.get_logger().warn('Goal request is greater than 0')
            self.get_logger().warn('Rejecting goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle): # client calls a cancel request, goal_handle is a ServerGoalHandle
        self.get_logger().info('Received cancel request')
        self.stop()
        return CancelResponse.ACCEPT # sets goal_handle.is_cancel_requested to true

    def execute_callback(self, goal_handle): # automatically called from goal_callback func when goal is accepted, goal_handle is a ServerGoalHandle
        self.get_logger().info('Executing goal...')
        # initialize variables
        target_depth = float(goal_handle.request.target_depth) # get our action goal
        rate = self.create_rate(10)
        self.active = True
        min_error = np.inf
        start_time = Clock().now()

        while rclpy.ok() and self.active:
            current_time = Clock().now()
            start_secs = start_time.nanoseconds / 1e9
            current_secs = current_time.nanoseconds / 1e9
            elapsed = current_secs - start_secs

            if elapsed > self.timeout:
                self.get_logger().warn("Server Request Timed Out")
                self.stop()
                goal_handle.abort()
                return GoToDepth.Result(reached_final_depth=False)

            if self.current_depth is None:
                self.get_logger().warn('Waiting for valid depth from odometry...')
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    self.stop()
                    goal_handle.canceled() # Goal State = canceled
                    return GoToDepth.Result(reached_final_depth=False)
                
                rclpy.spin_once(self, timeout_sec=0.01)
                #rate.sleep()
                continue

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                self.stop()
                goal_handle.canceled()
                return GoToDepth.Result(reached_final_depth=False)

            error = target_depth - self.current_depth
            self.get_logger().info(f'Current Z: {self.current_depth:.2f}, Target: {target_depth}, Error: {error:.2f}')

            if abs(error) < self.tolerance:
                self.stop()
                self.get_logger().info('Depth Reached.')
                goal_handle.succeed() # goal state = success
                result = GoToDepth.Result()
                result.reached_final_depth = True
                return result

            # robot depth no longer approaching goal 
            if ((abs(error)) > min_error + self.error_buffer):
                 self.get_logger().warn('Goal Missed')
                 self.stop()
                 self.get_logger().info(f'Current Distance Error: {abs(error):.3f} '
                                        f'Minimum Distance Error: {min_error:.3f}')
                 goal_handle.succeed()
                 result = GoToDepth.Result()
                 result.reached_final_depth = True
                 return result
            # robot depth getting closer to target depth
            elif (abs(error) < min_error):
                min_error = abs(error)
            # else robot depth no longer approaching goal but within buffer

            feedback_msg = GoToDepth.Feedback()
            feedback_msg.current_depth = self.current_depth
            goal_handle.publish_feedback(feedback_msg)

            cmd = Twist()
            cmd.linear.z = max(min(self.Kp * error, self.max_velocity), -self.max_velocity)
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
            #rate.sleep()

        self.stop()
        goal_handle.abort()
        return GoToDepth.Result(reached_final_depth=False)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Stopping.')

def main(args=None):
    rclpy.init(args=args)
    server = DepthControlServer()
    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
        #rclpy.spin(server, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
