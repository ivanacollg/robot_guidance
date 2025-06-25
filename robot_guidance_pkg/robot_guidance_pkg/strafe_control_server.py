import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_guidance_interfaces.action import GoToSide
from rclpy.executors import MultiThreadedExecutor # For multithreading
from rclpy.callback_groups import ReentrantCallbackGroup # For multithreading
from scipy.spatial.transform import Rotation
from rclpy.clock import Clock
import numpy as np
import math

# Cancel all goals
# ros2 service call /go_to_side/_action/cancel_goal action_msgs/srv/CancelGoal "{}"


class StrafeControlServer(Node):
    """
    Server node for GoToSide action (Strafe action).

    Args:
        Node:
            Make this class a ROS2 Node
    """
    def __init__(self):
        """
        Initializes the server.

        Args:
            self:
                The server node
        
        Returns:
            None: None
        """
        super().__init__('strafe_control_server')

        # Declare parameters with defaults
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('max_velocity', 0.25)
        self.declare_parameter('timeout', 15.0)
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
            GoToSide,
            'go_to_side',
            goal_callback=self.goal_callback,   
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            #callback_group=ReentrantCallbackGroup()
        )

        self.current_y = None  # z from odometry


    def odom_callback(self, msg):
        """
        Updates the known current position of the robot when a new odometry message is received.
        
        Args:
            self:
                The server node
            msg:
                A message of type Odometry from Nav2
        
        Returns:
            None: None
        """
        #self.get_logger().info('Getting Odometry...')
        self.current_pose = msg.pose.pose


    def goal_callback(self, goal_request):
        """
        Called when the server receives a goal request from a client. \n
        Automatically accepts all goal requests and sends an accept goal response.

        Args:
            self:
                The server node
            goal_request:
                The goal request sent by the client

        Returns:
            None: None
        """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):
        """
        Called when the client requests to cancel a goal request. \n
        Responds to the client with an accept or reject cancel reponse message.

        Args:
            self:
                The server node
            goal_handle:
                The server goal handle of the request to be canceled
        
        Returns: 
            An accepted or rejected CancelResponse 
        """
        self.get_logger().info('Received cancel request')
        self.stop()
        return CancelResponse.ACCEPT


    def compute_distance(self, pose1, pose2):
        """
        Computes the distance between two poses (neglecting the z axis).

        Args:
            self:
                The server node
            pose1:
                The starting pose
            pose2:
                The ending pose

        Returns:
            float:
                The distance between pose1 and pose2
        """
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        #dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy) #+ dz*dz)
    

    def compute_relative_y(self, robot_pose, goal_pose):
        """
            Computes the relative yaw of the robot.
        
        Args:
            self:
                The server node
            robot_pose:
                The current position of the robot
            goal_pose:
                The position of the goal 

        Returns:
            float:
                The relative yaw of the robot
        """
        # Vector from robot to target in global frame
        dx = goal_pose.position.x - robot_pose.position.x
        dy = goal_pose.position.y - robot_pose.position.y

        q = robot_pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        # Transform to robot frame (rotate the vector by -theta_r)
        # Robot frame: X points forward, Y points to the left
        y_robot = -math.sin(yaw) * dx + math.cos(yaw) * dy

        return y_robot


    def execute_callback(self, goal_handle):
        """
        Executres the GoToSide goal (strafe goal) request sent by the client.

        Args:
            self:
                The server node
            goal_handle:
                The server goal handle for the goal request in progress
        
        Returns:
            bool:
                The result of the server operation. \n
                True if the strafe is successful. \n
                False otherwise. 
        """
        self.get_logger().info('Executing goal...')
        # initialize variables
        target_pose = goal_handle.request.target_pose.pose
        #rate = self.create_rate(10)
        min_distance_error = np.inf
        start_time = Clock().now()

        while rclpy.ok():
            current_time = Clock().now()
            start_secs = start_time.nanoseconds / 1e9
            current_secs = current_time.nanoseconds / 1e9
            elapsed = current_secs - start_secs

            if elapsed > self.timeout:
                self.get_logger().warn("Server Request Timed Out")
                self.stop()
                goal_handle.abort()
                return GoToSide.Result(target_reached=False)
            
            if self.current_pose is None:
                self.get_logger().warn('Waiting for valid y from odometry...')
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    self.stop()
                    goal_handle.canceled()
                    return GoToSide.Result(target_reached=False)
                
                rclpy.spin_once(self, timeout_sec=0.1)
                #rate.sleep()
                continue

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                self.stop()
                goal_handle.canceled()
                return GoToSide.Result(target_reached=False)
            
            distance_error = self.compute_distance(target_pose, self.current_pose)
            self.get_logger().info(
                f'Current Pose: {self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, '
                f'Desired Pose: {target_pose.position.x:.2f}, {target_pose.position.y:.2f}, '
                f'Distance Error: {distance_error:.2f}, '
                f'Min Distance Error: {min_distance_error:.4f}')

            if abs(distance_error) < self.tolerance:
                self.stop()
                self.get_logger().info('Pose Reached.')
                goal_handle.succeed()
                result = GoToSide.Result()
                result.target_reached = True
                return result
            
            relative_y = self.compute_relative_y(self.current_pose, target_pose)

            # robot has passed by closest point to goal on current path
            if (distance_error > min_distance_error + self.error_buffer):
                self.get_logger().warn('Goal Missed')
                self.stop()
                self.get_logger().info(f'Current Distance Error: {distance_error:.3f} '
                                       f'Minimum Distance Error: {min_distance_error:.3f}')
                goal_handle.succeed()
                result = GoToSide.Result()
                result.target_reached = True
                return result
            # robot getting closer to goal on current path
            elif (distance_error < min_distance_error):
                min_distance_error = distance_error
            # else robot is getting farther away from goal but still within error buffer

            feedback_msg = GoToSide.Feedback()
            feedback_msg.distance_remaining = distance_error
            goal_handle.publish_feedback(feedback_msg)

            cmd = Twist()
            cmd.linear.y = max(min(self.Kp * np.sign(relative_y)*distance_error, self.max_velocity), -self.max_velocity)
            self.cmd_pub.publish(cmd)
            #rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop()
        goal_handle.abort()
        return GoToSide.Result(target_reached=False)


    def stop(self):
        """
        Stops the robot by setting and publising velocities of 0.
        
        Args:
            self:
                The server node
        
        Returns:
            None: None
        """
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Stopping.')


def main(args=None):
    rclpy.init(args=args)
    server = StrafeControlServer()
    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
