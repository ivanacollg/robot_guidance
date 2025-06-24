#!/usr/bin/env python3

import math
#import asyncio
import time
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import DriveOnHeading
from builtin_interfaces.msg import Duration
from robot_guidance_interfaces.action import GoToDepth, GoToSide, NavigateAprilTags # Custom action

from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation  

class AprilTagNavigation(Node):
    """
    Server node for path navigation 

    Args:
        Node: Make this class a ROS2 Node 

    Returns:
        None: None

    """
    def __init__(self, navigator):
        """
        Initiliazes the server

        Args:
            self: 
                The server node
            navigator:
                Later set to a Nav2 Basic Navigator node

        Returns:
            None: None

        """
        super().__init__('apriltag_navigation_server')
        # Get topic names
        self.declare_parameter('odom_topic', '/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        # self.goal_tolerance = 0.05
        
        # Subscribers 
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Basic Navigator for moving to a pose
        self.navigator = navigator
        
        self._action_server = ActionServer(
            self,
            NavigateAprilTags,
            'navigate_apriltags',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # GoToDepth  and DriveOnHeading action clients
        self.depth_control_client = ActionClient(self, GoToDepth, 'go_to_depth')
        self.strafe_control_client = ActionClient(self, GoToSide, 'go_to_side')
        # Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Variable Init
        self.current_pose = None
    

    def odom_callback(self, msg: Odometry):
        """
        Updates the known current position of the robot when a new odometry message is received
        
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
        Called when the server receives a goal request from a client
        Automatically accepts all goal requests and sends an accept goal response

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
        Called when the client requests to cancel a goal request, responds to the client with an accept or reject cancel reponse message

        Args:
            self:
                The server node
            goal_handle:
                The server goal handle of the request to be canceled

        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    

    def get_strafe_heading(self, start_pose: Pose, goal_pose: Pose, stafe_direction) -> Quaternion:
        """
        Determines the heading required for the strafe server 
        
        Args:
            self:
                The server node
            start_pose: 
                The starting robot pose
            goal_pose:
                The goal pose at the end of the strafe 
            strafe_direction:
                The direction in which the robot faces while strafing
        
        Returns:
            A Quaternion that is the required heading for the strafe server 
                
        """
        strafe_left = False
        if stafe_direction == "left":
            strafe_left = True
        
        dx = goal_pose.position.x - start_pose.position.x
        dy = goal_pose.position.y - start_pose.position.y

        # Direction from start to goal
        theta = math.atan2(dy, dx)

        # Rotate to face perpendicular
        if strafe_left:
            yaw = theta + math.pi / 2 
        else: # strafe right
            yaw = theta - math.pi / 2

        quat = Rotation.from_euler('z', yaw).as_quat()  # [x, y, z, w]

        q = Quaternion()
        q.x, q.y, q.z, q.w = quat
        return q
    

    def compute_distance(self, pose1, pose2):
        """
        Computes the distance between two poses (neglecting the z axis)

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


    async def execute_callback(self, goal_handle):
        """
        Executes the navigation goal request sent by the client

        Args:
            self:
                The server node

            goal_handle:
                The goal handle of the goal request the server is currently working on

        Returns:
            bool:
                The result of the server operation \n
                True if the navigation is successful. \n
                False otherwise.

        """
        goal = goal_handle.request
        goals_list = goal.goals
        command_list = goal.commands

        for i, tag_id in enumerate(goals_list[:-1]): # Loop excludes last element
            current_goal = goals_list[i]
            next_goal = goals_list[i+1]

            self.get_logger().info(f'Going to pose number {i}')
            feedback_msg = NavigateAprilTags.Feedback()
            feedback_msg.next_pose = next_goal
            feedback_msg.next_command = command_list[i+1]
            goal_handle.publish_feedback(feedback_msg)

            # 3. Send initial goToPose
            # Build navigation goal from transform
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = current_goal.pose.position.x
            pose.pose.position.y = current_goal.pose.position.y
            pose.pose.orientation = current_goal.pose.orientation
            self.navigator.goToPose(pose)

            start_time = self.get_clock().now()
            # 4. Spin while goal is not yet reached
            while rclpy.ok() and not self.navigator.isTaskComplete():
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9  # seconds
                if elapsed_time > 600:
                # If task taking too long or april tag detected
                    self.navigator.cancelTask()
                    self.get_logger().warn(f'Goal timeout without being reached {tag_id}')
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed=False) 
                

            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Failed to reach a goal.")
                goal_handle.abort()
                return NavigateAprilTags.Result(navigation_completed=False)

            # Execute command depending on Goal Pose
            self.get_logger().info(f'Reached pose number {i}, executing command...')

            command = command_list[i]

            self.get_logger().info(f'Command {command}')

            if command == 'vertical': 
                self.get_logger().info(f'Vertical Command')
                desired_depth = next_goal.pose.position.z
                depth_goal = GoToDepth.Goal()
                depth_goal.target_depth = desired_depth

                self.get_logger().info(f'Waiting for depth control server to start')
                # Wait for server to be ready
                if not self.depth_control_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("GoToDepth action server not available after 5 seconds!")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed=False)
                # Send the goal
                self.get_logger().info(f"Sending GoToDepth goal to {desired_depth:.2f} meters")
                
                send_goal_future = self.depth_control_client.send_goal_async(depth_goal)
                depth_goal_handle = await send_goal_future

                if not depth_goal_handle.accepted:
                    self.get_logger().warn("Depth goal rejected")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
            
                result_response = await depth_goal_handle.get_result_async()
                
                if not result_response.result.reached_final_depth:
                    self.get_logger().warn("Depth goal failed")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
                
                self.get_logger().info("Depth goal succeeded")
            
            elif command == 'left' or command == 'right':
                self.get_logger().info(f'Horizontal Command')
                distance = np.inf
                
                #while distance > self.goal_tolerance:
                distance = self.compute_distance(self.current_pose, next_goal.pose)
                self.get_logger().info(f"Distance {distance:.2f} meters")

                self.get_logger().info(f'Spin in place to stafing angle')
                q = self.get_strafe_heading(self.current_pose, next_goal.pose, command)

                # Build navigation goal from transform
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = self.current_pose.position.x
                pose.pose.position.y = self.current_pose.position.y
                pose.pose.orientation = q
                self.navigator.goToPose(pose)

                start_time = self.get_clock().now()
                # 4. Spin while goal is not yet reached
                while rclpy.ok() and not self.navigator.isTaskComplete():
                    elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9  # seconds
                    if elapsed_time > 600:
                    # If task taking too long
                        self.navigator.cancelTask()
                        self.get_logger().warn(f'Strafing heading goal timeout without being reached')
                        goal_handle.abort()
                        return NavigateAprilTags.Result(navigation_completed = False)
                    
                if self.navigator.getResult() != TaskResult.SUCCEEDED:
                    self.get_logger().warn(f"Failed to reach desired heading.")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
                
                strafe_goal = GoToSide.Goal()
                strafe_goal.target_pose = next_goal

                self.get_logger().info(f'Waiting for strafe control server to start')
                # Wait for server to be ready
                if not self.strafe_control_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("GoToSide action server not available after 5 seconds!")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed=False)
                # Send the goal
                self.get_logger().info(f"Sending GoToSide goal")
                
                send_goal_future = self.strafe_control_client.send_goal_async(strafe_goal)
                strafe_goal_handle = await send_goal_future

                if not strafe_goal_handle.accepted:
                    self.get_logger().warn("Strafe goal rejected")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
            
                result_response = await strafe_goal_handle.get_result_async()
                
                if not result_response.result.target_reached:
                    self.get_logger().warn("Strafe goal failed")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_completed = False)
                
                self.get_logger().info("Strafe goal succeeded")
                            
        self.get_logger().info("Navigation succeeded")
        goal_handle.succeed()
        return NavigateAprilTags.Result(navigation_completed = True)


def main(args=None):
    rclpy.init(args=args)
    # Basic Navigator for moving to a pose
    navigator = BasicNavigator()
    # waiting for Nav2Active
    navigator.waitUntilNav2Active()  # The BasicNavigator.waitUntilNav2Active() function explicitly checks for /amcl/get_state. If you're not using AMCL and that service doesn't exist, this call will block forever.
    node = AprilTagNavigation(navigator)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
