#!/usr/bin/env python3

import math
#import asyncio
import time
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import DriveOnHeading
from builtin_interfaces.msg import Duration
from robot_guidance_interfaces.action import GoToDepth, GoToSide, NavigateAprilTags # Custom action

from tf2_ros import TransformException, Buffer, TransformListener
from scipy.spatial.transform import Rotation  

class AprilTagNavigation(Node):

    def __init__(self):
        super().__init__('apriltag_navigation_server')
        # Get topic nameSs
        self.declare_parameter('tag_detections_topic', '/tag_detections')
        self.declare_parameter('odom_topic', '/odom')
        tag_topic = self.get_parameter('tag_detections_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        # Subscribers 
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            tag_topic,
            self.tag_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        # Basic Navigator for moving to a pose
        self.navigator = BasicNavigator()
        # waiting for Nav2Active
        self.navigator.waitUntilNav2Active()  # The BasicNavigator.waitUntilNav2Active() function explicitly checks for /amcl/get_state. If you're not using AMCL and that service doesn't exist, this call will block forever.
        
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
        self.current_detections = {}
        self.latest_odom = None
        # April Tag Type
        self.tag_family = 'tag36h11'
        self.goal_offset = 0.5
    
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def tag_callback(self, msg: AprilTagDetectionArray):
        self.current_detections = {
            f'{detection.family}:{detection.id}': detection for detection in msg.detections
            }

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        id_list = goal.ids
        command_list = goal.commands

        for i, tag_id in enumerate(id_list[:-1]): # Loop excludes last element
            next_tag_id = id_list[i+1]
            tag_frame = f'{self.tag_family}:{tag_id}'
            # If desired April tag is not detected, navigate to it
            #if tag_frame not in self.current_detections:
            # 1. Get initial transform from map to tag frame (assumed known static TF)
            # Wait until the tag appears in tf
            self.get_logger().info(f'Waiting for transform from map to {tag_frame}...')
            transform = None
            while rclpy.ok():
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        tag_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    break
                except TransformException as ex:
                    self.get_logger().warn(f'Waiting for transform {tag_frame}: {ex}')
                    time.sleep(0.5)#await asyncio.sleep(0.5)#await rclpy.sleep(0.5)

            if transform is None:
                self.get_logger().warn(f'Transform to {tag_frame} not found.')
                goal_handle.abort()
                return NavigateAprilTags.Result(navigation_complete=False)
            
            self.get_logger().info(f'Transform recived {tag_frame}, publishing global goal to it.')
            # 2. Compute initial offset goal
            # Build navigation goal from transform but with a 0.5 meter offset in x
            # Extract translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            # Extract rotation as quaternion
            q = transform.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            yaw = Rotation.from_quat(quat).as_euler('xyz')[2]  # only yaw

            # Offset 0.5 meters forward (along tag's x-axis)
            goal_x = tx - self.goal_offset * np.cos(yaw)
            goal_y = ty - self.goal_offset * np.sin(yaw)

            # 3. Send initial goToPose
            # Build navigation goal from transform
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = goal_x
            pose.pose.position.y = goal_y
            pose.pose.orientation = q
            go_to_pose_task = self.navigator.goToPose(pose)

            start_time = self.get_clock().now()
            # 4. Spin while tag is not yet detected and goal not reached
            while rclpy.ok() and not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9  # seconds
                if elapsed_time > 600:
                # If task taking too long or april tag detected
                    self.navigator.cancelTask()
                    self.get_logger().warn(f'Goal timeout without being reached {tag_id}')
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete=False) # What is this?
                
            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Failed to reach updated goal near {tag_id}.")
                goal_handle.abort()
                return NavigateAprilTags.Result(navigation_complete=False)
            '''
            elif tag_frame in self.current_detections:
                self.navigator.cancelTask()
                self.get_logger().info(f'Canceling current goal because goal Tag detected {tag_id}')
            #rclpy.spin_once(self) # Is this necessary? 

            if tag_frame not in self.current_detections:
                self.get_logger().warn(f"Tag {tag_frame} not currently detected")
                # Handle this case: maybe wait, retry, or abort

            self.get_logger().info(f'Tag {tag_id} detected, sending navigation goal...')
            # 6. Get tag pose from detection (in camera frame)
            
            ################################################
            # Current simulated detections do not have pose information, running apriltag detection for
            detection = self.current_detections[tag_frame]

            tag_pose_cam = PoseStamped()
            tag_pose_cam.header = detection.pose.header
            tag_pose_cam.pose = detection.pose.pose.pose

            # 7. Transform detection pose to map frame
            try:
                tag_pose_in_map = self.tf_buffer.transform(tag_pose_cam, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            except TransformException as ex:
                self.get_logger().error(f"Failed to transform detected tag pose to map: {ex}")
                return

            # 8. Offset 0.5m in front of detection pose
            tx = tag_pose_in_map.pose.position.x
            ty = tag_pose_in_map.pose.position.y
            q = tag_pose_in_map.pose.orientation
            yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
            goal_x = tx + self.goal_offset * np.cos(yaw)
            goal_y = ty + self.goal_offset * np.sin(yaw)

            # 9. New goal
            new_goal = PoseStamped()
            new_goal.header.frame_id = 'map'
            new_goal.header.stamp = self.get_clock().now().to_msg()
            new_goal.pose.position.x = goal_x
            new_goal.pose.position.y = goal_y
            new_goal.pose.orientation = q

            self.get_logger().info(f"Sending updated goal based on detected tag {tag_frame}.")
            self.navigator.goToPose(new_goal)

            while not self.navigator.isTaskComplete():
                #rclpy.spin_once(self) #what does this do?
                feedback = self.navigator.getFeedback()
                # If task taking too long or april tag detected
                if feedback.navigation_duration > 600:
                    self.navigator.cancelTask()
                    self.get_logger().warn(f'Goal timeout without being reached {tag_id}')
                    goal_handle.abort()
                    return NavigateAprilTags.Result() # What is this?

            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Failed to reach updated goal near {tag_id}.")
                goal_handle.abort()
                return NavigateAprilTags.Result()
            '''
            # Execute command depending on Tag Pattern
            self.get_logger().info(f'Reached tag {tag_id}, executing command...')

            next_tag_frame = f'{self.tag_family}:{next_tag_id}'
            transform = None
            while rclpy.ok():
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        next_tag_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    break
                except TransformException as ex:
                    self.get_logger().warn(f'Waiting for transform {tag_frame}: {ex}')
                    time.sleep(0.5)#await asyncio.sleep(0.5)#await rclpy.sleep(0.5)
            self.get_logger().info(f'Got transform to next tag id: {next_tag_id}')

            command = command_list[i]
            if command == 'vertical': 
                self.get_logger().info(f'Vertical Command')
                tz = transform.transform.translation.z
                depth_goal = GoToDepth.Goal()
                depth_goal.target_depth = tz

                self.get_logger().info(f'Waiting for depth control server to start')
                # Wait for server to be ready
                if not self.depth_control_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("GoToDepth action server not available after 5 seconds!")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete=False)
                # Send the goal
                self.get_logger().info(f"Sending GoToDepth goal to {tz:.2f} meters")
                
                send_goal_future = self.depth_control_client.send_goal_async(depth_goal)
                depth_goal_handle = await send_goal_future

                if not depth_goal_handle.accepted:
                    self.get_logger().warn("Depth goal rejected")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)
            
                result_response = await depth_goal_handle.get_result_async()
                
                if not result_response.result.success:
                    self.get_logger().warn("Depth goal failed")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)
                
                self.get_logger().info("Depth goal succeeded")
            
            elif command == 'horizontal':
                self.get_logger().info(f'Horizontal Command')
                ty = transform.transform.translation.y
                y_goal = GoToSide.Goal()
                y_goal.target_y = ty

                self.get_logger().info(f'Waiting for depth control server to start')
                # Wait for server to be ready
                if not self.strafe_control_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("GoToSide action server not available after 5 seconds!")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete=False)
                # Send the goal
                self.get_logger().info(f"Sending GoToSide goal to {tz:.2f} meters")
                
                send_goal_future = self.strafe_control_client.send_goal_async(y_goal)
                strafe_goal_handle = await send_goal_future

                if not strafe_goal_handle.accepted:
                    self.get_logger().warn("Depth goal rejected")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)
            
                result_response = await strafe_goal_handle.get_result_async()
                
                if not result_response.result.target_reached:
                    self.get_logger().warn("Depth goal failed")
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)
                
                self.get_logger().info("Depth goal succeeded")
                            
        goal_handle.succeed()
        return NavigateAprilTags.Result(navigation_complete = True)


def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigation()
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
