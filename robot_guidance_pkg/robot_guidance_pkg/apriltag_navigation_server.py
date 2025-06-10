#!/usr/bin/env python3

import math
import asyncio
import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import DriveOnHeading
from builtin_interfaces.msg import Duration
from robot_guidance_interfaces.action import GoToDepth, NavigateAprilTags # Custom action

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
        self.drive_action_client = rclpy.action.ActionClient(self, DriveOnHeading, 'drive_on_heading')
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

        for i, tag_id in enumerate(id_list):
            next_tag_id = id_list[i+1]
            tag_frame = f'{self.tag_family}:{tag_id}'
            # If desired April tag is not detected, navigate to it
            if tag_frame not in self.current_detections:
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
                        self.get_logger().warn_throttle(5000, f'Waiting for transform {tag_frame}: {ex}')
                        await rclpy.sleep(0.5)

                if transform is None:
                    self.get_logger().warn(f'Transform to {tag_frame} not found.')
                    goal_handle.abort()
                    return NavigateAprilTags.Result()
                
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
                goal_x = tx + self.goal_offset * np.cos(yaw)
                goal_y = ty + self.goal_offset * np.sin(yaw)

                # 3. Send initial goToPose
                # Build navigation goal from transform
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = goal_x
                pose.pose.position.y = goal_y
                pose.pose.orientation = q
                go_to_pose_task = self.navigator.goToPose(pose)

                # 4. Spin while tag is not yet detected and goal not reached
                while rclpy.ok() and not self.navigator.isTaskComplete(task = go_to_pose_task):
                    feedback = self.navigator.getFeedback(task = go_to_pose_task)
                    # If task taking too long or april tag detected
                    if feedback.navigation_duration > 600:
                        self.navigator.cancelTask()
                        self.get_logger().warn(f'Goal timeout without being reached {tag_id}')
                        goal_handle.abort()
                        return NavigateAprilTags.Result() # What is this?
                    elif tag_frame in self.current_detections:
                        self.navigator.cancelTask()
                        self.get_logger().info(f'Tag detected {tag_id}')
                    #rclpy.spin_once(self) # Is this necessary? 
                '''
                # If goal reached
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    self.get_logger().warn(f'Goal Failed to reach tag {tag_id}')
                '''
            self.get_logger().info(f'Tag {tag_id} detected, sending navigation goal...')
            # 6. Get tag pose from detection (in camera frame)
            detection = self.current_detections[tag_frame]
            tag_pose_cam = PoseStamped()
            tag_pose_cam.header = detection.pose.pose.pose.header
            tag_pose_cam.pose = detection.pose.pose.pose.pose

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

            # Execute command depending on Tag Pattern
            self.get_logger().info(f'Reached tag {tag_id}, executing command...')

            command = command_list[i]
            if command == 'vertical': #and i + 1 < len(id_list):
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
                        self.get_logger().warn_throttle(5000, f'Waiting for transform {tag_frame}: {ex}')
                        await asyncio.sleep(0.5)#await rclpy.sleep(0.5)
                
                tz = transform.transform.translation.z
                depth_goal = GoToDepth.Goal()
                depth_goal.target_depth = tz

                # Wait for server to be ready
                await self.depth_control_client.wait_for_server()
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
                self.get_logger().info('Starting DriveOnHeading plugin...')
                if not self.drive_action_client.wait_for_server(timeout_sec=10.0):
                    self.get_logger().error('DriveOnHeading action server not available.')
                    depth_goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)

                drive_goal = DriveOnHeading.Goal()
                #drive_goal.target.yaw = 0.0  # forward
                drive_goal.target.yaw = math.pi/2  # forward
                drive_goal.target.time = Duration(sec=10)
                drive_goal.target.speed = 0.2
                
                send_goal_future = self.drive_action_client.send_goal_async(drive_goal)
                drive_goal_handle = await send_goal_future

                if not drive_goal_handle.accepted:
                    self.get_logger().warn('DriveOnHeading goal rejected.')
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)

                result_response = await drive_goal_handle.get_result_async()

                if not result_response.result.success:
                    self.get_logger().warn('DriveOnHeading task failed')
                    goal_handle.abort()
                    return NavigateAprilTags.Result(navigation_complete = False)
                self.get_logger().info('DriveOnHeading task succeeded')
            
            goal_handle.succeed()
        return NavigateAprilTags.Result(navigation_complete = True)


def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigation()
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
