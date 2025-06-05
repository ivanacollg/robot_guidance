#! /usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformException, Buffer, TransformListener
from builtin_interfaces.msg import Duration
from nav2_msgs.action import DriveOnHeading

from rclpy.action import ActionClient
from robot_guidance_interfaces.action import GoToDepth

import numpy as np
from scipy.spatial.transform import Rotation

from robot_guidance_interfaces.action import NavigateAprilTags  # Assume custom action interface defined

class AprilTagNavigator(Node):

    def __init__(self):
        super().__init__('apriltag_navigator_server')

        self.navigator = BasicNavigator()
        self._action_server = ActionServer(
            self,
            NavigateAprilTags,
            'navigate_apriltags',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_detections = {}
        self.latest_odom = None
        self.tag_family = 'tag36h11'
        self.navigator.waitUntilNav2Active()

        self.depth_control_client = ActionClient(self, GoToDepth, 'go_to_depth')
        self.drive_action_client = rclpy.action.ActionClient(self, DriveOnHeading, 'drive_on_heading')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def tag_callback(self, msg: AprilTagDetectionArray):
        self.tag_family = detection.family
        self.current_detections = {f'{detection.family}:{detection.id}': detection for detection in msg.detections}

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        id_list = goal.ids
        command_list = goal.commands

        for i, tag_id in enumerate(id_list):
            next_tag_id = id_list[i+1]
            tag_frame = f'{self.tag_family}:{tag.id}'
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
                    continue

                self.get_logger().info(f'Tag {tag_id} detected, sending navigation goal...')
                
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
                offset = 0.5
                goal_x = tx + offset * np.cos(yaw)
                goal_y = ty + offset * np.sin(yaw)
                # 3. Send initial goToPose
                # Build navigation goal from transform
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = goal_x
                pose.pose.position.y = goal_y
                pose.pose.position.z = transform.transform.translation.z # Will get ignored I think
                pose.pose.orientation = transform.transform.rotation
                self.navigator.goToPose(pose)

                # 4. Spin while tag is not yet detected and goal not reached
                while rclpy.ok() and not self.navigator.isTaskComplete() and tag_frame not in self.current_detections:
                    rclpy.spin_once(self)
                # If goal reached
                if self.navigator.isTaskComplete():
                    result = self.navigator.getResult()
                    if result != TaskResult.SUCCEEDED:
                        self.get_logger().warn(f'Failed to reach tag {tag_id}')
                        continue
                else:
                    # 5. Cancel initial navigation because tag was detected
                    self.get_logger().info(f"Detected {tag_frame} — cancelling initial navigation goal.")
                    self.navigator.cancelTask()

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
            goal_x = tx + offset * np.cos(yaw)
            goal_y = ty + offset * np.sin(yaw)

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
                rclpy.spin_once(self)

            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Failed to reach updated goal near {tag_id}.")
            else:
                self.get_logger().info(f"Successfully navigated to updated goal near {tag_id}.")

            # Execute command depending on Tag Pattern
            self.get_logger().info(f'Reached tag {tag_id}, executing command...')


            if command_list[i] == 'vertical':
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
                        await rclpy.sleep(0.5)
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
                    self.get_logger().warn("Depth goal rejected by server")
                    continue

                self.get_logger().info("Depth goal accepted, waiting for result...")
                get_result_future = depth_goal_handle.get_result_async()
                result_response = await get_result_future

                if result_response.result.success:
                    self.get_logger().info("Successfully reached target depth.")
                else:
                    self.get_logger().warn("Failed to reach target depth.")

            elif command_list[i] == 'horizontal':
                self.get_logger().info('Starting DriveOnHeading plugin...')
                if not self.drive_action_client.wait_for_server(timeout_sec=10.0):
                    self.get_logger().error('DriveOnHeading action server not available.')
                    depth_goal_handle.abort()
                    return NavigateAprilTags.Result()

                drive_goal = DriveOnHeading.Goal()
                #drive_goal.target.yaw = 0.0  # forward
                drive_goal.target.yaw = math.pi/2  # forward
                drive_goal.target.time = Duration(sec=10)
                drive_goal.target.speed = 0.2
                send_goal_future = self.drive_action_client.send_goal_async(drive_goal)
                await send_goal_future
                drive_goal_handle = send_goal_future.result()

                if not drive_goal_handle.accepted:
                    self.get_logger().warn('DriveOnHeading goal rejected.')
                    continue

                feedback_future = drive_goal_handle.get_result_async()

                result_response = await feedback_future  # wait for completion or cancellation
                if result_response.result.success:
                    self.get_logger().info("Successfully finished strafing.")
                else:
                    self.get_logger().warn("Failed to finish strafing task.")
            
        goal_handle.succeed()
        result = NavigateAprilTags.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
