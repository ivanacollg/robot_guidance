import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_guidance_interfaces.action import NavigateAprilTags
from scipy.spatial.transform import Rotation as R
import os
import yaml

class ApriltagNavigationClient(Node):
    def __init__(self):
        super().__init__('apriltag_navigation_client')

        self.declare_parameter('tag_map_path', '')
        self.declare_parameter('cleaning_routine_topic', 'cleaning_routine/poses')

        cleaning_routine_topic = self.get_parameter('cleaning_routine_topic').get_parameter_value().string_value
        self.pose_pub = self.create_publisher(PoseArray, cleaning_routine_topic, 10)

        self.apriltag_navigation_action_client = ActionClient(
            self, 
            NavigateAprilTags,
            'navigate_apriltags',
        )


    def send_goal(self, pose_list, commands):
        self.apriltag_navigation_action_client.wait_for_server()
        goal = NavigateAprilTags.Goal()
        goal.goals = pose_list
        goal.commands = commands
        
        self.get_logger().info('Sending goal:') #Ids = ' + str(ids) + ' commands = ' + str(commands))
        self.apriltag_navigation_action_client. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)

        # timer to cancel goal to test functionality
        # self.timer_ = self.create_timer(10.0, self.cancel_goal)

    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async(). \
                add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.navigation_completed))

    
    def goal_feedback_callback(self, feedback_msg):
        current_id = feedback_msg.feedback.current_id
        current_command = feedback_msg.feedback.current_command
        self.get_logger().info("Current ID: " + str(current_id) + " Current command: " + current_command)


    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle_.cancel_goal_async()
        
        # timer to test functionality
        # self.timer_.cancel()

    def load_tag_map_and_convert_to_poses(self):
        tag_map_path = self.get_parameter('tag_map_path').get_parameter_value().string_value

        if not os.path.isfile(tag_map_path):
            self.get_logger().error(f"Tag map file not found: {tag_map_path}")
            return []

        with open(tag_map_path, 'r') as f:
            tag_data = yaml.safe_load(f)

        pose_list = []
        command_list = []
        for tag in tag_data['poses']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = tag['x']
            pose.pose.position.y = tag['y']
            pose.pose.position.z = tag['z']

            # Convert RPY to quaternion
            r = R.from_euler('xyz', [tag['roll'], tag['pitch'], tag['yaw']], degrees=True)
            q = r.as_quat()

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            pose_list.append(pose)

            command = tag['command']
            command_list.append(command)

        self.get_logger().info(f"Loaded {len(pose_list)} poses from tag map.")
        return pose_list, command_list


    def publish_poses(self, pose_list):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        pose_only_list = [p.pose for p in pose_list]
        msg.poses = pose_only_list
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Published PoseArray with {len(pose_list)} poses')

def main(args=None):
    rclpy.init(args=args)
    client = ApriltagNavigationClient()
    pose_list, command_list = client.load_tag_map_and_convert_to_poses()
    client.publish_poses(pose_list)
    client.send_goal(pose_list, command_list)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()