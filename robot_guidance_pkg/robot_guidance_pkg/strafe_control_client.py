import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from robot_guidance_interfaces.action import GoToSide
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from nav2_simple_commander.robot_navigator import BasicNavigator

import math


class StrafeControlClient(Node):
    def __init__(self):
        super().__init__('stafe_control_client')

        self.action_client = ActionClient(
            self, 
            GoToSide,
            'go_to_side',
        )

    # called when robot wants to move to new depth, returns None
    def send_goal(self, target_pose):
        self.get_logger().info("Waiting for server")
        # wait for server to be running
        self.action_client.wait_for_server()

        # create and set goal
        goal = GoToSide.Goal()
        goal.target_pose = target_pose
        
        # send goal and set the feedback callback func
        self.get_logger().info('Sending goal: Target Depth = ' + str(target_pose))
        sending = self.action_client
        future_server_response = sending.send_goal_async(goal, feedback_callback=self.goal_feedback_callback) 

        # call goal_response_callback when server responds to client
        future_server_response.add_done_callback(self.goal_response_callback) 


    # called when server responds about accept/reject of goal
    def goal_response_callback(self, server_response):
        self.goal_handle = server_response.result() # get goal handle
        if not self.goal_handle.accepted: # check if goal not accepted
            self.get_logger().warn('Goal Rejected')
            return
        
        future_server_result = self.goal_handle.get_result_async() # wait for result of goal
        future_server_result.add_done_callback(self.goal_result_callback) # call goal_result_callback when goal completed


    # called when goal completed and result received by client 
    def goal_result_callback(self, server_result): 
        service_result = server_result.result() # get service result
        status = service_result.status   # ClientGoalHandle goal status
        result = service_result.result   # action specific result

        if status == GoalStatus.STATUS_SUCCEEDED: # check and print goal status
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.target_reached)) # print result


    # called when feedback is published by the server
    def goal_feedback_callback(self, feedback_msg):
        distance_remaining= feedback_msg.feedback.distance_remaining
        self.get_logger().info("Current distance remaining: " + str(distance_remaining))


    # called when robot wants to cancel goal request
    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle.cancel_goal_async()  # sends cancel request to server


def main(args=None):
    rclpy.init(args=args)
    client = StrafeControlClient()

    # Create goal pose
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = client.get_clock().now().to_msg() 

    goal.pose.position.x = 0.0
    goal.pose.position.y = 3.0
    goal.pose.position.z = 0.0

    # Convert yaw to quaternion
    # Convert yaw to quaternion using scipy
    yaw = math.radians(0)
    r = R.from_euler('zyx', [yaw, 0.0, 0.0])  # yaw (Z), pitch (Y), roll (X)
    q = r.as_quat()  # Returns [x, y, z, w]

    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    client.get_logger().info("Sending goal")

    client.send_goal(goal)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()