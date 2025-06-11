import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

import sys

class SimpleNav2Client(Node):
    def __init__(self):
        super().__init__('simple_nav2_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Create quaternion from Euler angles (roll, pitch, yaw)
        r = R.from_euler('xyz', [0, 0, theta])
        q = r.as_quat()  # Returns in [x, y, z, w] order
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current position: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNav2Client()

    # Example usage: send goal to (2.0, 3.0, 90 degrees)
    import math
    node.send_goal(2.0, 0.0, math.radians(0))

    rclpy.spin(node)

if __name__ == '__main__':
    main()
