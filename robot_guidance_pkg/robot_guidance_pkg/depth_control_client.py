import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from robot_guidance_interfaces.action import GoToDepth

class DepthControlClient(Node):
    def __init__(self):
        super().__init__('depth_control_client')
        self._depth_action_client = ActionClient(
            self, 
            GoToDepth,
            'go_to_depth',
        )


    def send_goal(self, target_depth):
        self._depth_action_client.wait_for_server()
        goal = GoToDepth.Goal()
        goal.target_depth = target_depth
        
        self.get_logger().info('Sending goal: Target Depth = ' + str(target_depth))
        self._depth_action_client. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async(). \
                add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: " + str(result.success))

    
    def goal_feedback_callback(self, feedback_msg):
        current_depth = feedback_msg.feedback.current_depth
        self.get_logger().info("Got feedback: " + str(current_depth))

def main(args=None):
    rclpy.init(args=args)
    client = DepthControlClient()
    client.send_goal(1.5)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()