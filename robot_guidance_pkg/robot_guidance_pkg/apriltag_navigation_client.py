import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_guidance_interfaces.action import NavigateAprilTags

class ApriltagNavigationClient(Node):
    def __init__(self):
        super().__init__('apriltag_navigation_client')
        self.apriltag_navigation_action_client = ActionClient(
            self, 
            NavigateAprilTags,
            'navigate_apriltags',
        )


    def send_goal(self, ids, commands):
        self.apriltag_navigation_action_client.wait_for_server()
        goal = NavigateAprilTags.Goal()
        goal.ids = ids
        goal.commands = commands
        
        self.get_logger().info('Sending goal: Ids = ' + str(ids) + ' commands = ' + str(commands))
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



def main(args=None):
    rclpy.init(args=args)
    client = ApriltagNavigationClient()
    ids = [1,2,1,3,4,3,5,6,5]
    commands = ['vertical', 'vertical', 'horizontal', 'vertical', 'vertical', 'horizontal', 'vertical', 'vertical', 'finish']
    client.send_goal(ids, commands)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()