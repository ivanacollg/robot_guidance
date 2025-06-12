import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from robot_guidance_interfaces.action import GoToDepth


class DepthControlClient(Node):
    def __init__(self):
        super().__init__('depth_control_client')
        self.action_client = ActionClient(
            self, 
            GoToDepth,
            'go_to_depth',
        )


    # called when robot wants to move to new depth, returns None
    def send_goal(self, target_depth):
        # wait for server to be running
        self.action_client.wait_for_server()

        # create and set goal
        goal = GoToDepth.Goal()

        # check that goal is valid here before using the parameter target_depth anywhere
        if not type(target_depth) == float:
             if type(target_depth) == int:
                 target_depth = float(target_depth)
             else:
                self.get_logger().warn('Invalid Goal Request. Input must be integer or float')
                return

        goal.target_depth = target_depth
        
        # send goal and set the feedback callback func
        self.get_logger().info('Sending goal: Target Depth = ' + str(target_depth))
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
        self.get_logger().info("Result: " + str(result.reached_final_depth)) # print result


    # called when feedback is published by the server
    def goal_feedback_callback(self, feedback_msg):
        current_depth = feedback_msg.feedback.current_depth
        self.get_logger().info("Current depth: " + str(current_depth))


    # called when robot wants to cancel goal request
    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle.cancel_goal_async()  # sends cancel request to server


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