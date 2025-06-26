import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from robot_guidance_interfaces.action import GoToDepth


class DepthControlClient(Node):
    """
    Client node for GoToDepth action.

    Args:
        Node:
            Make this class a ROS2 Node
    """
    def __init__(self):
        """
        Initializes the client.

        Args:
            self:
                The client node
        
        Returns:
            None: None
        """
        super().__init__('depth_control_client')
        self.action_client = ActionClient(
            self, 
            GoToDepth,
            'go_to_depth',
        )


    # called when robot wants to move to new depth, returns None
    def send_goal(self, target_depth: float):
        """
        Called when the client sends a depth goal to the server.

        Args:
            self:
                The client node
            target_depth:
                The target depth(z position) of the next goal
                
        Returns:
            None: None
        """
        # wait for server to be running
        self.action_client.wait_for_server()

        # create and set goal
        goal = GoToDepth.Goal()

        # check that goal is valid here before using the parameter target_depth anywhere
        if not type(target_depth) == float:
             if type(target_depth) == int:
                 target_depth = float(target_depth)
             else:
                self.get_logger().warn('Invalid Goal Request. Input must be an integer or float')
                self.create_timer(0.1, self.shutdown_node)
                return

        goal.target_depth = target_depth
        
        # send goal and set the feedback callback func
        self.get_logger().info('Sending goal: Target Depth = ' + str(target_depth))
        sending = self.action_client
        future_server_response = sending.send_goal_async(goal, feedback_callback=self.goal_feedback_callback) 

        # call goal_response_callback when server responds to client
        future_server_response.add_done_callback(self.goal_response_callback) 


    # called when server responds about accept/reject of goal
    def goal_response_callback(self, server_response: Future):
        """
        Gets a future server_response (goal accepted or rejected) and waits for the server result.

        Args:
            self:
                The client node
            server_response:
                A future goal reponse from the server
        
        Returns:
            None: None
        """
        self.goal_handle = server_response.result() # get goal handle
        if not self.goal_handle.accepted: # check if goal not accepted
            self.get_logger().warn('Goal Rejected')
            self.create_timer(0.1, self.shutdown_node)
            return
        
        future_server_result = self.goal_handle.get_result_async() # wait for result of goal
        future_server_result.add_done_callback(self.goal_result_callback) # call goal_result_callback when goal completed


    # called when goal completed and result received by client 
    def goal_result_callback(self, server_result: Future):
        """
        Gets a interprets a future goal result from the server.

        Args:
            self:
                The client node
            server_result:
                A future goal result from the server

        Returns:
            None: None
        """
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

        self.create_timer(0.1, self.shutdown_node)
        return

    # called when feedback is published by the server
    def goal_feedback_callback(self, feedback_msg: GoToDepth.Feedback):
        """
        Gets feedback from the server containing the current depth of the robot.

        Args:
            self:
                The client node
            feedback_msg:
                A feedback message from the server
        
        Returns:
            None: None
        """
        current_depth = feedback_msg.feedback.current_depth
        self.get_logger().info("Current depth: " + str(current_depth))


    # called when robot wants to cancel goal request
    def cancel_goal(self):
        """
        Sends a goal cancel request to the server. 

        Args:
            self:
                The client node

        Returns:
            None: None
        """
        self.get_logger().info("Send a cancel request")
        self.goal_handle.cancel_goal_async()  # sends cancel request to server


    def shutdown_node(self):
        """
        Shutdown the client node
        
        Args:
            self:
                The client node
        
        Return:
            None: None
        """
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = DepthControlClient()
    client.send_goal(1.5)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Make sure we don’t double shutdown
            rclpy.shutdown()
        client.destroy_node()

if __name__ == '__main__':
    main()