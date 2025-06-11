import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from robot_guidance_interfaces.action import GoToDepth

import asyncio

class DepthControlClient2(Node):
    def __init__(self):
        super().__init__('depth_control_client2')
        self.action_client = ActionClient(
            self, 
            GoToDepth,
            'go_to_depth',
        )

    # called when robot wants to move to new depth, returns None
    async def send_goal(self, target_depth):
        self.action_client.wait_for_server()
        goal = GoToDepth.Goal()
        goal.target_depth = target_depth
        
        self.get_logger().info('Sending goal: Target Depth = ' + str(target_depth))
        sending = self.action_client
        future_server_response = sending.send_goal_async(goal, feedback_callback=self.goal_feedback_callback)
        self.goal_handle = await future_server_response

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal Rejected')
            return
        future_server_result = self.goal_handle.get_result_async()

        service_result = await future_server_result
        status = service_result.status   # ClientGoalHandle goal status
        result = service_result.result   # action specific result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")

        self.get_logger().info("Result: " + str(result.reached_final_depth))
        
        
    # called when feedback is published by the server, returns None
    def goal_feedback_callback(self, feedback_msg):  
        current_depth = feedback_msg.feedback.current_depth
        self.get_logger().info("Current depth: " + str(current_depth))


    # called when robot wants to cancel goal request, returns None
    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle.cancel_goal_async()  # sends cancel request to server


def main(args=None):
    rclpy.init(args=args)
    client = DepthControlClient2()

    # gets pythons default event loop
    # allows us to run async def funcs using await
    loop = asyncio.get_event_loop()

    # creates an executor, adds the node, and spins it in a background thread
    # this makes it so that await still works in the main thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    loop.run_in_executor(None, executor.spin)
    
    try:
        # this calls the async func, send_goal, until it is completed
        loop.run_until_complete(client.send_goal(1.5))
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()