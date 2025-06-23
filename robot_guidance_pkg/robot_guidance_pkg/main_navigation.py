
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from robot_guidance_pkg.apriltag_navigation_server import AprilTagNavigation

def main(args=None):
    rclpy.init(args=args)
    
    # Basic Navigator for moving to a pose
    navigator = BasicNavigator()
    # waiting for Nav2Active
    navigator.waitUntilNav2Active()  # The BasicNavigator.waitUntilNav2Active() function explicitly checks for /amcl/get_state. If you're not using AMCL and that service doesn't exist, this call will block forever.
    
    nav_node = AprilTagNavigation(navigator)

    executor = SingleThreadedExecutor()
    executor.add_node(nav_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
