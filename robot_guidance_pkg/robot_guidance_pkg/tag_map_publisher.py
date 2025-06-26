import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml
import os
from scipy.spatial.transform import Rotation as R 

class TagMapPublisher(Node):
    def __init__(self):
        super().__init__('tag_map_publisher')
        self.declare_parameter('tag_map_path', '')

        tag_map_path = self.get_parameter('tag_map_path').get_parameter_value().string_value
        if not os.path.isfile(tag_map_path):
            self.get_logger().error(f"Tag map file not found: {tag_map_path}")
            return

        with open(tag_map_path, 'r') as f:
            tag_data = yaml.safe_load(f)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_tag_transforms(tag_data.get('fiducials', []))

    def publish_tag_transforms(self, tag_list):
        transforms = []
        for tag in tag_list:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'optitrack'
            tf_msg.child_frame_id = f"tag36h11:{tag['id']}"

            tf_msg.transform.translation.x = float(tag['x'])
            tf_msg.transform.translation.y = float(tag['y'])
            tf_msg.transform.translation.z = float(tag['z'])

            # Use scipy to compute quaternion
            r = R.from_euler('xyz', [
                float(tag.get('roll', 0.0)),
                float(tag.get('pitch', 0.0)),
                float(tag.get('yaw', 0.0))
            ])
            q = r.as_quat()  # [x, y, z, w]

            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]

            transforms.append(tf_msg)

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {len(transforms)} tag transforms.")


def main(args=None):
    rclpy.init(args=args)
    node = TagMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
