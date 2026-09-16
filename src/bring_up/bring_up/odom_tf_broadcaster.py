import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
        )

        self.get_logger().info(
                'Broadcasting /odom as TF: odom -> base_link'
        )

    def odom_callback(self, msg):
        transform = TransformStamped()

        # Use the timestamp from the simulated odometry
        transform.header.stamp = msg.header.stamp

        # These should already be "odom" and "base_link"
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = msg.child_frame_id

        # Copy position
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Copy orientation
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    node = OdomTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
