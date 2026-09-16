import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from bring_up.gps_math import offset_to_fix


class SimulatedGps(Node):
    def __init__(self) -> None:
        super().__init__('simulated_gps')
        self.origin_latitude = float(
            self.declare_parameter('origin_latitude', 40.7864).value
        )
        self.origin_longitude = float(
            self.declare_parameter('origin_longitude', -119.2065).value
        )
        self.beacon_east = float(
            self.declare_parameter('beacon_east', 4.0).value
        )
        self.beacon_north = float(
            self.declare_parameter('beacon_north', 0.0).value
        )
        self.beacon_radius = float(
            self.declare_parameter('beacon_radius', 0.0).value
        )
        self.beacon_angular_speed = float(
            self.declare_parameter('beacon_angular_speed', 0.0).value
        )
        publish_rate = float(self.declare_parameter('publish_rate', 5.0).value)

        self.robot_east = 0.0
        self.robot_north = 0.0
        self.have_odometry = False
        self.start_time = self.get_clock().now()
        self.robot_publisher = self.create_publisher(
            NavSatFix, '/gps/robot/fix', 10
        )
        self.beacon_publisher = self.create_publisher(
            NavSatFix, '/gps/beacon/fix', 10
        )
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_timer(1.0 / publish_rate, self._publish)

    def _odom_callback(self, message: Odometry) -> None:
        self.robot_east = message.pose.pose.position.x
        self.robot_north = message.pose.pose.position.y
        self.have_odometry = True

    def _fix(self, east: float, north: float, frame_id: str) -> NavSatFix:
        latitude, longitude = offset_to_fix(
            self.origin_latitude,
            self.origin_longitude,
            east,
            north,
        )
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = 0.0
        message.position_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 1.0,
        ]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return message

    def _publish(self) -> None:
        if not self.have_odometry:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        angle = self.beacon_angular_speed * elapsed
        beacon_east = self.beacon_east + self.beacon_radius * math.cos(angle)
        beacon_north = self.beacon_north + self.beacon_radius * math.sin(angle)
        self.robot_publisher.publish(
            self._fix(self.robot_east, self.robot_north, 'gps_robot')
        )
        self.beacon_publisher.publish(
            self._fix(beacon_east, beacon_north, 'gps_beacon')
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulatedGps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
