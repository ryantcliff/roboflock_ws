"""
Hardware-free stand-in for the robot's drivers and motors.

Integrates velocity commands into a unicycle pose in true east/north metres
around a GPS datum, and publishes what the real sensors would: laser odometry
(/odom_rf2o), IMU (/imu/data, magnetic yaw), robot GPS (/gps/robot/fix) and an
empty lidar scan. Ground truth goes to /sim/pose (frame 'enu') for tests.
"""
import math
import random

from bring_up.sim_geo import LocalTangent, utm_convergence
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus

RATE = 20.0


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        lat = self.declare_parameter('datum_lat', 43.0008).value
        lon = self.declare_parameter('datum_lon', -78.7890).value
        # Must match navsat_transform_node in ekf_navsat_params.yaml.
        declination = self.declare_parameter('magnetic_declination', 0.17872172).value
        self.gps_noise = self.declare_parameter('gps_noise', 0.0).value
        self.max_speed = self.declare_parameter('max_speed', 1.5).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        cmd_topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel').value

        self.tangent = LocalTangent(lat, lon)
        # A heading of true east reads magnetic yaw -(declination - convergence).
        self.imu_offset = declination - utm_convergence(lat, lon)
        self.x = self.y = 0.0
        self.yaw = self.declare_parameter('initial_yaw', 0.0).value  # true ENU yaw
        self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw
        self.v = self.w = 0.0
        self.last_cmd = None

        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom_rf2o', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/robot/fix', 10)
        self.truth_pub = self.create_publisher(PoseStamped, 'sim/pose', 10)
        self.create_timer(1.0 / RATE, self._step)
        self.create_timer(0.2, self._publish_gps)
        self.get_logger().info(f'Fake robot at {lat:.5f}, {lon:.5f} listening on {cmd_topic}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg):
        self.v = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.w = msg.angular.z
        self.last_cmd = self._now()

    def _step(self):
        if self.last_cmd is None or self._now() - self.last_cmd > self.cmd_timeout:
            self.v = self.w = 0.0
        dt = 1.0 / RATE
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt))
        stamp = self.get_clock().now().to_msg()

        # Laser odometry: pose relative to the start pose, like rf2o.
        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = c * dx - s * dy
        odom.pose.pose.position.y = s * dx + c * dy
        qz, qw = yaw_quaternion(self.yaw - self.yaw0)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        for i in range(6):
            odom.pose.covariance[i * 7] = 0.01
            odom.twist.covariance[i * 7] = 0.01
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        qz, qw = yaw_quaternion(self.yaw - self.imu_offset)
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.z = self.w
        imu.linear_acceleration.z = 9.80665
        for i in range(3):
            imu.orientation_covariance[i * 4] = 0.01
            imu.angular_velocity_covariance[i * 4] = 0.01
            imu.linear_acceleration_covariance[i * 4] = 0.01
        self.imu_pub.publish(imu)

        scan = LaserScan()
        # rplidar_ros stamps a scan at its start, one scan period back. A scan
        # stamped ahead of TF makes the costmap wait on it asynchronously, which
        # can deadlock tf2_ros 0.25.23 (MessageFilter vs testTransformableRequests).
        scan.header.stamp = (self.get_clock().now()
                             - rclpy.duration.Duration(seconds=0.1)).to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min = 0.15
        scan.range_max = 12.0
        scan.scan_time = 1.0 / RATE
        scan.ranges = [float('inf')] * 360  # open field: nothing in range
        self.scan_pub.publish(scan)

        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = 'enu'
        truth.pose.position.x, truth.pose.position.y = self.x, self.y
        qz, qw = yaw_quaternion(self.yaw)
        truth.pose.orientation.z, truth.pose.orientation.w = qz, qw
        self.truth_pub.publish(truth)

    def _publish_gps(self):
        east = self.x + random.gauss(0.0, self.gps_noise)
        north = self.y + random.gauss(0.0, self.gps_noise)
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = self.tangent.to_latlon(east, north)
        fix.altitude = 180.0
        variance = max(self.gps_noise ** 2, 0.25)
        fix.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
