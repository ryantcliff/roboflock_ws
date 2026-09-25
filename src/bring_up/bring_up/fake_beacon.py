"""
Hardware-free stand-in for the beacon carrier.

Waits start_delay seconds, walks the waypoint path (east/north metres from the
datum) at speed, then stands still. After dropout_after seconds standing still
it stops publishing, as if the radio link dropped (negative: never).
Publishes /gps/beacon/fix and ground truth /sim/beacon_pose (frame 'enu').
"""
import math

from bring_up.sim_geo import LocalTangent
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class FakeBeacon(Node):

    def __init__(self):
        super().__init__('fake_beacon')
        lat = self.declare_parameter('datum_lat', 43.0008).value
        lon = self.declare_parameter('datum_lon', -78.7890).value
        flat = self.declare_parameter('waypoints', [6.0, 0.0, 21.0, 0.0, 21.0, 10.0]).value
        self.speed = self.declare_parameter('speed', 0.4).value
        self.start_delay = self.declare_parameter('start_delay', 20.0).value
        self.dropout_after = self.declare_parameter('dropout_after', -1.0).value
        self.tangent = LocalTangent(lat, lon)
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.x, self.y = self.waypoints[0]
        self.next = 1
        self.start = self._now()
        self.arrived_at = None
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/beacon/fix', 10)
        self.truth_pub = self.create_publisher(PoseStamped, 'sim/beacon_pose', 10)
        self.create_timer(0.2, self._step)
        self.get_logger().info(
            f'Fake beacon: {len(self.waypoints)} waypoints at {self.speed} m/s, '
            f'walking in {self.start_delay:.0f} s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _step(self):
        now = self._now()
        if now - self.start >= self.start_delay and self.next < len(self.waypoints):
            tx, ty = self.waypoints[self.next]
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            step = self.speed * 0.2
            if dist <= step:
                self.x, self.y = tx, ty
                self.next += 1
                if self.next == len(self.waypoints):
                    self.arrived_at = now
                    self.get_logger().info('Beacon reached the last waypoint')
            else:
                self.x += dx / dist * step
                self.y += dy / dist * step
        if (self.arrived_at is not None and self.dropout_after >= 0.0 and
                now - self.arrived_at > self.dropout_after):
            return  # simulated link loss: no more fixes
        stamp = self.get_clock().now().to_msg()
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = self.tangent.to_latlon(self.x, self.y)
        fix.altitude = 180.0
        fix.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)
        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = 'enu'
        truth.pose.position.x, truth.pose.position.y = self.x, self.y
        truth.pose.orientation.w = 1.0
        self.truth_pub.publish(truth)


def main(args=None):
    rclpy.init(args=args)
    node = FakeBeacon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
