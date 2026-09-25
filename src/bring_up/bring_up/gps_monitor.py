"""Warn when a GPS fix stream goes stale or loses its fix."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


class FixHealth:
    """Tracks one NavSatFix stream; state is 'ok', 'no_fix' or 'stale'."""

    def __init__(self, max_age):
        self.max_age = max_age
        self._last_time = None
        self._has_fix = False

    def on_fix(self, status, now):
        self._last_time = now
        self._has_fix = status >= NavSatStatus.STATUS_FIX

    def state(self, now):
        if self._last_time is None or now - self._last_time > self.max_age:
            return 'stale'
        return 'ok' if self._has_fix else 'no_fix'


class GpsMonitor(Node):

    def __init__(self):
        super().__init__('gps_monitor')
        topics = self.declare_parameter('topics', ['/gps/robot/fix']).value
        max_age = self.declare_parameter('max_age', 2.0).value
        self.health = {}
        self.reported = {}
        for topic in topics:
            self.health[topic] = FixHealth(max_age)
            self.reported[topic] = None
            self.create_subscription(
                NavSatFix, topic,
                lambda msg, topic=topic: self.health[topic].on_fix(msg.status.status, self._now()),
                qos_profile_sensor_data)
        self.create_timer(0.5, self._check)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _check(self):
        for topic, health in self.health.items():
            state = health.state(self._now())
            if state != self.reported[topic]:
                # rclpy forbids changing severity at one call site, so use two.
                if state == 'ok':
                    self.get_logger().info(f'{topic}: {state}')
                else:
                    self.get_logger().warn(f'{topic}: {state}')
                self.reported[topic] = state


def main(args=None):
    rclpy.init(args=args)
    node = GpsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
