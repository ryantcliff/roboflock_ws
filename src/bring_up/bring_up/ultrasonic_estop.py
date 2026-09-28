#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class UltrasonicLogic:
    """
    Decide when the ultrasonic sensors call for a stop.

    A sensor counts as blocked after required_readings consecutive readings
    closer than min_safe_distance, so a single glitched echo doesn't stop the
    robot. One reading at or beyond the limit (including +inf, no echo) clears
    it. Sensors with no reading in max_reading_age are ignored.
    """

    def __init__(self, frame_ids, min_safe_distance, max_reading_age, required_readings):
        self.min_safe_distance = min_safe_distance
        self.max_reading_age = max_reading_age
        self.required_readings = required_readings
        self._close_count = {frame_id: 0 for frame_id in frame_ids}
        self._last_reading = {}

    def on_range(self, frame_id, distance, now):
        if distance < self.min_safe_distance:
            self._close_count[frame_id] = self._close_count.get(frame_id, 0) + 1
        else:
            self._close_count[frame_id] = 0
        self._last_reading[frame_id] = now

    def unsafe(self, now):
        for frame_id, count in self._close_count.items():
            stamp = self._last_reading.get(frame_id)
            if stamp is None or now - stamp > self.max_reading_age:
                continue  # no recent reading from this sensor
            if count >= self.required_readings:
                return True
        return False


class UltrasonicEstop(Node):
    """
    Provide an independent safety failsafe.

    LIDAR-based Nav2 costmap obstacle avoidance is the primary obstacle
    detection system. This node is a last-resort backstop for the LIDAR's
    blind spots (e.g. low obstacles below the scan plane): it zeroes the
    robot's velocity via twist_mux's highest-priority input whenever any
    ultrasonic sensor reads closer than min_safe_distance, and simply stops
    publishing once every sensor is clear again so lower-priority commands
    (Nav2, joystick) resume through twist_mux.
    """

    def __init__(self):
        super().__init__('ultrasonic_estop')

        self.declare_parameter('frame_ids', [
            'left_ultrasonic', 'center_ultrasonic', 'right_ultrasonic'])
        self.declare_parameter('min_safe_distance', 0.15)  # meters
        self.declare_parameter('max_reading_age', 1.0)     # seconds
        self.declare_parameter('publish_rate', 10.0)       # Hz
        # Consecutive close readings before stopping (20 Hz sensors: 2 = ~50 ms)
        self.declare_parameter('required_readings', 2)

        self.frame_ids = self.get_parameter('frame_ids').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        publish_rate = self.get_parameter('publish_rate').value
        self._logic = UltrasonicLogic(
            self.frame_ids, self.min_safe_distance,
            self.get_parameter('max_reading_age').value,
            self.get_parameter('required_readings').value)

        for frame_id in self.frame_ids:
            topic = f'range/{frame_id}'
            self.create_subscription(
                Range, topic,
                lambda msg, fid=frame_id: self._range_callback(msg, fid),
                10)

        self._estop_pub = self.create_publisher(Twist, 'cmd_vel_estop', 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._check_and_publish)

        self.get_logger().info(
            f'Ultrasonic e-stop watching {self.frame_ids} '
            f'at {self.min_safe_distance:.2f} m, '
            f'{self._logic.required_readings} readings in a row')

    def _range_callback(self, msg: Range, frame_id: str):
        self._logic.on_range(frame_id, msg.range, time.monotonic())

    def _check_and_publish(self):
        if self._logic.unsafe(time.monotonic()):
            self._estop_pub.publish(Twist())  # all-zero


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
