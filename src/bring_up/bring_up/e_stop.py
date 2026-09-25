"""Joystick e-stop: publishes a latched /e_stop flag for diff_drive_controller.

Starts stopped. Press reset (Options) to arm; press stop (Cross) to stop.
Losing /joy for joy_timeout seconds also stops. Every stop latches until reset.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


E_STOP_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class EStopLogic:
    def __init__(self, stop_button, reset_button, joy_timeout):
        self.stop_button = stop_button
        self.reset_button = reset_button
        self.joy_timeout = joy_timeout
        self.stopped = True
        self._buttons = []
        self._last_joy = None

    def _pressed(self, index):
        return 0 <= index < len(self._buttons) and self._buttons[index] == 1

    def on_joy(self, buttons, now):
        self._buttons = list(buttons)
        self._last_joy = now
        if self._pressed(self.stop_button):
            self.stopped = True
        elif self._pressed(self.reset_button):
            self.stopped = False

    def update(self, now):
        if self._last_joy is None or now - self._last_joy > self.joy_timeout:
            self.stopped = True
        return self.stopped


class EStop(Node):
    def __init__(self):
        super().__init__('e_stop')
        # Defaults match a PS4 controller under joy_node (hid-generic): Cross = 1, Options = 9.
        self.logic = EStopLogic(
            stop_button=self.declare_parameter('stop_button', 1).value,
            reset_button=self.declare_parameter('reset_button', 9).value,
            joy_timeout=self.declare_parameter('joy_timeout', 0.5).value,
        )
        self.publisher = self.create_publisher(Bool, 'e_stop', E_STOP_QOS)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        # Heartbeat lets diff_drive_controller stop if this node dies.
        self.create_timer(0.05, self._tick)
        self._last_state = None
        self._tick()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _joy_cb(self, msg):
        self.logic.on_joy(msg.buttons, self._now())
        self._tick()

    def _tick(self):
        stopped = self.logic.update(self._now())
        if stopped != self._last_state:
            self.get_logger().warn('E-STOP ENGAGED' if stopped else 'E-stop released')
            self._last_state = stopped
        self.publisher.publish(Bool(data=stopped))


def main(args=None):
    rclpy.init(args=args)
    node = EStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Bool(data=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
