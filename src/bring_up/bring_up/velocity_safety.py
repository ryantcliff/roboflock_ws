import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from bring_up.safety_logic import SafetyController


class VelocitySafety(Node):
    def __init__(self) -> None:
        super().__init__('velocity_safety')

        command_timeout = self.declare_parameter(
            'command_timeout', 0.5
        ).value
        range_timeout = self.declare_parameter('range_timeout', 0.5).value
        stop_distance = self.declare_parameter('stop_distance', 0.30).value
        require_ultrasonic = self.declare_parameter(
            'require_ultrasonic', True
        ).value
        publish_rate = self.declare_parameter('publish_rate', 20.0).value

        self.controller = SafetyController(
            command_timeout=float(command_timeout),
            range_timeout=float(range_timeout),
            stop_distance=float(stop_distance),
            require_ultrasonic=bool(require_ultrasonic),
        )

        self.create_subscription(
            Twist,
            '/cmd_vel/teleop',
            lambda msg: self._command_callback('teleop', msg),
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel/nav',
            lambda msg: self._command_callback('nav', msg),
            10,
        )
        for sensor in ('left', 'center', 'right'):
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor}',
                lambda msg, name=sensor: self._range_callback(name, msg),
                10,
            )
        self.create_subscription(
            Bool,
            '/safety/emergency_stop',
            self._emergency_stop_callback,
            10,
        )

        self.velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel/safe', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/safety/state', 10
        )
        self.create_service(Trigger, '/safety/reset', self._reset_callback)
        self.create_timer(1.0 / float(publish_rate), self._publish)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _command_callback(self, source: str, message: Twist) -> None:
        self.controller.set_command(
            source,
            message.linear.x,
            message.angular.z,
            self._now(),
        )

    def _range_callback(self, sensor: str, message: Range) -> None:
        self.controller.set_range(sensor, message.range, self._now())

    def _emergency_stop_callback(self, message: Bool) -> None:
        self.controller.set_emergency_stop(message.data)

    def _reset_callback(self, request, response):
        del request
        response.success = self.controller.reset()
        response.message = (
            'Safety stop reset'
            if response.success
            else 'Emergency-stop input is still active'
        )
        return response

    def _publish(self) -> None:
        linear_x, angular_z, state = self.controller.evaluate(self._now())
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.velocity_publisher.publish(command)
        self.state_publisher.publish(String(data=state))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocitySafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
