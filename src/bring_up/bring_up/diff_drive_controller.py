import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

from bring_up.drive_kinematics import wheel_turn_rates


class DiffDriveController(Node):
    def __init__(self) -> None:
        super().__init__('diff_drive_controller')

        self.wheel_radius = float(
            self.declare_parameter('wheel_radius', 0.10).value
        )
        self.wheel_separation = float(
            self.declare_parameter('wheel_separation', 0.64).value
        )
        self.gear_ratio = float(
            self.declare_parameter('gear_ratio', 30.0).value
        )
        self.max_motor_turns_per_second = float(
            self.declare_parameter(
                'max_motor_turns_per_second', 60.0
            ).value
        )
        self.command_timeout = float(
            self.declare_parameter('command_timeout', 0.5).value
        )
        self.velocity_ramp_rate = float(
            self.declare_parameter('velocity_ramp_rate', 10.0).value
        )
        self.command_topic = str(
            self.declare_parameter('command_topic', '/cmd_vel/safe').value
        )

        defaults = {
            'FR': ('front_right', '316633543334', -1.0),
            'FL': ('front_left', '357B358B3135', 1.0),
            'RR': ('rear_right', '336636543334', -1.0),
            'RL': ('rear_left', '336536573334', 1.0),
        }
        self.serial_numbers = {}
        self.polarities = {}
        for wheel, (parameter_name, serial, polarity) in defaults.items():
            self.serial_numbers[wheel] = str(
                self.declare_parameter(
                    f'{parameter_name}.serial_number', serial
                ).value
            )
            self.polarities[wheel] = float(
                self.declare_parameter(
                    f'{parameter_name}.polarity', polarity
                ).value
            )

        self.drives = {}
        self.last_command_time = self.get_clock().now()
        self._connect_and_enable()
        self.create_subscription(
            Twist, self.command_topic, self._cmd_vel_callback, 10
        )
        self.create_timer(0.1, self._watchdog_callback)
        self.get_logger().info(
            f'Diff drive controller listening on {self.command_topic}'
        )

    def _connect_and_enable(self) -> None:
        for name, serial in self.serial_numbers.items():
            self.get_logger().info(f'Connecting to {name} ({serial})...')
            drive = odrive.find_sync(serial_number=serial)
            drive.clear_errors()
            drive.axis0.controller.config.control_mode = (
                ControlMode.VELOCITY_CONTROL
            )
            drive.axis0.controller.config.input_mode = InputMode.VEL_RAMP
            drive.axis0.controller.config.vel_ramp_rate = self.velocity_ramp_rate
            drive.axis0.controller.input_vel = 0.0
            drive.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.drives[name] = drive
            self.get_logger().info(f'{name} connected and enabled')

    def _cmd_vel_callback(self, message: Twist) -> None:
        left_turns, right_turns = wheel_turn_rates(
            message.linear.x,
            message.angular.z,
            self.wheel_radius,
            self.wheel_separation,
            self.gear_ratio,
            self.max_motor_turns_per_second,
        )
        self._set_motor_rates(left_turns, right_turns)
        self.last_command_time = self.get_clock().now()

    def _set_motor_rates(self, left_turns: float, right_turns: float) -> None:
        requested = {
            'FL': left_turns,
            'RL': left_turns,
            'FR': right_turns,
            'RR': right_turns,
        }
        for wheel, turns in requested.items():
            self.drives[wheel].axis0.controller.input_vel = (
                turns * self.polarities[wheel]
            )

    def _watchdog_callback(self) -> None:
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if age > self.command_timeout:
            self._set_motor_rates(0.0, 0.0)

    def _stop_motors(self) -> None:
        for drive in self.drives.values():
            try:
                drive.axis0.controller.input_vel = 0.0
                drive.axis0.requested_state = AxisState.IDLE
            except Exception as error:
                self.get_logger().error(f'Failed to idle motor: {error}')

    def destroy_node(self):
        self.get_logger().info('Stopping and idling motors')
        self._stop_motors()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DiffDriveController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
