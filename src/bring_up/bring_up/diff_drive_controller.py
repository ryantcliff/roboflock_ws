import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

<<<<<<< HEAD
from bring_up.drive_kinematics import wheel_turn_rates
=======
WHEEL_RADIUS = 0.254
WHEEL_SEPARATION = 0.3
GEAR_RATIO = 30.0
SERIAL_NUMBERS = {
    "FR": "316633543334",
    "FL": "357B358B3135",
    "RR": "336636543334",
    "RL": "336536573334",
}
>>>>>>> upstream/dev

FAST_DECEL = 50.0
STD_ACCEL  = 10.0

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
<<<<<<< HEAD
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
=======
            dev = odrive.find_sync(serial_number=serial)
            dev.clear_errors()
            dev.axis0.controller.config.input_mode = InputMode.VEL_RAMP
            dev.axis0.controller.config.vel_ramp_rate = STD_ACCEL
            dev.axis0.controller.config.vel_gain = 0.02          # default ~0.16, lower = less resistance
            dev.axis0.controller.config.vel_integrator_gain = 0.05
            dev.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.drives[name] = dev
            self.get_logger().info(f'{name} connected and enabled')

    def _set_velocity(self, name: str, target_vel: float):
        dev = self.drives[name]
        current_vel = dev.axis0.encoder.vel_estimate

        slowing_down = abs(target_vel) < abs(current_vel)
        changing_dir = target_vel * current_vel < 0

        if slowing_down or changing_dir:
            dev.axis0.controller.config.vel_ramp_rate = FAST_DECEL
        else:
            dev.axis0.controller.config.vel_ramp_rate = STD_ACCEL

        dev.axis0.controller.input_vel = target_vel

    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        print(f"cmd_vel → linear: {v}, angular: {w}")

        v_left  = v - (w * WHEEL_SEPARATION / 2.0)
        v_right = v + (w * WHEEL_SEPARATION / 2.0)
        print(f"v_left: {v_left} m/s, v_right: {v_right} m/s")

        turns_left  = (v_left  / (2.0 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO
        turns_right = (v_right / (2.0 * math.pi * WHEEL_RADIUS)) * GEAR_RATIO

        self._set_velocity("FL",  turns_left)
        self._set_velocity("RL",  turns_left)
        self._set_velocity("FR", -turns_right)
        self._set_velocity("RR", -turns_right)
>>>>>>> upstream/dev

    def destroy_node(self):
        self.get_logger().info('Stopping and idling motors')
        self._stop_motors()
        return super().destroy_node()

<<<<<<< HEAD

def main(args=None) -> None:
=======
def main(args=None):
>>>>>>> upstream/dev
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
