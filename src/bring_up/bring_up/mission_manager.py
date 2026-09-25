"""
Choose what the robot drives to: nothing, the beacon, or the home station.

Modes: idle, follow (beacon, 3 m standoff) and home (home station, 1.5 m
standoff). Each mode sends one NavigateToPose goal with its own behavior tree;
the mode's target then moves the goal through /goal_update, so the robot
tracks a walking beacon or a relocated home station. The goal is cancelled
when the mode changes or its target goes stale, and re-sent when a fresh
target returns (retry_delay after Nav2 aborts).

Commands: /station/command (follow, home, stop, status) from the Meshtastic
bridge or `ros2 topic pub`, and /follow/enable (true = follow, false = idle).
A status line goes to /robot/status every status_period seconds.
"""
import math
import os

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

MODES = ('idle', 'follow', 'home')


class MissionLogic:
    """Decides when to send or cancel the Nav2 goal; no ROS calls."""

    def __init__(self, mode, stale_after, retry_delay):
        self.mode = mode
        self.stale_after = stale_after  # {'follow': s, 'home': s}
        self.retry_delay = retry_delay
        self.goal = 'idle'  # idle, sending, active, canceling
        self.goal_mode = None
        self._last_target = {'follow': None, 'home': None}
        self._retry_at = 0.0

    def on_target(self, mode, now):
        self._last_target[mode] = now

    def fresh(self, mode, now):
        last = self._last_target.get(mode)
        return last is not None and now - last <= self.stale_after[mode]

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self._retry_at = 0.0

    def step(self, now):
        """Return ('send', mode), ('cancel', reason) or None."""
        if self.goal == 'idle':
            if self.mode != 'idle' and self.fresh(self.mode, now) and now >= self._retry_at:
                self.goal = 'sending'
                self.goal_mode = self.mode
                return ('send', self.mode)
        elif self.goal == 'active':
            if self.mode != self.goal_mode:
                self.goal = 'canceling'
                return ('cancel', f'mode changed to {self.mode}')
            if not self.fresh(self.goal_mode, now):
                self.goal = 'canceling'
                return ('cancel', f'{self.goal_mode} target stale')
        return None

    def on_accepted(self):
        if self.goal == 'sending':
            self.goal = 'active'

    def on_finished(self, now, cancelled_by_us):
        """Goal rejected, aborted, succeeded or cancelled."""
        self.goal = 'idle'
        self.goal_mode = None
        self._retry_at = now if cancelled_by_us else now + self.retry_delay


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        trees = os.path.join(get_package_share_directory('bring_up'), 'behavior_trees')
        self.trees = {
            'follow': self.declare_parameter(
                'follow_tree', os.path.join(trees, 'follow_beacon.xml')).value,
            'home': self.declare_parameter(
                'home_tree', os.path.join(trees, 'return_home.xml')).value,
        }
        mode = self.declare_parameter('initial_mode', 'idle').value
        if mode not in MODES:
            raise ValueError(f'initial_mode must be one of {MODES}')
        self.logic = MissionLogic(
            mode=mode,
            stale_after={'follow': self.declare_parameter('follow_stale_after', 3.0).value,
                         'home': self.declare_parameter('home_stale_after', 3.0).value},
            retry_delay=self.declare_parameter('retry_delay', 2.0).value,
        )
        self.status_period = self.declare_parameter('status_period', 30.0).value
        self.targets = {'follow': None, 'home': None}
        self.robot = None
        self.goal_handle = None
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_update', 10)
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        self.create_subscription(PoseStamped, 'beacon/goal',
                                 lambda msg: self._target_cb('follow', msg), 10)
        self.create_subscription(PoseStamped, 'home/goal',
                                 lambda msg: self._target_cb('home', msg), 10)
        self.create_subscription(Odometry, 'odometry/global', self._odom_cb, 10)
        self.create_subscription(String, 'station/command', self._command_cb, 10)
        self.create_service(SetBool, 'follow/enable', self._enable_cb)
        self.create_timer(0.2, self._tick)
        self.create_timer(self.status_period, self._publish_status)
        self.get_logger().info(f'Mission manager ready in {mode} mode')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, mode, msg):
        self.targets[mode] = msg
        self.logic.on_target(mode, self._now())
        if mode == self.logic.goal_mode and self.logic.goal == 'active':
            self.goal_pub.publish(msg)  # Nav2's GoalUpdater moves the running goal

    def _odom_cb(self, msg):
        self.robot = msg.pose.pose.position

    def _set_mode(self, mode):
        if mode != self.logic.mode:
            self.get_logger().info(f'Mode: {self.logic.mode} -> {mode}')
        self.logic.set_mode(mode)

    def _command_cb(self, msg):
        command = msg.data.strip().lower()
        if command in ('follow', 'home'):
            self._set_mode(command)
        elif command == 'stop':
            self._set_mode('idle')
        elif command == 'status':
            self._publish_status()
        else:
            self.get_logger().warn(f'Unknown command: {msg.data!r}')

    def _enable_cb(self, request, response):
        self._set_mode('follow' if request.data else 'idle')
        response.success = True
        response.message = f'mode {self.logic.mode}'
        return response

    def _distance(self, mode):
        target = self.targets[mode]
        if target is None or self.robot is None or not self.logic.fresh(mode, self._now()):
            return None
        return math.hypot(target.pose.position.x - self.robot.x,
                          target.pose.position.y - self.robot.y)

    def _publish_status(self):
        parts = [f'mode={self.logic.mode}', f'nav={self.logic.goal}']
        for mode, label in (('follow', 'beacon'), ('home', 'home')):
            distance = self._distance(mode)
            parts.append(f'{label}={distance:.0f}m' if distance is not None else f'{label}=none')
        self.status_pub.publish(String(data=' '.join(parts)))

    def _tick(self):
        if not self.client.server_is_ready():
            return
        action = self.logic.step(self._now())
        if action is None:
            return
        kind, detail = action
        if kind == 'send':
            goal = NavigateToPose.Goal()
            goal.pose = self.targets[detail]
            goal.behavior_tree = self.trees[detail]
            self.get_logger().info(f'Starting {detail} goal')
            self.client.send_goal_async(goal).add_done_callback(self._goal_response)
        elif kind == 'cancel' and self.goal_handle is not None:
            self.get_logger().warn(f'Stopping goal ({detail})')
            self.goal_handle.cancel_goal_async()

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self.logic.on_finished(self._now(), cancelled_by_us=False)
            return
        self.goal_handle = handle
        self.logic.on_accepted()
        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, future):
        status = future.result().status
        cancelled = status == GoalStatus.STATUS_CANCELED
        if not cancelled:
            self.get_logger().warn(f'Goal ended with status {status}; retrying')
        self.goal_handle = None
        self.logic.on_finished(self._now(), cancelled_by_us=cancelled)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
