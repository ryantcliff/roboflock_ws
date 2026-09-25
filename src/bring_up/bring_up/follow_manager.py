"""
Start and stop Nav2 beacon following.

Sends one NavigateToPose goal that runs the follow_beacon.xml behavior tree;
beacon_goalpose then moves the target through /goal_update. The goal is
cancelled when following is disabled or the beacon target goes stale, and
re-sent (after retry_delay) when fresh targets return or Nav2 aborts.
"""
import os

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool


class FollowLogic:
    """Decides when to send or cancel the follow goal; no ROS calls."""

    def __init__(self, enabled, stale_after, retry_delay):
        self.enabled = enabled
        self.stale_after = stale_after
        self.retry_delay = retry_delay
        self.goal = 'idle'  # idle, sending, active, canceling
        self._last_target = None
        self._retry_at = 0.0

    def on_target(self, now):
        self._last_target = now

    def fresh(self, now):
        return self._last_target is not None and now - self._last_target <= self.stale_after

    def step(self, now):
        """Return 'send', 'cancel' or None."""
        if self.goal == 'idle':
            if self.enabled and self.fresh(now) and now >= self._retry_at:
                self.goal = 'sending'
                return 'send'
        elif self.goal == 'active':
            if not self.enabled or not self.fresh(now):
                self.goal = 'canceling'
                return 'cancel'
        return None

    def on_accepted(self):
        if self.goal == 'sending':
            self.goal = 'active'

    def on_finished(self, now, cancelled_by_us):
        """Goal rejected, aborted, succeeded or cancelled."""
        self.goal = 'idle'
        self._retry_at = now if cancelled_by_us else now + self.retry_delay


class FollowManager(Node):

    def __init__(self):
        super().__init__('follow_manager')
        default_tree = os.path.join(
            get_package_share_directory('bring_up'), 'behavior_trees', 'follow_beacon.xml')
        self.tree = self.declare_parameter('behavior_tree', default_tree).value
        self.logic = FollowLogic(
            enabled=self.declare_parameter('enabled', False).value,
            stale_after=self.declare_parameter('stale_after', 3.0).value,
            retry_delay=self.declare_parameter('retry_delay', 2.0).value,
        )
        self.target = None
        self.goal_handle = None
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, 'goal_update', self._target_cb, 10)
        self.create_service(SetBool, 'follow/enable', self._enable_cb)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'Follow manager ready (enabled={self.logic.enabled}), tree {self.tree}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg):
        self.target = msg
        self.logic.on_target(self._now())

    def _enable_cb(self, request, response):
        self.logic.enabled = request.data
        response.success = True
        response.message = 'following enabled' if request.data else 'following disabled'
        self.get_logger().info(response.message)
        return response

    def _tick(self):
        if not self.client.server_is_ready():
            return
        action = self.logic.step(self._now())
        if action == 'send':
            goal = NavigateToPose.Goal()
            goal.pose = self.target
            goal.behavior_tree = self.tree
            self.get_logger().info('Beacon fresh: starting follow goal')
            self.client.send_goal_async(goal).add_done_callback(self._goal_response)
        elif action == 'cancel' and self.goal_handle is not None:
            reason = 'disabled' if not self.logic.enabled else 'beacon stale'
            self.get_logger().warn(f'Stopping follow goal ({reason})')
            self.goal_handle.cancel_goal_async()

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the follow goal')
            self.logic.on_finished(self._now(), cancelled_by_us=False)
            return
        self.goal_handle = handle
        self.logic.on_accepted()
        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, future):
        status = future.result().status
        cancelled = status == GoalStatus.STATUS_CANCELED
        if not cancelled:
            self.get_logger().warn(f'Follow goal ended with status {status}; retrying')
        self.goal_handle = None
        self.logic.on_finished(self._now(), cancelled_by_us=cancelled)


def main(args=None):
    rclpy.init(args=args)
    node = FollowManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
