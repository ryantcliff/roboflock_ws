from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float
    angular_z: float
    received_at: float


class SafetyController:
    """Select a velocity source and enforce stop conditions."""

    def __init__(
        self,
        command_timeout: float,
        range_timeout: float,
        stop_distance: float,
        require_ultrasonic: bool,
    ) -> None:
        self.command_timeout = command_timeout
        self.range_timeout = range_timeout
        self.stop_distance = stop_distance
        self.require_ultrasonic = require_ultrasonic
        self.commands: Dict[str, VelocityCommand] = {}
        self.ranges: Dict[str, Tuple[float, float]] = {}
        self.emergency_stop_active = False
        self.stop_latched = False

    def set_command(
        self,
        source: str,
        linear_x: float,
        angular_z: float,
        now: float,
    ) -> None:
        self.commands[source] = VelocityCommand(linear_x, angular_z, now)

    def set_range(self, sensor: str, distance: float, now: float) -> None:
        self.ranges[sensor] = (distance, now)

    def set_emergency_stop(self, active: bool) -> None:
        self.emergency_stop_active = active
        if active:
            self.stop_latched = True

    def reset(self) -> bool:
        if self.emergency_stop_active:
            return False
        self.stop_latched = False
        return True

    def evaluate(self, now: float) -> Tuple[float, float, str]:
        if self.stop_latched:
            return 0.0, 0.0, 'emergency_stop'

        command = self._active_command(now)
        if command is None:
            return 0.0, 0.0, 'command_timeout'

        if command.linear_x > 0.0:
            range_state = self._forward_range_state(now)
            if range_state is not None:
                return 0.0, 0.0, range_state

        return command.linear_x, command.angular_z, 'running'

    def _active_command(self, now: float) -> Optional[VelocityCommand]:
        for source in ('teleop', 'nav'):
            command = self.commands.get(source)
            if command and now - command.received_at <= self.command_timeout:
                return command
        return None

    def _forward_range_state(self, now: float) -> Optional[str]:
        expected = ('left', 'center', 'right')
        if self.require_ultrasonic:
            if any(sensor not in self.ranges for sensor in expected):
                return 'ultrasonic_unavailable'
            if any(
                now - self.ranges[sensor][1] > self.range_timeout
                for sensor in expected
            ):
                return 'ultrasonic_stale'

        for distance, received_at in self.ranges.values():
            if now - received_at <= self.range_timeout:
                if 0.0 < distance < self.stop_distance:
                    return 'ultrasonic_stop'
        return None
