"""
Bridge between the robot's Meshtastic node and ROS.

- Position reports from the home station node become /home/fix. The last
  position is republished at 1 Hz with a fresh stamp (Nav2's GoalUpdater only
  accepts newer goals, and a parked station may report only every few minutes),
  until the last real report is older than max_home_age.
- Text messages ("follow", "home", "stop") from allowed nodes become
  /station/command.
- /robot/status text is sent on the status channel, at most every
  status_min_interval seconds to save airtime.
"""
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

COMMANDS = ('follow', 'home', 'stop', 'status')


def parse_node_id(value):
    """Accept 1819531008, '1819531008' or '!6c7b4a00'; return an int."""
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.startswith('!'):
        return int(text[1:], 16)
    return int(text)


def parse_position(packet):
    """Return (lat, lon, alt, precision_bits) from a POSITION_APP packet, else None."""
    decoded = packet.get('decoded', {})
    if decoded.get('portnum') != 'POSITION_APP':
        return None
    position = decoded.get('position', {})
    if 'latitudeI' in position and 'longitudeI' in position:
        lat = position['latitudeI'] * 1e-7
        lon = position['longitudeI'] * 1e-7
    elif 'latitude' in position and 'longitude' in position:
        lat, lon = position['latitude'], position['longitude']
    else:
        return None
    if lat == 0.0 and lon == 0.0:  # node has no GPS fix yet
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return lat, lon, float(position.get('altitude', 0.0)), position.get('precisionBits', 32)


def parse_command(packet):
    """Return a lower-case command word from a TEXT_MESSAGE_APP packet, else None."""
    decoded = packet.get('decoded', {})
    if decoded.get('portnum') != 'TEXT_MESSAGE_APP':
        return None
    word = str(decoded.get('text', '')).strip().lower()
    return word if word in COMMANDS else None


class MeshtasticBridge(Node):

    def __init__(self):
        super().__init__('meshtastic_bridge')
        self.port = self.declare_parameter('port', '/dev/meshtastic').value
        # Node ids may be given as numbers or '!hex' strings.
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.home_node = parse_node_id(
            self.declare_parameter('home_node_id', '0', any_type).value)
        commanders = self.declare_parameter('command_node_ids', '3145785460', any_type).value
        if not isinstance(commanders, (list, tuple)):
            commanders = str(commanders).split(',')  # default: tablet e074
        self.command_nodes = [parse_node_id(n) for n in commanders]
        self.status_channel = self.declare_parameter('status_channel', 1).value
        self.max_home_age = self.declare_parameter('max_home_age', 1800.0).value
        self.status_interval = self.declare_parameter('status_min_interval', 30.0).value

        self.home = None  # (lat, lon, alt, received_at)
        self.last_status_sent = None
        self.iface = None
        self.home_pub = self.create_publisher(NavSatFix, 'home/fix', 10)
        self.command_pub = self.create_publisher(String, 'station/command', 10)
        self.create_subscription(String, 'robot/status', self._status_cb, 10)
        self.create_timer(1.0, self._republish_home)
        self.create_timer(5.0, self._ensure_connected)
        self._ensure_connected()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _ensure_connected(self):
        if self.iface is not None:
            return
        import meshtastic.serial_interface
        from pubsub import pub
        try:
            self.iface = meshtastic.serial_interface.SerialInterface(devPath=self.port)
        except Exception as error:  # device missing or busy: retry on the timer
            self.get_logger().warn(f'Meshtastic node not available on {self.port}: {error}',
                                   throttle_duration_sec=60.0)
            self.iface = None
            return
        pub.subscribe(self._on_receive, 'meshtastic.receive')
        pub.subscribe(self._on_lost, 'meshtastic.connection.lost')
        self.get_logger().info(f'Connected to Meshtastic node on {self.port}')

    def _on_lost(self, interface=None):
        self.get_logger().warn('Meshtastic connection lost; reconnecting')
        self.iface = None

    def _on_receive(self, packet, interface=None):
        sender = packet.get('from')
        position = parse_position(packet)
        if position is not None and sender == self.home_node:
            lat, lon, alt, bits = position
            if bits < 32:
                self.get_logger().warn(
                    f'Home position precision is {bits} bits; set the channel position '
                    'precision to 32 on the home node', throttle_duration_sec=600.0)
            self.home = (lat, lon, alt, self._now())
            self.get_logger().info(f'Home station at {lat:.6f}, {lon:.6f}')
            return
        command = parse_command(packet)
        if command is not None and sender in self.command_nodes:
            self.get_logger().info(f'Command from station: {command}')
            self.command_pub.publish(String(data=command))

    def _republish_home(self):
        if self.home is None:
            return
        lat, lon, alt, received_at = self.home
        if self._now() - received_at > self.max_home_age:
            self.get_logger().warn('Home position is too old; dropping it',
                                   throttle_duration_sec=300.0)
            return
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = lat, lon, alt
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.home_pub.publish(fix)

    def _status_cb(self, msg):
        now = self._now()
        if self.iface is None or (self.last_status_sent is not None and
                                  now - self.last_status_sent < self.status_interval):
            return
        try:
            self.iface.sendText(msg.data, channelIndex=self.status_channel)
            self.last_status_sent = now
        except Exception as error:
            self.get_logger().warn(f'Could not send status: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = MeshtasticBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.iface is not None:
            node.iface.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
