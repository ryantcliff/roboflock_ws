#!/usr/bin/env python3
"""
Activate real Nav2 against a stationary synthetic robot, then request a path.

Run after sourcing ROS and this workspace. Uses an isolated ROS domain (87 by
 default); refuses to run if any nodes already exist there. No motor nodes,
 hardware drivers or motion simulation are involved. Optional EKF modes use
 synthetic IMU, laser odometry and GPS measurements.
"""

import argparse
import math
import os
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=87)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument(
        '--localization', choices=['synthetic', 'local', 'gps'],
        default='synthetic',
        help='Use synthetic TF or actual robot_localization EKFs')
    parser.add_argument('--log', default='/tmp/roboflock-nav2-smoke.log')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error('domain-id must be between 0 and 101')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/roboflock-smoke-ros')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan, Imu, NavSatFix, NavSatStatus
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('nav2_smoke_fixture')
    process = None
    localization_process = None
    log = None
    try:
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        others = [n for n in node.get_node_names() if n != node.get_name()]
        if others:
            raise RuntimeError(f'Domain {args.domain_id} is occupied: {others}')

        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(OccupancyGrid, '/map', durable)
        odom_topic = '/odometry/local' if args.localization == 'synthetic' else '/odom_rf2o'
        odom_pub = node.create_publisher(Odometry, odom_topic, 10)
        imu_pub = node.create_publisher(Imu, '/imu/data', 10)
        gps_pub = node.create_publisher(NavSatFix, '/gps/robot/fix', 10)
        received = {'local': False, 'global': False, 'gps': False}
        latest = {}
        # Synthetic robot motion along +x (east): pose x and velocity vx.
        motion = {'x': 0.0, 'vx': 0.0}
        # A robot driving true east reads magnetic yaw -(declination - grid
        # convergence); ekf_navsat_params.yaml sets declination 0.17872172 rad.
        gps_lat, gps_lon = 42.9, -78.7
        convergence = math.atan(math.tan(math.radians(gps_lon + 81.0))
                                * math.sin(math.radians(gps_lat)))  # UTM zone 17
        yaw = -(0.17872172 - convergence) if args.localization == 'gps' else 0.0

        def received_odom(kind, msg):
            if all(math.isfinite(v) for v in (msg.pose.pose.position.x, msg.pose.pose.position.y)):
                received[kind] = True
                latest[kind] = msg.pose.pose.position
        for kind, topic in [('local', '/odometry/local'), ('global', '/odometry/global'),
                            ('gps', '/odometry/gps')]:
            node.create_subscription(Odometry, topic,
                                     lambda msg, kind=kind: received_odom(kind, msg), 10)
        scan_pub = node.create_publisher(LaserScan, '/scan', 10)
        tf_edges = set()

        def record_tf(msg):
            for tf in msg.transforms:
                edge = (tf.header.frame_id, tf.child_frame_id)
                tf_edges.add(edge)

        node.create_subscription(TFMessage, '/tf', record_tf, 100)
        node.create_subscription(TFMessage, '/tf_static', record_tf, durable)
        dynamic_tf = TransformBroadcaster(node) if args.localization == 'synthetic' else None
        static_tf = StaticTransformBroadcaster(node)

        def transform(parent, child):
            tf = TransformStamped()
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            return tf

        static_transforms = [transform('base_link', 'laser'), transform('base_link', 'gps')]
        if args.localization != 'gps':
            static_transforms.append(transform('map', 'odom'))
        static_tf.sendTransform(static_transforms)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05
        grid.info.width = grid.info.height = 200
        grid.info.origin.position.x = grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 40000

        def publish_map():
            grid.header.stamp = node.get_clock().now().to_msg()
            map_pub.publish(grid)

        def publish_sensors():
            motion['x'] += motion['vx'] * 0.05
            stamp = node.get_clock().now().to_msg()
            if args.localization == 'synthetic':
                dynamic_tf.sendTransform(transform('odom', 'base_link'))
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = motion['x'] * math.cos(yaw)
            odom.pose.pose.position.y = motion['x'] * math.sin(yaw)
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            odom.twist.twist.linear.x = motion['vx']
            for i in range(6):
                odom.pose.covariance[i * 7] = 0.01
                odom.twist.covariance[i * 7] = 0.01
            odom_pub.publish(odom)
            if args.localization != 'synthetic':
                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = 'base_link'
                imu.orientation.z = math.sin(yaw / 2)
                imu.orientation.w = math.cos(yaw / 2)
                imu.linear_acceleration.z = 9.80665
                for i in range(3):
                    imu.orientation_covariance[i * 4] = 0.01
                    imu.angular_velocity_covariance[i * 4] = 0.01
                    imu.linear_acceleration_covariance[i * 4] = 0.01
                imu_pub.publish(imu)
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'laser'
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min = 0.1
            scan.range_max = 10.0
            scan.scan_time = 0.05
            scan.ranges = [8.0] * 360
            scan_pub.publish(scan)

        def publish_gps():
            fix = NavSatFix()
            fix.header.stamp = node.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            # The robot drives true east, so distance maps to longitude.
            east_deg = math.degrees(motion['x'] / (6388838.29 * math.cos(math.radians(gps_lat))))
            fix.latitude, fix.longitude, fix.altitude = gps_lat, gps_lon + east_deg, 150.0
            fix.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            gps_pub.publish(fix)

        if args.localization == 'gps':
            node.create_timer(0.2, publish_gps)
        node.create_timer(0.05, publish_sensors)
        node.create_timer(1.0, publish_map)
        publish_map()
        log = open(args.log, 'w')
        if args.localization != 'synthetic':
            localization_process = subprocess.Popen([
                'ros2', 'launch', 'bring_up', 'localization.launch.py',
                'gps_localization:=' + ('true' if args.localization == 'gps' else 'false'),
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        process = subprocess.Popen([
            'ros2', 'launch', 'bring_up', 'nav2.launch.py', 'autostart:=true',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + args.timeout

        def wait(future, request_timeout=None):
            request_deadline = time.monotonic() + request_timeout if request_timeout else deadline
            while not future.done():
                if time.monotonic() >= request_deadline and request_timeout:
                    future.cancel()
                    return None
                if process.poll() is not None:
                    raise RuntimeError(f'Nav2 exited; see {args.log}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'Nav2 test timed out; see {args.log}')
                rclpy.spin_once(node, timeout_sec=0.05)
            return future.result()

        names = ['controller_server', 'planner_server', 'smoother_server',
                 'behavior_server', 'bt_navigator', 'waypoint_follower',
                 'velocity_smoother']
        clients = {name: node.create_client(GetState, f'/{name}/get_state')
                   for name in names}
        pending = set(names)
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            with open(args.log) as output:
                if 'Failed to bring up all requested nodes' in output.read():
                    raise RuntimeError(f'Nav2 activation failed; see {args.log}')
            for name in list(pending):
                if clients[name].service_is_ready():
                    response = wait(clients[name].call_async(GetState.Request()), 2.0)
                    if response is not None and response.current_state.id == 3:
                        print(f'PASS {name}: active', flush=True)
                        pending.remove(name)
        if pending:
            raise TimeoutError(f'Nodes did not activate: {sorted(pending)}; see {args.log}')

        client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        while not client.server_is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError('Planner action unavailable')
            rclpy.spin_once(node, timeout_sec=0.05)
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = 1.0
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = 'GridBased'
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise AssertionError('Planner rejected the test goal')
        result = wait(handle.get_result_async())
        if result.status != 4 or len(result.result.path.poses) < 2:
            raise AssertionError(f'Planning failed: status={result.status}')
        endpoint = result.result.path.poses[-1].pose.position
        if math.hypot(endpoint.x - 1.0, endpoint.y) > 0.15:
            raise AssertionError('Path endpoint does not reach the requested goal')
        print(f'PASS path to (1, 0): {len(result.result.path.poses)} poses', flush=True)
        expected = [] if args.localization == 'synthetic' else ['local']
        if args.localization == 'gps':
            expected += ['global', 'gps']
        while not all(received[k] for k in expected) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not all(received[k] for k in expected):
            raise AssertionError(f'Missing filtered odometry: {received}')
        if expected:
            print(f'PASS filtered odometry streams: {expected}')
        for edge in [('map', 'odom'), ('odom', 'base_link'), ('base_link', 'laser')]:
            if edge not in tf_edges:
                raise AssertionError(f'Missing TF edge {edge}: {tf_edges}')
        expected_publishers = {
            'synthetic': ['nav2_smoke_fixture'],
            'local': ['ekf_filter_node_odom'],
            'gps': ['ekf_filter_node_map', 'ekf_filter_node_odom'],
        }[args.localization]
        publishers = sorted(info.node_name for info in node.get_publishers_info_by_topic('/tf'))
        if publishers != expected_publishers:
            raise AssertionError(f'Unexpected dynamic TF publishers: {publishers}')
        print(f'PASS TF chain and expected dynamic publishers: {publishers}')
        if args.localization == 'gps':
            # Drive 10 m east at 1 m/s; GPS and laser odometry move together.
            start = {k: (latest[k].x, latest[k].y) for k in ('global', 'gps')}
            motion['vx'] = 1.0
            walk_end = time.monotonic() + 10.0
            while time.monotonic() < walk_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            motion['vx'] = 0.0
            settle_end = time.monotonic() + 3.0
            while time.monotonic() < settle_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            moved = {k: (latest[k].x - start[k][0], latest[k].y - start[k][1])
                     for k in ('global', 'gps')}
            for kind, (dx, dy) in moved.items():
                if abs(math.hypot(dx, dy) - motion['x']) > 0.5:
                    raise AssertionError(f'/odometry/{kind} moved {math.hypot(dx, dy):.2f} m; '
                                         f'expected {motion["x"]:.2f} m')
            gap = math.hypot(moved['global'][0] - moved['gps'][0],
                             moved['global'][1] - moved['gps'][1])
            if gap > 0.2:
                raise AssertionError(f'/odometry/global is {gap:.2f} m from /odometry/gps')
            print(f'PASS GPS walk: global moved {math.hypot(*moved["global"]):.2f} m, '
                  f'gps {math.hypot(*moved["gps"]):.2f} m, gap {gap:.2f} m', flush=True)
        print(
            'PASS hardware-free Nav2 activation and planning '
            f'({args.localization}). Log: {args.log}')
    finally:
        for child in (process, localization_process):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        if log is not None:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
