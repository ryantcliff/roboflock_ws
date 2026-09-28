#!/usr/bin/env python3
"""Exercise the real beacon executable and navsat conversion without hardware."""
import argparse
import math
import os
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=91)
    parser.add_argument('--log', default='/tmp/roboflock-beacon-test.log')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error('domain-id must be in 0..101')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = '/tmp/roboflock-beacon-ros'

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, NavSatFix
    from robot_localization.srv import FromLL

    rclpy.init()
    node = rclpy.create_node('beacon_test_fixture')
    children = []
    output = []
    gps_output = []
    log = open(args.log, 'w')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def until(predicate, seconds=8.0):
        end = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > end:
                raise AssertionError(f'Timed out; see {args.log}')
            rclpy.spin_once(node, timeout_sec=0.02)

    def start(command):
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children.append(child)
        return child

    def stop(child):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()

    # Real, non-degenerate datum (UB North Campus). lat/lon 0,0 makes
    # navsat_transform abort inside GeographicLib ("MGRS string too short").
    datum_lat, datum_lon = 43.0008, -78.7890
    a, e2 = 6378137.0, 6.69437999014e-3
    s2 = math.sin(math.radians(datum_lat)) ** 2
    prime_vertical = a / math.sqrt(1 - e2 * s2)
    meridian = a * (1 - e2) / (1 - e2 * s2) ** 1.5

    def fix(lat=datum_lat, lon=datum_lon):
        msg = NavSatFix()
        msg.header.stamp = node.get_clock().now().to_msg()
        # Coincident test sensor: no guessed real-world antenna offset.
        msg.header.frame_id = 'base_link'
        msg.status.status = 0
        msg.latitude, msg.longitude, msg.altitude = lat, lon, 0.0
        msg.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = 2
        return msg

    try:
        spin(2)
        if [n for n in node.get_node_names() if n != node.get_name()]:
            raise RuntimeError('Test domain is occupied; choose another --domain-id')
        beacon_pub = node.create_publisher(NavSatFix, '/gps/beacon/fix', 10)
        robot_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)
        imu_pub = node.create_publisher(Imu, '/imu', 10)
        odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
        node.create_subscription(PoseStamped, '/beacon/map_pose', output.append, 10)
        goals = []
        node.create_subscription(PoseStamped, '/goal_update', goals.append, 10)
        node.create_subscription(Odometry, '/odometry/gps', gps_output.append,
                                 qos_profile_sensor_data)
        start(['ros2', 'run', 'beacon_pkg', 'beacon_goalpose', '--ros-args',
               '-p', 'request_timeout:=0.4'])
        until(lambda: beacon_pub.get_subscription_count() > 0)
        spin(0.5)
        beacon_pub.publish(fix())
        spin(0.5)
        assert not output, 'Published before GPS conversion was ready'
        print('PASS no output before datum readiness', flush=True)

        def sensors():
            now = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'base_link'
            imu.orientation.w = 1.0
            imu_pub.publish(imu)
            robot_pub.publish(fix())

        timer = node.create_timer(0.05, sensors)
        navsat = start(['ros2', 'run', 'robot_localization', 'navsat_transform_node',
                        '--ros-args',  # UTM mode, as in ekf_navsat_params.yaml
                        '-p', 'magnetic_declination_radians:=0.0', '-p', 'yaw_offset:=0.0',
                        '-p', 'delay:=0.0', '-p', 'zero_altitude:=true'])
        until(lambda: len(gps_output) > 2)
        spin(0.3)

        def check_position(msg, x, y):
            output.clear()
            beacon_pub.publish(msg)
            until(lambda: bool(output), 3)
            pose = output[-1]
            assert pose.header.frame_id == 'map'
            assert pose.header.stamp == msg.header.stamp, 'Measurement timestamp changed'
            assert math.hypot(pose.pose.position.x - x, pose.pose.position.y - y) < 0.05, pose
            assert pose.pose.position.z == 0.0
            spin(0.1)

        check_position(fix(), 0, 0)
        # Independent WGS84 small-offset expectations at the datum latitude.
        east = math.degrees(10 / (prime_vertical * math.cos(math.radians(datum_lat))))
        north = math.degrees(10 / meridian)
        check_position(fix(lon=datum_lon + east), 10, 0)
        check_position(fix(lat=datum_lat + north), 0, 10)
        print('PASS same location, 10 m east, 10 m north and original timestamps', flush=True)

        # Follow targets beyond max_goal_distance (20 m) are pulled in toward the robot.
        goals.clear()
        check_position(fix(lon=datum_lon + 4 * east), 40, 0)
        until(lambda: bool(goals), 3)
        goal = goals[-1].pose.position
        assert abs(goal.x - 20.0) < 0.1 and abs(goal.y) < 0.1, goal
        goals.clear()
        check_position(fix(lon=datum_lon + east), 10, 0)
        until(lambda: bool(goals), 3)
        assert abs(goals[-1].pose.position.x - 10.0) < 0.05, goals[-1].pose.position
        print('PASS /goal_update: 40 m target clamped to 20 m, 10 m target unchanged', flush=True)

        invalid = []
        msg = fix()
        msg.status.status = -1
        invalid.append(msg)
        invalid.extend([fix(lat=float('nan')), fix(lat=91.0), fix(lon=181.0)])
        msg = fix()
        msg.altitude = float('nan')
        invalid.append(msg)
        msg = fix()
        msg.header.stamp.sec -= 10
        invalid.append(msg)
        msg = fix()
        msg.header.stamp.sec += 10
        invalid.append(msg)
        msg = fix()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        invalid.append(msg)
        output.clear()
        for msg in invalid:
            beacon_pub.publish(msg)
            spin(0.15)
        assert not output, 'Published an invalid or stale fix'
        assert node.count_publishers('/goal_pose') == 0, 'Unexpected navigation goal publisher'
        print(
            'PASS invalid/no-fix/stale/future/unstamped rejection; no navigation goals',
            flush=True)

        stop(navsat)
        timer.cancel()
        spin(2.2)
        output.clear()
        beacon_pub.publish(fix())
        spin(0.6)
        assert not output, 'Published after GPS readiness expired'
        print('PASS GPS conversion outage suppresses output', flush=True)

        # Fault injection: a service returning late or after a newer measurement.
        ready_pub = node.create_publisher(Odometry, '/odometry/gps', 10)

        def heartbeat():
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            ready_pub.publish(msg)
        heartbeat_timer = node.create_timer(0.05, heartbeat)
        behavior = {'delay': 0.0, 'replace': False, 'nan': False, 'calls': 0}

        def convert(request, response):
            behavior['calls'] += 1
            if behavior['replace']:
                replacement = fix()
                replacement.status.status = -1
                beacon_pub.publish(replacement)
            time.sleep(behavior['delay'])
            response.map_point.x = float('nan') if behavior['nan'] else 7.0
            response.map_point.y = 8.0
            return response
        service = node.create_service(FromLL, '/fromLL', convert)
        spin(1)
        behavior['delay'] = 0.7
        output.clear()
        calls = behavior['calls']
        beacon_pub.publish(fix())
        until(lambda: behavior['calls'] > calls)
        spin(0.4)
        assert not output, 'Published a timed-out response'
        behavior.update(delay=0.15, replace=True)
        calls = behavior['calls']
        beacon_pub.publish(fix())
        until(lambda: behavior['calls'] > calls)
        spin(0.3)
        assert not output, 'Published a response superseded by a newer invalid fix'
        behavior.update(delay=0.0, replace=False, nan=True)
        calls = behavior['calls']
        beacon_pub.publish(fix())
        until(lambda: behavior['calls'] > calls)
        spin(0.2)
        assert not output, 'Published a non-finite conversion result'
        behavior['nan'] = False
        check_position(fix(), 7, 8)
        print('PASS timeout, superseded response, invalid response and recovery', flush=True)
        heartbeat_timer.cancel()
        node.destroy_service(service)
        print(f'PASS beacon conversion integration. Log: {args.log}', flush=True)
    finally:
        for child in reversed(children):
            stop(child)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
