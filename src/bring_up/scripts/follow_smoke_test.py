#!/usr/bin/env python3
"""
Hardware-free beacon-following and return-home test on the full stack.

Launches bringup.launch.py with follow:=true sim:=true (real EKFs,
navsat_transform, Nav2, beacon_goalpose/home_goalpose, mission_manager and
twist_mux; fake robot and sensors). It plays the beacon, the home station's
/home/fix (as meshtastic_bridge publishes it) and the station's commands:
  A. beacon stands 6 m ahead: robot approaches and stops near the 3 m standoff
  B. beacon walks 12 m east and stops: robot follows and stops near 3 m again
  C. beacon walks north, then its fixes stop: robot halts within a few seconds
  D. fixes resume farther away: robot starts following again
  E. "home": robot drives to the home station and stops near 1.5 m
  F. home station moves 10 m: robot follows it
  G. "stop": robot halts and reports mode=idle on "status"
  H. "follow": robot returns to the beacon
Uses an isolated ROS domain (88 by default) and refuses an occupied one.
"""

import argparse
import math
import os
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=88)
    parser.add_argument('--log', default='/tmp/roboflock-follow-smoke.log')
    parser.add_argument('--beacon-speed', type=float, default=0.4,
                        help='Walking speed of the simulated beacon (m/s)')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error('domain-id must be between 0 and 101')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/roboflock-smoke-ros')

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import String

    from bring_up.sim_geo import LocalTangent

    rclpy.init()
    node = rclpy.create_node('follow_smoke_fixture')
    tangent = LocalTangent(43.0008, -78.7890)  # fake_robot's default datum
    robot = {}
    beacon = {'x': 6.0, 'y': 0.0, 'on': True}
    home = {'x': -4.0, 'y': 6.0, 'on': False}
    statuses = []
    launch = None
    log = open(args.log, 'w')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if launch is not None and launch.poll() is not None:
                raise RuntimeError(f'Launch exited; see {args.log}')
            rclpy.spin_once(node, timeout_sec=0.05)

    def distance(target=beacon):
        return math.hypot(robot['x'] - target['x'], robot['y'] - target['y'])

    def position():
        return robot['x'], robot['y']

    def moved_during(seconds):
        start = position()
        spin(seconds)
        return math.hypot(robot['x'] - start[0], robot['y'] - start[1])

    def settle(timeout, low=2.4, high=4.0, target=beacon):
        """Wait until the robot rests within [low, high] m of the target."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if low <= distance(target) <= high and moved_during(3.0) < 0.1:
                return distance(target)
            spin(0.5)
        raise AssertionError(
            f'Robot did not settle: distance {distance(target):.2f} m; see {args.log}')

    def command(word):
        command_pub.publish(String(data=word))
        spin(0.5)

    def walk(dx, dy, speed=None, stop_after=None):
        """Move the beacon; stop publishing fixes after stop_after seconds."""
        speed = speed or args.beacon_speed
        steps = int(math.hypot(dx, dy) / speed / 0.2)
        for i in range(steps):
            if stop_after is not None and i * 0.2 >= stop_after:
                beacon['on'] = False
            beacon['x'] += dx / steps
            beacon['y'] += dy / steps
            spin(0.2)

    def publish_beacon():
        if not beacon['on']:
            return
        fix = NavSatFix()
        fix.header.stamp = node.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = tangent.to_latlon(beacon['x'], beacon['y'])
        fix.altitude = 180.0
        fix.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        beacon_pub.publish(fix)

    def publish_home():
        # meshtastic_bridge republishes the last station position at 1 Hz.
        if not home['on']:
            return
        fix = NavSatFix()
        fix.header.stamp = node.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = tangent.to_latlon(home['x'], home['y'])
        fix.altitude = 180.0
        home_pub.publish(fix)

    def truth(msg):
        robot['x'], robot['y'] = msg.pose.position.x, msg.pose.position.y

    try:
        spin(2.0)
        others = [n for n in node.get_node_names() if n != node.get_name()]
        if others:
            raise RuntimeError(f'Domain {args.domain_id} is occupied: {others}')
        beacon_pub = node.create_publisher(NavSatFix, '/gps/beacon/fix', 10)
        node.create_subscription(PoseStamped, '/sim/pose', truth, 10)
        node.create_timer(0.2, publish_beacon)
        home_pub = node.create_publisher(NavSatFix, '/home/fix', 10)
        node.create_timer(1.0, publish_home)
        command_pub = node.create_publisher(String, '/station/command', 10)
        node.create_subscription(String, '/robot/status', lambda m: statuses.append(m.data), 10)
        launch = subprocess.Popen([
            'ros2', 'launch', 'bring_up', 'bringup.launch.py', 'follow:=true', 'slam:=false',
            'hardware:=false', 'sim:=true', 'fake_beacon:=false',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        end = time.monotonic() + 30.0
        while 'x' not in robot:
            if time.monotonic() > end:
                raise TimeoutError(f'No /sim/pose; see {args.log}')
            spin(0.2)

        d = settle(90.0)
        print(f'PASS A: approached a standing beacon and stopped {d:.2f} m away', flush=True)

        start = position()
        walk(12.0, 0.0)
        travelled = math.hypot(robot['x'] - start[0], robot['y'] - start[1])
        # At walking pace the robot lags (0.5 m/s^2 acceleration, matching the
        # ODrive velocity ramp) and catches up once the beacon stops.
        if travelled < 6.0:
            raise AssertionError(
                f'Robot moved only {travelled:.2f} m while the beacon walked 12 m')
        d = settle(60.0)
        print(f'PASS B: followed a 12 m walk (robot moved {travelled:.1f} m during it), '
              f'stopped {d:.2f} m away', flush=True)

        walk(0.0, 10.0, stop_after=6.0)  # fixes stop 6 s into the walk
        # Stale after 3 s, then braking from up to 1.2 m/s at 1.5 m/s^2 (~1 s).
        spin(4.0)
        drift = moved_during(5.0)
        if drift > 0.1:
            raise AssertionError(f'Robot still moving {drift:.2f} m/5 s after beacon loss')
        print(f'PASS C: halted after beacon fixes stopped (moved {drift:.2f} m in 5 s)',
              flush=True)

        beacon['on'] = True
        start = position()
        spin(30.0)
        resumed = math.hypot(robot['x'] - start[0], robot['y'] - start[1])
        if resumed < 1.0:
            raise AssertionError(f'Robot moved only {resumed:.2f} m after fixes resumed')
        print(f'PASS D: resumed following when fixes returned (moved {resumed:.1f} m)',
              flush=True)

        home['on'] = True
        command('home')
        d = settle(90.0, 0.8, 2.6, home)
        print(f'PASS E: drove home and stopped {d:.2f} m from the station', flush=True)

        home['y'] += 10.0  # station relocated
        d = settle(60.0, 0.8, 2.6, home)
        print(f'PASS F: followed the relocated station, stopped {d:.2f} m away', flush=True)

        home['y'] += 10.0
        spin(2.0)  # let the robot start moving toward the new spot
        command('stop')
        spin(3.0)
        drift = moved_during(4.0)
        if drift > 0.1:
            raise AssertionError(f'Robot still moving {drift:.2f} m/4 s after stop')
        statuses.clear()
        command('status')
        spin(1.0)
        if not any(s.startswith('mode=idle') for s in statuses):
            raise AssertionError(f'Expected mode=idle status, got {statuses}')
        print(f'PASS G: stopped on command (moved {drift:.2f} m in 4 s); status "{statuses[-1]}"',
              flush=True)

        command('follow')
        d = settle(90.0)
        print(f'PASS H: back to following, stopped {d:.2f} m from the beacon', flush=True)
        print(f'PASS hardware-free beacon following and return home. Log: {args.log}',
              flush=True)
    finally:
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
