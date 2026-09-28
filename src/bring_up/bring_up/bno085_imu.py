"""
BNO085 IMU on I2C -> /imu/data (sensor_msgs/Imu) in base_link.

Orientation is the BNO085 rotation vector, which fuses the magnetometer, so
yaw is an absolute heading. It is published as the sensor reports it; the
offset between the sensor's zero heading and ENU east is corrected with
navsat_transform's yaw_offset and magnetic_declination_radians once it has
been measured outdoors. The board must be mounted flat with its X arrow
pointing forward and Y to the left.

Needs the Adafruit libraries (PyPI):
  pip install --user adafruit-circuitpython-bno08x adafruit-extended-bus

Unlike mpu9250driver, nothing is published while the sensor can't be read.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Diagonal variances. Roll/pitch come from gravity and are tight; heading
# depends on the magnetometer and the robot's own magnetic fields.
ROLL_PITCH_VARIANCE = math.radians(2.0) ** 2
YAW_VARIANCE = math.radians(5.0) ** 2
GYRO_VARIANCE = math.radians(0.5) ** 2
ACCEL_VARIANCE = 0.05 ** 2


def fill_imu(msg, quaternion, gyro, acceleration):
    """
    Fill an Imu message from Adafruit BNO08x readings.

    quaternion is (i, j, k, real), gyro is rad/s and acceleration is m/s^2
    including gravity (the EKF removes it).
    """
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quaternion
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
    (msg.linear_acceleration.x, msg.linear_acceleration.y,
     msg.linear_acceleration.z) = acceleration
    msg.orientation_covariance = [
        ROLL_PITCH_VARIANCE, 0.0, 0.0,
        0.0, ROLL_PITCH_VARIANCE, 0.0,
        0.0, 0.0, YAW_VARIANCE]
    msg.angular_velocity_covariance = [
        GYRO_VARIANCE, 0.0, 0.0, 0.0, GYRO_VARIANCE, 0.0, 0.0, 0.0, GYRO_VARIANCE]
    msg.linear_acceleration_covariance = [
        ACCEL_VARIANCE, 0.0, 0.0, 0.0, ACCEL_VARIANCE, 0.0, 0.0, 0.0, ACCEL_VARIANCE]
    return msg


class Bno085Imu(Node):

    def __init__(self):
        super().__init__('bno085_imu')
        self.i2c_bus = self.declare_parameter('i2c_bus', 1).value
        self.address = self.declare_parameter('address', 0x4A).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        rate = self.declare_parameter('rate', 50.0).value
        self.report_interval_us = int(1e6 / rate)

        self.bno = None
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_timer(1.0 / rate, self._poll)
        self.create_timer(2.0, self._ensure_connected)
        self._ensure_connected()

    def _ensure_connected(self):
        if self.bno is not None:
            return
        try:
            import adafruit_bno08x
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_extended_bus import ExtendedI2C

            bno = BNO08X_I2C(ExtendedI2C(self.i2c_bus), address=self.address)
            for feature in (adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR,
                            adafruit_bno08x.BNO_REPORT_GYROSCOPE,
                            adafruit_bno08x.BNO_REPORT_ACCELEROMETER):
                bno.enable_feature(feature, self.report_interval_us)
        except Exception as error:  # missing library, unwired sensor or bus error
            self.get_logger().warn(
                f'BNO085 not available on /dev/i2c-{self.i2c_bus} at '
                f'0x{self.address:02X}: {error}', throttle_duration_sec=30.0)
            return
        self.bno = bno
        self.get_logger().info(
            f'BNO085 connected on /dev/i2c-{self.i2c_bus} at 0x{self.address:02X}')

    def _poll(self):
        if self.bno is None:
            return
        try:
            quaternion = self.bno.quaternion
            gyro = self.bno.gyro
            acceleration = self.bno.acceleration
        except Exception as error:
            self.get_logger().warn(f'BNO085 read failed, reconnecting: {error}')
            self.bno = None
            return
        if quaternion is None or gyro is None or acceleration is None:
            return
        msg = fill_imu(Imu(), quaternion, gyro, acceleration)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bno085Imu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
