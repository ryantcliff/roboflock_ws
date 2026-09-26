import math

from bring_up.bno085_imu import fill_imu
from sensor_msgs.msg import Imu


def test_quaternion_order_is_converted_to_ros():
    msg = Imu()
    # The Adafruit library returns (i, j, k, real).
    fill_imu(msg, quaternion=(0.1, 0.2, 0.3, 0.9), gyro=(0.0, 0.0, 0.0),
             acceleration=(0.0, 0.0, 9.81))
    assert (msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w) == (0.1, 0.2, 0.3, 0.9)


def test_gyro_and_acceleration_are_copied():
    msg = Imu()
    fill_imu(msg, quaternion=(0.0, 0.0, 0.0, 1.0), gyro=(0.01, -0.02, 0.5),
             acceleration=(0.1, -0.2, 9.8))
    assert (msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z) == (0.01, -0.02, 0.5)
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z) == (0.1, -0.2, 9.8)


def test_covariances_are_set_on_the_diagonal():
    msg = Imu()
    fill_imu(msg, quaternion=(0.0, 0.0, 0.0, 1.0), gyro=(0.0, 0.0, 0.0),
             acceleration=(0.0, 0.0, 9.81))
    for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                msg.linear_acceleration_covariance):
        assert all(cov[i] > 0.0 for i in (0, 4, 8))
        assert all(cov[i] == 0.0 for i in (1, 2, 3, 5, 6, 7))
    # Heading from the magnetometer is looser than roll/pitch from gravity.
    assert msg.orientation_covariance[8] > msg.orientation_covariance[0]
    assert math.isclose(msg.orientation_covariance[8], math.radians(5.0) ** 2)
