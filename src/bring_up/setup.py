from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bring_up'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboflock',
    maintainer_email='adityasc16@gmail.com',
    description='Top-level orchestration: launch files and configs that bring up the full Roboflock autonomy stack',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diff_drive_controller = bring_up.diff_drive_controller:main',
            'e_stop = bring_up.e_stop:main',
            'ps4_teleop = bring_up.ps4_teleop:main',
            'ultrasonic_estop = bring_up.ultrasonic_estop:main',
            'bno085_imu = bring_up.bno085_imu:main',
            'gps_monitor = bring_up.gps_monitor:main',
            'mission_manager = bring_up.mission_manager:main',
            'meshtastic_bridge = bring_up.meshtastic_bridge:main',
            'fake_robot = bring_up.fake_robot:main',
            'fake_beacon = bring_up.fake_beacon:main',
        ],
    },
)
