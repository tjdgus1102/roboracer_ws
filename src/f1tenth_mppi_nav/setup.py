import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'f1tenth_mppi_nav'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sangwon',
    maintainer_email='user@example.com',
    description='Nav2 MPPI bringup and cmd_vel-to-Ackermann bridge for F1TENTH simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann = f1tenth_mppi_nav.cmd_vel_to_ackermann:main',
            'goal_pose_relay = f1tenth_mppi_nav.goal_pose_relay:main',
            'record_path = f1tenth_mppi_nav.record_path:main',
            'path_follower = f1tenth_mppi_nav.path_follower:main',
        ],
    },
)
