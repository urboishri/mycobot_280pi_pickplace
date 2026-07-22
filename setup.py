from setuptools import setup
import os
from glob import glob

package_name = 'mycobot_280pi_pickplace'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Required: registers this as a ROS 2 package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Pick and place node for myCobot 280 Pi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package.module:function'
            # This creates a command: ros2 run mycobot_280pi_pickplace pick_and_place
            'pick_and_place = mycobot_280pi_pickplace.pick_and_place:main',
        ],
    },
)