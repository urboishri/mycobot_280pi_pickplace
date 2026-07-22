import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, LogInfo)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('mycobot_280pi_gazebo')
    moveit_pkg = get_package_share_directory("mycobot_280pi_moveit_config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    moveit_launch = TimerAction(
        period = 150.0
        actions = [
            LogInfo(msg='Starting MoveIt move_grp...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        moveit_pkg, 'launch', 'move_group.launch.py'
                    )
                )
            )
        ]
    )

    pickplace_node = TimerAction(
        period = 170.0,
        actions = [
            LogInfo(msg = 'Starting Pick and Place Sequence...'),
            Node(
                package = 'mycobot_280pi_pickplace',
                executable = 'pick_and_place',
                name = 'pick_and_place_node',
                output = 'screen',
                parameters = [{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='Launching full pick and place system...'),
        LogInfo(msg = 'Gazebo starting now, MoveIt in 150s. Task in 170s'),
        gazebo_launch,
        moveit_launch,
        pickplace_node,
    ])