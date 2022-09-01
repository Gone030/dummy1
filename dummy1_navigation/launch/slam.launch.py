import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('dummy1_navigation'), 'config', 'slam.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('dummy1_navigation'), 'rviz', 'slam.rviz']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name = 'sim',
            default_value = 'false',
            description = 'use sim time enable'
        ),

        DeclareLaunchArgument(
            name = 'rviz',
            default_value = 'false',
            description = 'run rviz'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments = {
                'use_sim_time': LaunchConfiguration("sim"),
                'slam_param_name' : slam_config_path
            }.items()
        ),

        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            output = 'screen',
            arguments = ['-d', rviz_config_path],
            condition = IfCondition(LaunchConfiguration("rvizw")),
            parameters = [{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])
