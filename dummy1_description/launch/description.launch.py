import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('dummy1_description'), 'urdf', 'base.urdf.xacro']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('dummy1_description'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),
        DeclareLaunchArgument(
            name='pablish_joints',
            default_value='true',
            description='launch joint_states_publisher'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='run rviz'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='use sim time'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro', LaunchConfiguration('urdf')])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
