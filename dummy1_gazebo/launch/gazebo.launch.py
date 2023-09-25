import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = True;

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("dummy1_bringup"), "param", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("dummy1_gazebo"), "worlds", "playground.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('dummy1_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value = world_path,
            description = 'Gazebo world'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': world_path
            }.item()
        ),

        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            argument = [
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ]
        ),

        Node(
            package = 'robot_localization',
            executable = 'ekf_node',
            name = 'ekf_filter_node',
            output = 'screen',
            parameters = [
                {'use_sim_time' : use_sim_time},
                ekf_config_path
            ],
            remappings = [("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments = {
                'use_sim_time' : str(use_sim_time),
                'publish_joints' : 'false',
            }.items()
        ),

    ])
