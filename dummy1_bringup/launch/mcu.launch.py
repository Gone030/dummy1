from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  description_launch_path = PathJoinSubstitution(
    [FindPackageShare('dummy1_description'), 'launch', 'description.launch.py']
  )
  lidar_parameter = PathJoinSubstitution(
    [FindPackageShare('dummy1_bringup'), 'param', 'ydlidar.yaml']
  )
  ekf_config_path = PathJoinSubstitution(
    [FindPackageShare('dummy1_bringup'), 'param', 'ekf.yaml']
  )
  # mpu_config_path = PathJoinSubstitution(
  #   [FindPackageShare('dummy1_bringup'), 'param', 'mpu9250.yaml']
  # )
  return LaunchDescription([
    DeclareLaunchArgument(
      name='serial_port',
      default_value='/dev/ttyACM0',
      description='Serial port'
    ),
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[
        ekf_config_path
      ],
      remappings=[("odometry/filtered", "odom")]
    ),
    Node(
      package='micro_ros_agent',
      executable='micro_ros_agent',
      name='micro_ros_agent',
      output='screen',
      arguments=['serial', '--dev', LaunchConfiguration("serial_port")]
    ),
    Node(
      package='ydlidar_ros2_driver',
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      parameters=[lidar_parameter],
      emulate_tty=True,
      output='screen',
      remappings=[("base/scan", "/scan")]
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(description_launch_path)
    )
  ])
