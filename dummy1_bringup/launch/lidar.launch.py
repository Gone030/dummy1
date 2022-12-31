#!/usr/bin/evn python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
#from launch.actions import IncludeLaunchDescription
#from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # LifecycleNode


def generate_launch_description():
  lidar_parameter = LaunchConfiguration(
    'lidar_parameter',
    default=os.path.join(
      get_package_share_directory('dummy1_bringup'),
      'param/ydlidar.yaml'
    )
  )
  return LaunchDescription([
    DeclareLaunchArgument(
      'lidar_parameter',
      default_value=lidar_parameter
    ),
    Node(
      package='ydlidar_ros2_driver',
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      parameters=[lidar_parameter],
      emulate_tty=True,
      output='screen',
    ),
    Node(
        package='dummy1_bringup',
        executable='lidar_way_node',
        name='lidar_way_node',
        parameters=[lidar_parameter],
        emulate_tty=True,
        output='screen',
    ),
  ])
