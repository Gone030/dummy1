import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  mcu_parameter = LaunchConfiguration(
  'mcu',
  default=os.path.join(
      get_package_share_directory('dummy1_bringup'),
      'param',
      'mcu.yaml'
    )
  )
  return LaunchDescription([
    DeclareLaunchArgument(
      'mcu',
      default_value=mcu_parameter
    ),

    Node(
      package='dummy1_bringup',
      executable='mcu_node',
      name='mcu_node',
      parameters=[mcu_parameter],
      emulate_tty=True,
      output='screen',
    )
])
