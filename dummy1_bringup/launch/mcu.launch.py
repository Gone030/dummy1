from platform import node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

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
  return LaunchDescription([
    # DeclareLaunchArgument(
    #   name='serial_port',
    #   default_value='/dev/ttyACM0',
    #   description='Serial port'
    # ),
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
      package='mpu9250driver',
      executable='mpu9250driver_node',
      name='mpu9250driver_node',
      output='screen',
      remappings=[("imu", "imu/data_raw")]
    ),
    Node(
      pakage='imu_filter_madgwick',
      executable='imu_filter',
      name='imu_filter',
      output='screen',
    ),
    # Node(
    #   package='dummy1_bringup',
    #   executable='mcu_node',
    #   name='mcu_node',
    #   output='screen',
    # ),
    Node(
      package='ydlidar_ros2_driver',
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      parameters=[lidar_parameter],
      emulate_tty=True,
      output='screen',
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(description_launch_path)
    )
  ])
