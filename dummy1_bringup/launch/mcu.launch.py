from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition # run 조건의 캡슐화. boolean 매개변수가 참일 때 실행 해당 노드 실행
from launch.launch_description_sources import PythonLaunchDescriptionSource # 다른 패키지의 python launch file 캡슐화
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution) # run 구성 변수 접근 가능한 대체, 플렛폼 독립적인 방식으로 경로 연결 대체)
from launch_ros.actions import Node # Ros2 node 실행
from launch_ros.substitutions import FindPackageShare # ros 패키지의 경로를 찾아줌 (share 폴더 내)


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

  return LaunchDescription([ # 이하 launch 가능한 시스템
    DeclareLaunchArgument( # 실행 인수 선언
      name= 'use_micro_ros',
      default_value='true',
      description= 'use micro ros'
    ),
    Node(
      package='robot_localization', # 해당 패키지에 포함된 확장 칼만 필터 사용
      executable='ekf_node', # 확장 칼만필터를 사용해 IMU와 Wheel decoder의 오차를 보정 및 fusing
      name='ekf_filter_node',
      output='screen',
      parameters=[ekf_config_path],
      remappings=[("odometry/filtered", "odom")]
    ),
    Node(
      package='micro_ros_agent', # 해당 패키지를 사용해 Arduino Due와 Serial 통신
      executable='micro_ros_agent',
      name='micro_ros_agent',
      output='screen',
      condition=IfCondition(LaunchConfiguration("use_micro_ros")),
      arguments=['serial', '--dev', LaunchConfiguration("serial_port")]
    ),
    Node(
      package='dummy1_bringup', # 해당 노드로 rviz 상에서 joint의 움직임을 확인하기 위해 사용
      executable='jointstatepub',
      name='jointstatepub',
      output='screen'
    ),
    Node(
      package='ydlidar_ros2_driver', # Lidar 를 구동하기 위한 패키지.
      executable='ydlidar_ros2_driver_node',
      name='ydlidar_ros2_driver_node',
      parameters=[lidar_parameter],
      emulate_tty=True,
      output='screen',
      remappings=[("base/scan", "/scan")]
    ),
    IncludeLaunchDescription( # 다른 패키지 include할 때
      PythonLaunchDescriptionSource(description_launch_path)
    )
  ])
