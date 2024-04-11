# Car-like robot

<img src = "https://github.com/Gone030/dummy1_mcu/assets/89852937/cfba7c47-eaa2-4da2-b8ce-a86a2c5e9a2e" width="300" height="400"> <img src = "https://github.com/Gone030/dummy1_mcu/assets/89852937/f6fe0611-ae4d-4477-82a8-a49e15eef6b1" width="300" height="400">

## 프로젝트 소개
Dummy1은 RC카 프레임을 사용한 Car-like 로봇입니다. Ros2 galactic을 구동하기 위해 Jetson nano를 사용하고 모터 및 센서 구동을 위한 MCU로 Arduino Due를 채택해 사용하고 있습니다.
MCU의 코드 및 자세한 설명은 [dummy1_mcu](https://github.com/Gone030/dummy1_mcu) 에서 확인할 수 있습니다.

Dummy1은 느리지만 멈추지 않는 지속적인 개발이란 모토로, Slam_toolbox 패키지를 이용한 slam을 성공했으며, 향후 Ros2에 대한 이해도를 높힌 뒤 Navigation 까지 시도해 볼 예정입니다.

(3배속 재생입니다.)

https://github.com/Gone030/dummy1/assets/89852937/ab614d2d-06ac-457e-a768-a9f669c35ab4





## 개발 기간
* 2022.4 ~ 2023.6

## 개발 환경
* Robot Computer : `Ubuntu 20.04` , `Ros2 Galactic`
* Host PC : `Ubuntu 22.04` , `Ros2 Humble`
* MCU : `Arduino Due`
* `PlatformIO`

## 부품 정보
* IMU : MPU9250
* Laser Sensor : [YDLidar](https://www.ydlidar.com/lidars.html) X4
* Encoder : [E30S4-3000-3-V-5](https://kr.misumi-ec.com/vona2/detail/221005279659/?HissuCode=E30S4-3000-3-V-5)

## 동작 방식

![동작방식](https://user-images.githubusercontent.com/89852937/232405742-3338bc55-86fc-495d-8be8-c995b19ee979.png)

![MCU동작방식](https://user-images.githubusercontent.com/89852937/232420930-1fde0742-22cb-4bbb-84f6-3927214fe83c.png)

Micro-ros 패키지를 사용해 Serial 통신으로 MCU에서 측정하는 IMU Data(/imu/data)와 엔코더로 측정한 Data를 기반으로 계산된 RPM 및 Pose(odom/unfiltered)를 전송합니다. 이후 robot_localization 패키지에서 제공하는 확장 칼만 필터를 활용해 센서융합으로 현재 자세 및 위치 추정의 정확도를 높혔습니다.

## Robot 동작

### 1. Dummy1_bringup 실행

    ros2 launch dummy1_bringup mcu.launch.py

Lidar와 Micro-ros Agent, Robot_localization 패키지가 동작하며 MCU와 Robot Computer간의 시리얼 통신이 연결되고 MCU로부터 수신된 Odometry와 IMU 데이터를 ekf_node가 수신해 센서융합합니다.
 추가적인 매개변수 :
 * __use_micro_ros__ : False로 설정하면 Micro ros Agent 패키지의 실행을 비활성화 합니다.

예시 :

    ros2 launch dummy1_bringup mcu.launch.py use_micro_ros:=false

### 2. 로봇 제어
다음의 명령어로 키보드를 통해 동작할 수 있습니다.

    ros2 launch dummy1_bringup teleop

최대 속도와 최대 조향 각도가 제한되어 있습니다.

### 3. SLAM
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

    ros2 launch dummy1_navigation slam.launch.py

`rviz`를 통해 작성되는 지도를 확인할 수 있습니다.

