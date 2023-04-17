# Car-like robot

## 프로젝트 소개
Dummy1은 RC카 프레임을 사용한 Car-like 로봇입니다. Ros2 galactic을 구동하기 위해 Jetson nano를 사용하고 모터 및 센서 구동을 위한 MCU로 Arduino Due를 채택해 사용하고 있습니다.
MCU의 코드는 [dummy1_mcu](https://github.com/Gone030/dummy1_mcu) 에서 확인할 수 있습니다.

Dummy1은 느리지만 멈추지 않는 지속적인 개발이란 모토로, 현재(2023.4 기준)는 SLAM의 최적화를 진행하고 있습니다. 최종 목표로 안정적인 자율주행과 SLAM을 계획하고 있습니다.

(여기에 움짤 추가 예정)

## 개발 기간
* 2022.4 ~ Present

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

## 진행도 및 확인된 문제

(지도 틀어지는 움짤 추가 예정)
간헐적으로 회전 중 지도가 틀어지는 현상이 발생해 수정중에 있습니다.
