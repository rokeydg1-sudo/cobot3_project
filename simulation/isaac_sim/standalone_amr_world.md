3. 기존 파일 수정 내용

3.1 standalone_amr_world.py

기존 역할

기존에는 Isaac Sim 내부의 Nova Carter를 직접 제어하는 역할이 중심이었다.

초기 단계에서는 TCP 5005/5006을 이용해 목표와 Pose를 주고받았고,
이후 ROS2 /amr/goal을 받아 Isaac 내부 WheelBasePoseController가 목표 좌표까지 직접 이동하는 구조를 사용했다.

현재 변경 내용

Nav2를 사용할 수 있도록 Isaac Sim을 물리 시뮬레이션 + 센서/Actuator 인터페이스 계층으로 확장했다.

추가된 주요 ROS2 인터페이스:

Isaac Sim → ROS2
/amr/odom
/front_2d_lidar/scan
/clock
/tf
/tf_static

ROS2 → Isaac Sim
/cmd_vel

추가된 기능

1. /cmd_vel Subscriber

Nav2에서 계산한 geometry_msgs/msg/Twist를 받아
Nova Carter의 Differential Controller에 전달한다.

Nav2
 ↓
/cmd_vel
 ↓
Isaac Sim ROS2 Bridge
 ↓
DifferentialController
 ↓
Nova Carter Wheel

2. Front 2D LiDAR ROS2 Publish

Nova Carter Asset에 원래 포함된 Front RPLidar를 재사용한다.

RPLidar
 ↓
ROS2RtxLidarHelper
 ↓
/front_2d_lidar/scan

Frame ID:

front_2d_lidar

3. TF 추가

Nav2와 AMCL이 사용할 수 있도록 TF Tree를 추가했다.

odom
 ↓
base_link
 ↓
front_2d_lidar

odom -> base_link: Dynamic TF

base_link -> front_2d_lidar: Static TF

4. /clock 추가

Isaac Simulation Time을 ROS2 /clock으로 발행하여
Nav2 전체가 use_sim_time:=true로 같은 시간을 사용하도록 변경했다.

5. TCP 제거

기존 TCP 통신:

5005 Goal
5006 Pose

은 제거했다.

현재 파일에는 이전 /amr/goal Pose Controller 경로가 호환/테스트용으로 일부 남아 있지만,
Nav2 운용 시 실제 주행 제어는 /cmd_vel이 담당한다.
