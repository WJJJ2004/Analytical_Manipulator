# Analytical_Manipulator

> **휴머노이드 매니퓰레이션** 프로젝트의 Corridor Race 4 – Loco‑Manipulation 임무 수행을 위한 ROS 2 제어 노드  
> Ubuntu 22.04 + ROS 2 Humble 환경에서 작동합니다.

---

## 📁 목차

1. [개요](#개요)  
2. [설치 및 실행 방법](#설치-및-실행-방법)  
3. [파라미터 튜닝](#파라미터-튜닝)  
4. [개발 환경](#개발-환경)
5. [의존 패키지](#의존-패키지)  

---

## 개요

이 패키지는 Corridor Race 4: Loco‑Manipulation 미션에서 휴머노이드 로봇의 팔 제어를 위해 개발되었습니다.  
- YAML 기반 링크/오프셋/튜닝 파라미터 로딩  
- 캠별 스위치 상대 좌표 → EE 접근 경로 자동 생성 및 IK 계산
- 관련 매트랩 코드 레포 링크: https://github.com/WJJJ2004/Analytical_IK_solver
- ROS2 URDF 패키지 : https://github.com/WJJJ2004/torsoHumanoid2025

<img width="1920" height="1074" alt="image" src="https://github.com/user-attachments/assets/eed3e490-2914-4d96-aa13-3113127fed14" />

---

## 설치 및 실행 방법

### 1. 워크스페이스 생성
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 리포지토리 클론
```bash
git clone https://github.com/WJJJ2004/Analytical_Manipulator.git
```
### 3. 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select arm_ctrl
```
### 4. 실행 예제
```bash
source install/setup.bash
ros2 launch arm_ctrl arm_ctrl.launch.py
```

## 파라미터 튜닝
config/params.yaml에서 주요 튜닝 항목:

튜닝 기준 및 결과는 Issue에 이미지로 정리되어 있습니다 (issue #1 참조). https://github.com/WJJJ2004/Analytical_Manipulator/issues/1

## 개발 환경
OS: Ubuntu 22.04

ROS 2: Humble

언어: C++17, Python 3.10.12

## 의존 패키지:

rclcpp, sensor_msgs, geometry_msgs, tf2_ros

eigen3, yaml-cpp, ament_cmake, ament_python
