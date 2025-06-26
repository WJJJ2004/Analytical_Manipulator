### 1. `arm_ctrl`
- **기능**: IK 계산, trajectory 생성, 조인트 각도 퍼블리시 및 Gazebo 시각화.
- **메시지**: 
  - `ArmCtrlCmd.msg`: EE 목표 좌표 명령
  - `ArmCtrlFlag.msg`: 마스터 시스템 준비 플래그
  - `ArmJointAngle.msg`: 조인트 각도 명령
- **노드**:
  - `main_node`: 메인 제어 흐름 및 퍼블리셔 포함
- **설정파일**:
  - `arm_params.yaml`: IK 파라미터
  - `controller.yaml`: Gazebo용 controller 설정

Ananlytical IK 매트랩 코드: https://github.com/WJJJ2004/Analytical_IK_solver

Parameter 튜닝 예시 : https://github.com/WJJJ2004/Analytical_Manipulator/issues/1#issue-3177506858
