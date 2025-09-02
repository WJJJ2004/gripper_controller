# gripper_controller
Intelligent Robot Competition Humanoid Robot Sports

##  outline
휴머노이드 매니퓰레이터의 자율화 제어를 위한 ROS2 패키지입니다.  
Pinocchio 기반 IK 계산, 모션 생성, Teleop/Auto 모드를 지원합니다.

## how to run
```bash
# Teleop Mode 실행
ros2 launch gripper_controller teleop_mode.launch.py

# Auto Mode 실행
ros2 launch gripper_controller auto_mode.launch.py
```

## main dependency 
ROS2 Humble

Pinocchio / hpp-fcl (coal)

Eigen3

yaml-cpp
