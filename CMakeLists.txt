cmake_minimum_required(VERSION 3.22)
project(gripper_controller)

# 컴파일 옵션 (경고 표시 등)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(intelligent_humanoid_interfaces REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(TARGET yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp)
elseif(TARGET YAML::yaml-cpp)
  set(YAML_CPP_TARGET YAML::yaml-cpp)
endif()

# include 디렉토리 설정
include_directories(
  include
)

# GRIPPER MAIN NODE
add_executable(gripper_main_node
  src/main_node.cpp
  src/ik_module.cpp
  src/motion_editor.cpp
)
ament_target_dependencies(gripper_main_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
  Eigen3
  eigen3_cmake_module
  visualization_msgs
  pinocchio
  yaml-cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  intelligent_humanoid_interfaces
  dynamixel_rdk_msgs
)

# EE TELEOP NODE
add_executable(ee_teleop_node
  src/teleop_node.cpp
)
ament_target_dependencies(ee_teleop_node
  rclcpp std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

# Pinocchio 링크 (모든 기능 포함)
target_link_libraries(gripper_main_node
  pinocchio::pinocchio
  ${YAML_CPP_TARGET}
)

# intall cfg
install(TARGETS gripper_main_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY robot_model/
  DESTINATION share/${PROJECT_NAME}/robot_model
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(TARGETS ee_teleop_node gripper_main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
