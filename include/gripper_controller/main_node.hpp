#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// custom interfaces messages
#include "intelligent_humanoid_interfaces/msg/vision2_master_msg.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"

#include "gripper_controller/motion_editor.hpp"
#include "gripper_controller/ik_module.hpp"

namespace SRCIRC2025_HUMANOID_LOCOMOTION
{
enum class ControlMode { Auto, Teleop };
class GripperMainNode : public rclcpp::Node
{
public:
  explicit GripperMainNode(const std::string& urdf_path, const std::string& srdf_path, const std::string& AUTO_CTRL);

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ee_current_;
  Eigen::VectorXd q_current_;

private:
  std::string share_dir = ament_index_cpp::get_package_share_directory("gripper_controller");
  std::string yaml_path = share_dir + "/motion/grip.yaml";

  double waypoint_d = 0.035;
  ControlMode mode_;

  rclcpp::TimerBase::SharedPtr timer_pub;

  rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr sub_master_request_;
  rclcpp::Subscription<intelligent_humanoid_interfaces::msg::Vision2MasterMsg>::SharedPtr vision_sub_;
  rclcpp::Subscription<dynamixel_rdk_msgs::msg::DynamixelMsgs>::SharedPtr tilt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_load_param_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ctrl_flg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_collision_;

  // TF 관련
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool transformPointToBase(const geometry_msgs::msg::PointStamped& in_eye, geometry_msgs::msg::PointStamped& out_base);

  bool use_latest_tf_ = true;
  float tf_timeout_sec_ = 0.1;
  int tf_throttle_ms_ = 2000;
  bool debug_tf_ = false;

  std::string base_frame_;
  std::string camera_frame_;

  float tilt_angle = 0.0;     // radian
  bool tilt_only = true;      // computeIKPropertySE3 ctrl Flag

  void onMasterRequest(const geometry_msgs::msg::Point32::SharedPtr msg);
  void onVision(const intelligent_humanoid_interfaces::msg::Vision2MasterMsg::SharedPtr msg);
  void target_cmd_callback(const geometry_msgs::msg::Point::SharedPtr msg);
  void getParamsFromRos();

  std::shared_ptr<MotionEditor> motion_editor_;
  std::shared_ptr<IKModule> ik_;
  IKModule::Params params_;
};

} // namespace SRCIRC2025_HUMANOID_LOCOMOTION
