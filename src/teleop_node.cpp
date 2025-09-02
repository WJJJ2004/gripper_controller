/*
  * EE Teleoperation Node
  * @file teleop_node.cpp
  * This node allows teleoperation of an end-effector (EE) position using keyboard inputs.
  * It publishes the target position and a marker for visualization in RViz.
  *
  * Controls:
  * - W/S: Move along X axis (+/-)
  * - A/D: Move along Y axis (+/-)
  * - E/Q: Move along Z axis (+/-)
  * - R: Reset to current position
  * - L: Request to load parameters from the main node
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <termios.h>
#include <unistd.h>
#include <cstdio>

namespace FOR_DEBUGGING
{
class EETeleopNode : public rclcpp::Node {
public:
  EETeleopNode() : Node("ee_teleop_node")
  {
    declare_parameter<double>("step", 0.01);
    declare_parameter<double>("rate_hz", 30.0);
    get_parameter("step", step_);
    get_parameter("rate_hz", rate_hz_);

    pub_target_ = create_publisher<geometry_msgs::msg::Point>("/ee_target_position", 10);
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>("/ee_target_marker", 10);
    pub_load_param_cmd_ = create_publisher<std_msgs::msg::Bool>("/telop_cmd_load", 10);

    sub_current_ = create_subscription<geometry_msgs::msg::Point>
    (
      "/ee_current_position", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr m)
      {
        current_ = *m;
        if (!has_current_)
        {
          // 첫 수신 시 target을 현재 위치로 동기화
          target_ = current_;
          has_current_ = true;
        }
      }
    );
    // 초기 위치 (원하는 값으로 조정)
    target_.x = 0.3; target_.y = 0.0; target_.z = 0.3;

    // 터미널을 non-canonical, no-echo 모드로
    enableNonBlockingStdin();

    timer_ = create_wall_timer(std::chrono::milliseconds((int)(1000.0 / rate_hz_)),
                               std::bind(&EETeleopNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "EETeleopNode started. Keys: W/S(+/-X), A/D(+/-Y), E/Q(+/-Z), R(reset), L(Load param requst) Ctrl+C to quit.");
  }

  ~EETeleopNode() override {
    disableNonBlockingStdin();
  }
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_current_;
  geometry_msgs::msg::Point current_;   // 최신 EE 현재 위치
  bool has_current_{false};
private:
  void onTimer()
  {
  // 현재 위치를 아직 못 받았으면 대기 (marker만 유지)
  if (!has_current_) {
    publishMarker(target_);
    return;
  }

  geometry_msgs::msg::Point base = current_; // 항상 "현재"에서 시작
  int c = readCharNonBlocking();
  if (c != -1) {
    switch (c) {
      case 'w': case 'W': base.x += step_; break;
      case 's': case 'S': base.x -= step_; break;
      case 'a': case 'A': base.y += step_; break;
      case 'd': case 'D': base.y -= step_; break;
      case 'e': case 'E': base.z += step_; break;
      case 'q': case 'Q': base.z -= step_; break;
      case 'r': case 'R': base = current_; break; // 리셋을 "현재"로
      case 'l': case 'L': loadParameterCmd(); break; // 메인 노드 파라미터 로드 명령
      default: break;
    }
    target_ = base; // 이번 프레임의 목표를 갱신
  }

  pub_target_->publish(target_);
  publishMarker(target_);
  }

  void loadParameterCmd()
  {
    std_msgs::msg::Bool cmd_msg;
    cmd_msg.data = true; // 로드 명령
    pub_load_param_cmd_->publish(cmd_msg);
  }


  void publishMarker(const geometry_msgs::msg::Point& p)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";   // RViz Fixed Frame과 일치시켜야 보임
    m.header.stamp = now();
    m.ns = "ee_target";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = p.z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;  // 지름
    m.scale.y = 0.03;
    m.scale.z = 0.03;
    m.color.r = 1.0f;
    m.color.g = 0.1f;
    m.color.b = 0.1f;
    m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration(0,0);
    pub_marker_->publish(m);
  }

  /* ===== 터미널 입력 유틸 ===== */
  void enableNonBlockingStdin()
  {
    tcgetattr(STDIN_FILENO, &orig_termios_);
    termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO); // canonical 모드/echo 끔
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  void disableNonBlockingStdin()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
  }

  int readCharNonBlocking()
  {
    unsigned char c;
    int n = read(STDIN_FILENO, &c, 1);
    if (n == 1) return c;
    return -1;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_load_param_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point target_;

  double step_{0.01};
  double rate_hz_{30.0};

  termios orig_termios_{};
};

} // namespace FOR_DEBUGGING


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FOR_DEBUGGING::EETeleopNode>());
  rclcpp::shutdown();
  return 0;
}