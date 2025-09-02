// #ifndef GRIPPER_CONTROLLER_IK_MODULE_HPP
// #define GRIPPER_CONTROLLER_IK_MODULE_HPP
#pragma once

#include <iostream>
#include <string>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <pinocchio/spatial/se3.hpp>
#if __has_include(<coal/collision.h>)
  #include <coal/collision.h>
  namespace fcl_ns = ::coal;
  using Transform3 = fcl_ns::Transform3s;
#else
  #include <hpp/fcl/collision.h>
  namespace fcl_ns = ::hpp::fcl;
  using Transform3 = fcl_ns::Transform3s;
#endif

namespace SRCIRC2025_HUMANOID_LOCOMOTION {
class IKModule {
public:
  struct Params {
    int    max_iter      = 200;               // IK 최대 반복
    double pos_tol       = 1e-4;              // 위치 오차 허용치 (m)
    double se3_tol       = 1e-4;              // 6D 오차 노름 허용치
    double ori_tol       = 0.035;             // 회전 오차 허용치 (rad) ~2deg
    double alpha         = 0.2;               // 스텝 스케일 (0~1)
    double lambda        = 1e-3;              // LM 감쇠 파라미터
    double step_max      = 0.05;              // 각속 스텝 클램프 (rad/step)
    bool   use_ref_home  = true;              // SRDF 레퍼런스(home 등) 초기자세 사용
    std::string ref_name = "gravity_offset";  // SRDF 내부 키
    double yaw_weight    = 0.0;
    double roll_weight   = 0.0;
    double pitch_weight  = 0.0;
  };

  pinocchio::SE3 se3;
public:
  IKModule(const std::string& urdf_path,
           const std::string& srdf_path,
           const std::string& base_link = "base_link",
           const std::string& ee_link   = "ee_link_1");

  // DOF 조회
  int getDOF() const { return static_cast<int>(model_.nv); }

  // 파라미터 설정/조회
  void setParams(const Params& p) { params_ = p; lambda_sqred_ = p.lambda * p.lambda; }
  const Params& params() const { return params_; }

  bool computeIKPosition(const Eigen::Vector3d& target_pos, Eigen::VectorXd& solution);
  bool computeIKSE3(const pinocchio::SE3& target_se3, Eigen::VectorXd& solution);
  bool computeIKPrioritySE3(const pinocchio::SE3& target_se3, Eigen::VectorXd& solution, bool tilt_only); // 자기충돌 여부
  bool checkSelfCollision(const Eigen::VectorXd& q);

  // 현재 EE 프레임의 SE3 (world 기준)
  pinocchio::SE3 currentEEPose(const Eigen::VectorXd& q);

  // 초기 자세 얻기 (SRDF ref가 있으면 그걸, 없으면 neutral)
  Eigen::VectorXd initialConfiguration() const;

  // 프레임 ID 접근
  pinocchio::FrameIndex eeFrameId() const { return ee_frame_id_; }

  void buildTiltOnlyRotation(const Eigen::Matrix3d& R_current,
                             Eigen::Matrix3d& R_des_out,
                             const Eigen::Vector3d& z_target = Eigen::Vector3d(0,0,-1));
  pinocchio::SE3 makeTiltOnlyTarget(const Eigen::Matrix3d& R_current,
                                    const Eigen::Vector3d& target_p,
                                    const Eigen::Vector3d& z_target = Eigen::Vector3d(0,0,-1));
private:
  // 내부 헬퍼
  void clampToLimits(Eigen::VectorXd& q) const;
  void clampStep(Eigen::VectorXd& dq) const;
  // bool hasJointLimits() const;

private:
  // 모델/데이터
  pinocchio::Model model_;
  pinocchio::Data  data_;

  // 충돌 모델
  pinocchio::GeometryModel collision_model_;
  pinocchio::GeometryData  collision_data_;

  // 베이스/EE 명칭
  std::string base_link_name_;
  std::string ee_link_name_;

  // EE 프레임 ID (캐시)
  pinocchio::FrameIndex ee_frame_id_ = (pinocchio::FrameIndex)(-1);

  // 파라미터
  Params params_;
  double lambda_sqred_ = 0.0;  // 감쇠 파라미터 제곱 (LM-damped pseudo-inverse 용)
};

}
// namespace SRCIRC2025_HUMANOID_LOCOMOTION
