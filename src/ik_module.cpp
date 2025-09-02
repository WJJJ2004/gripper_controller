#include "gripper_controller/ik_module.hpp"

#include <stdexcept>
#include <iostream>

namespace SRCIRC2025_HUMANOID_LOCOMOTION
{
IKModule::IKModule(const std::string& urdf_path,
                   const std::string& srdf_path,
                   const std::string& base_link,
                   const std::string& ee_link)
: base_link_name_(base_link), ee_link_name_(ee_link)
{
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, collision_model_);
  collision_data_ = pinocchio::GeometryData(collision_model_);
  collision_model_.addAllCollisionPairs();

  try {
    pinocchio::srdf::loadReferenceConfigurations(model_, srdf_path, false);
  } catch (...) {
    std::cout << "IKModule: No SRDF found, using neutral configuration." << std::endl;
  }

  try {
    pinocchio::srdf::removeCollisionPairs(model_, collision_model_, srdf_path);
  } catch (...) {
    std::cout << "IKModule: No SRDF collision pairs found, using default." << std::endl;
  }

  if (!model_.existFrame(ee_link_name_)) {
    throw std::runtime_error("IKModule: EE frame '" + ee_link_name_ + "' not found in model.");
  }
  ee_frame_id_ = model_.getFrameId(ee_link_name_);

  std::cout << "[IK] #collisionObjects=" << collision_model_.geometryObjects.size()
          << ", #collisionPairs=" << collision_model_.collisionPairs.size() << std::endl;

  lambda_sqred_ = params_.lambda * params_.lambda;  // LM 감쇠 파라미터 제곱

  // 초기 자세 설정
  se3 = currentEEPose(initialConfiguration());

  std::cout << "\033[1;31m" <<"[IK] initial EE pose: " << std::endl;
  std::cout << se3.translation().transpose() << std::endl;
  std::cout << se3.rotation() << "\033[0m" <<std::endl;
}

/* ===========================
 *       Public Methods
 * =========================== */

Eigen::VectorXd IKModule::initialConfiguration() const
{
  if (params_.use_ref_home && model_.referenceConfigurations.count(params_.ref_name)) // SRDF ref가 있으면 + Params table에서 Flag ON >> SRDF ref를 offset pose로 사용
  {
    std::cout << "[IK] Using SRDF reference configuration: " << params_.ref_name << std::endl;
    return model_.referenceConfigurations.at(params_.ref_name);
  }
  else
  {
    std::cout << "\033[1;31m" << "[IK] No SRDF reference configuration found, using neutral pose." << std::endl;
    std::cout << "[IK] No SRDF reference configuration found, using neutral pose." << std::endl;
    std::cout << "[IK] SRDF reference name: " << params_.ref_name << std::endl;
    std::cout << "[IK] SRDF use_ref_home: " << (params_.use_ref_home ? "true" : "false") << "\033[0m" << std::endl;
    return pinocchio::neutral(model_);   // SRDF ref가 없으면 neutral pose(중앙값 0 rad)을 offset pose로 사용
  }
}

// ==================== FK current EE ====================
pinocchio::SE3 IKModule::currentEEPose(const Eigen::VectorXd& q)  // SE3 -> 동차변환행렬 컨테이너*(SE3)로 리턴
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[ee_frame_id_];
}

// ==================== IK Methods ======================
bool IKModule::computeIKPosition(const Eigen::Vector3d& target_pos, Eigen::VectorXd& solution)    // 초기 자세
{
  Eigen::VectorXd backup_solution = solution;
  for (int it = 0; it < params_.max_iter; ++it)
  {
    pinocchio::forwardKinematics(model_, data_, solution);
    pinocchio::updateFramePlacements(model_, data_);

    const Eigen::Vector3d p = data_.oMf[ee_frame_id_].translation();
    const Eigen::Vector3d e = target_pos - p;

    if (e.norm() < params_.pos_tol)
    {
      return true;
    }

    // 6×nv 야코비안 계산 → 상위 3행(위치)만 사용
    Eigen::MatrixXd J6(6, model_.nv);   // model_.nv >> 속도 자유도 수 6은 트위스트(선속도 3 + 각속도 3). nv는 속도 자유도 수.
    pinocchio::computeFrameJacobian(model_, data_, solution, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);
    // std::cout << "Jacobian: " << std::endl;
    // std::cout << J6 << std::endl;
    Eigen::MatrixXd J = J6.topRows(3);  // 3×nv >> 선속도 추출

    // [gripper_main_node-1] Jacobian:
    // [gripper_main_node-1]   0.168966  0.0206286 0.00279396 -0.0396331
    // [gripper_main_node-1] 0.00342757 -0.0138851 0.00308997    0.14187
    // [gripper_main_node-1]          0  0.0971924  0.0144949 -0.0216724
    // [gripper_main_node-1]          0  -0.558389  -0.828887    0.18048
    // [gripper_main_node-1]          0  -0.829579   0.557923    0.19761
    // [gripper_main_node-1]          1          0   0.040836   0.963523

    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + lambda_sqred_ * I3).inverse();
    Eigen::VectorXd dq = params_.alpha * (J_pinv * e);

    // 스텝 클램프
    clampStep(dq);

    // 업데이트 & 조인트 한계 클램프
    solution += dq;
    clampToLimits(solution);
  }
  solution = backup_solution;
  return false; // 미수렴
}

void IKModule::buildTiltOnlyRotation(const Eigen::Matrix3d& R_current,
                                     Eigen::Matrix3d&       R_des_out,
                                     const Eigen::Vector3d& z_target)
{
  const double eps = 1e-12;
  const Eigen::Vector3d z_d = z_target.normalized();

  const Eigen::Vector3d x_ref = R_current.col(0);
  const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - z_d * z_d.transpose();  // z_d 성분 제거
  Eigen::Vector3d x_proj = P * x_ref;

  if (x_proj.squaredNorm() < eps)
  {
    const Eigen::Vector3d y_ref = R_current.col(1);
    x_proj = P * y_ref;

    if (x_proj.squaredNorm() < eps)
    {
      x_proj = Eigen::Vector3d::UnitX();
      x_proj = P * x_proj;
    }
  }

  const Eigen::Vector3d x_d = x_proj.normalized();
  Eigen::Vector3d       y_d = z_d.cross(x_d);
  const double ny = y_d.norm();
  if (ny < eps)
  {
    Eigen::Vector3d x_alt = y_d.cross(z_d);
    if (x_alt.squaredNorm() < eps) x_alt = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d x_dn = (P * x_alt).normalized();
    y_d = z_d.cross(x_dn);
    R_des_out.col(0) = x_dn;
    R_des_out.col(1) = y_d.normalized();
    R_des_out.col(2) = z_d;
  }
  else
  {
    R_des_out.col(0) = x_d;
    R_des_out.col(1) = y_d / ny;
    R_des_out.col(2) = z_d;
  }
}

pinocchio::SE3 IKModule::makeTiltOnlyTarget(const Eigen::Matrix3d& R_current,
                                                const Eigen::Vector3d& target_p,
                                                const Eigen::Vector3d& z_target)
{
  Eigen::Matrix3d R_des;
  buildTiltOnlyRotation(R_current, R_des, z_target);
  return pinocchio::SE3(R_des, target_p);
}

bool IKModule::computeIKPrioritySE3(const pinocchio::SE3& target_se3,
                                    Eigen::VectorXd&      solution,
                                    bool                  tilt_only /* = true for z-align */)
{
  Eigen::VectorXd backup = solution;

  // 튜닝 파라미터
  const double alpha  = params_.alpha;                  // 스텝
  const double lambda = params_.lambda;                 // DLS 댐핑
  const double posTol = params_.pos_tol;
  const double oriTol = params_.ori_tol;

  for (int it = 0; it < params_.max_iter; ++it)
  {
    pinocchio::forwardKinematics(model_, data_, solution);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3& M_ee = data_.oMf[ee_frame_id_];

    pinocchio::SE3 M_des;
    if (tilt_only)
    {
      M_des = makeTiltOnlyTarget(M_ee.rotation(), target_se3.translation(), Eigen::Vector3d(0,0,-1));
    }
    else
    {
      M_des = target_se3; // 순수 pose 추적
    }

    const Eigen::Vector3d p    = M_ee.translation();
    const Eigen::Vector3d e_p  = M_des.translation() - p; // [월드 기준 위치오차]

    const Eigen::Matrix3d R_err = M_ee.rotation().transpose() * M_des.rotation();
    const Eigen::Vector3d e_R   = pinocchio::log3(R_err);

    Eigen::Matrix<double,6,1> e6;
    e6.template head<3>() = e_p;
    e6.template tail<3>() = e_R;

    Eigen::MatrixXd J6(6, model_.nv);
    pinocchio::computeFrameJacobian(model_, data_, solution, ee_frame_id_,pinocchio::LOCAL_WORLD_ALIGNED, J6);

    Eigen::Matrix<double,6,6> S = Eigen::Matrix<double,6,6>::Identity();
    S(3,3) = params_.roll_weight;  // roll
    S(4,4) = params_.pitch_weight;  // pitch
    S(5,5) = params_.yaw_weight; // 마지막 행/열: 회전 z축(=yaw)

    // 가중치 적용
    const Eigen::MatrixXd J = S * J6;    // (6×nv)
    const Eigen::Matrix<double,6,1> y = S * e6; // (6×1)

    // === DLS: dq = J^T (J J^T + λ^2 I)^{-1} y ===
    Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(J.rows(), J.rows());
    Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + lambda*lambda * I6).inverse();
    Eigen::VectorXd dq = alpha * (J_pinv * y);

    // 스텝 클램프(있다면)
    clampStep(dq);

    // 업데이트 & 조인트 한계
    solution += dq;
    clampToLimits(solution);

    // 수렴 판정
    //  - 위치: e_p
    //  - 회전: tilt-only면 yaw 성분이 0이므로 roll/pitch 성분만 남은 ‖S*e6_tail‖ 사용
    const double pos_err = e_p.norm();
    const double ori_err = (S * e6).tail<3>().norm();

    if (pos_err < posTol && ori_err < oriTol) {
      std::cout << "[IK] iteration: " << it << std::endl;
      return true;
    }
  }
  solution = backup; // 미수렴 → 롤백
  return false;
}

bool IKModule::checkSelfCollision(const Eigen::VectorXd& q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);

  for (const auto& cp : collision_model_.collisionPairs)
  {
    const auto& go1 = collision_model_.geometryObjects[cp.first];
    const auto& go2 = collision_model_.geometryObjects[cp.second];

    const auto& M1 = collision_data_.oMg[cp.first];
    const auto& M2 = collision_data_.oMg[cp.second];

    Transform3 T1, T2;
    T1.rotation() = M1.rotation();
    T1.translation() = M1.translation();
    T2.rotation() = M2.rotation();
    T2.translation() = M2.translation();

    fcl_ns::CollisionRequest req;
    fcl_ns::CollisionResult  res;

    fcl_ns::collide(go1.geometry.get(), T1, go2.geometry.get(), T2, req, res);

    if (res.isCollision())
    {
      std::cout << "\033[1;31m" <<"[IK]: Self-collision detected between: "
                << go1.name << " and " << go2.name << "\033[0m"  <<std::endl;
      return true;
    }
  }
  return false;
}

/* ===========================
 *       Private Helpers
 * =========================== */

void IKModule::clampStep(Eigen::VectorXd& dq) const
{
  // if (params_.step_max <= 0.0) return;
  for (int i = 0; i < dq.size(); ++i)
  {
    if (dq[i] >  params_.step_max) dq[i] =  params_.step_max;
    if (dq[i] < -params_.step_max) dq[i] = -params_.step_max;
  }
}

// bool IKModule::hasJointLimits() const
// {
//   return (model_.lowerPositionLimit.size() == model_.nq) && (model_.upperPositionLimit.size() == model_.nq);
// }

void IKModule::clampToLimits(Eigen::VectorXd& q) const
{
  // if (!hasJointLimits()) return;
  for (int i = 0; i < q.size(); ++i)
  {
    const double lo = model_.lowerPositionLimit[i];
    const double hi = model_.upperPositionLimit[i];
    if (lo < hi)
    {
      if (q[i] < lo) q[i] = lo;
      if (q[i] > hi) q[i] = hi;
    }
  }
}
}


// namespace SRCIRC2025_HUMANOID_LOCOMOTION