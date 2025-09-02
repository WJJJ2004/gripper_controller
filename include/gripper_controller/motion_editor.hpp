/*
 * Motion Editor
 * @file motion_editor.cpp
 * Provides utilities to load, edit, and save robot motion frames from YAML files.
 * Each frame contains joint states (id, position) and metadata (time, delay, name).
 * Unknown YAML entries are preserved as raw text for round-trip consistency.
 *
 * Key features:
 * - Load/save motion sequences from YAML
 * - List and retrieve frames by name
 * - Edit joint positions by joint name or ID
 * - Preserve unknown metadata (MetaBlob)
 */

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>
#include <fstream>

namespace SRCIRC2025_HUMANOID_LOCOMOTION
{
struct DxlValue {
  int id{};
  double position{}; // rad
};

struct Frame
{
  // Frame fields that appear in YAML
  int time{0};
  int delay{0};
  int repeat{0};
  std::string name;
  bool selected{false};
  std::vector<DxlValue> dxl; // id-position pairs
};

/// 편집 시 어떤 관절 이름을 얼마로 바꿀지 전달하기 위한 타입
using JointPosMap = std::unordered_map<std::string, double>; // joint_name -> rad

/// YAML 모션 파일 편집기
class MotionEditor {
public:
  /// 조인트명 -> 모터ID 매핑을 기본값으로 초기화
  MotionEditor();

  /// 외부에서 사용자 정의 매핑 주입도 가능
  explicit MotionEditor(const std::unordered_map<std::string,int>& joint_to_id);

  /// 파일 로드 (기존 데이터 모두 교체)
  void loadFromFile(const std::string& path);

  /// 파일 저장 (메타/프레임 순서는 로드된 구조를 최대한 유지)
  void saveToFile(const std::string& path) const;

  /// 모든 프레임 이름 목록
  std::vector<std::string> listStepNames() const;

  /// 이름으로 프레임 찾기 (없으면 std::nullopt)
  std::optional<Frame> getFrame(const std::string& step_name) const;

  /// 이름으로 프레임을 찾아 4개 관절만 업데이트 (없으면 예외)
  void editFourArmJoints(const std::string& step_name,
                         const JointPosMap& joint_positions_rad);

  void editJoints(const std::string& step_name,
                  const JointPosMap& joint_positions_rad,
                  bool strict = false);

  /// 매핑 접근 (읽기)
  const std::unordered_map<std::string,int>& jointToId() const { return joint_to_id_; }

  /// 매핑 수정 (주의: 이미 로드된 프레임에는 영향 없음, 편집 시에만 적용)
  void setJointToId(const std::unordered_map<std::string,int>& m) { joint_to_id_ = m; }

private:
  struct MetaBlob {
    std::string rawYaml;
  };

  std::vector<MetaBlob> meta_blobs_;
  std::vector<Frame> frames_;

  std::unordered_map<std::string,int> joint_to_id_;

  // 내부 유틸
  static bool approxEqual(double a, double b, double eps=1e-12);
  int findFrameIndexByName(const std::string& step_name) const;

  // YAML <-> 내부 변환
  static Frame parseFrame(const std::string& rawDump);
  static Frame parseFrameFromNode(const struct YAML::Node& node);
  static std::string dumpMetaNode(const struct YAML::Node& node);
  static struct YAML::Node buildYamlFromAll(const std::vector<MetaBlob>& metas,
                                            const std::vector<Frame>& frames);
};

} // namespace SRCIRC2025_HUMANOID_LOCOMOTION
