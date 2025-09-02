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

#include "gripper_controller/motion_editor.hpp"

namespace SRCIRC2025_HUMANOID_LOCOMOTION
{

MotionEditor::MotionEditor()
{
  joint_to_id_ = {
    {"rotate_torso", 22},
    {"rotate_0",      0},
    {"rotate_1",      1},
    {"rotate_2",      2},
    {"rotate_3",      3},
    {"rotate_5",      5},
  };
}

MotionEditor::MotionEditor(const std::unordered_map<std::string,int>& joint_to_id)
: joint_to_id_(joint_to_id) {}

static bool hasKey(const YAML::Node& n, const char* key)
{
  return n.IsMap() && n[key];
}

void MotionEditor::loadFromFile(const std::string& path)
{
  meta_blobs_.clear();
  frames_.clear();

  YAML::Node root = YAML::LoadFile(path);
  if (!root || !root.IsSequence())
  {
    throw std::runtime_error("MotionEditor: top-level must be a YAML sequence.");
  }

  for (const auto& item : root)
  {
    const bool appearsFrame =
      hasKey(item, "dxl") && hasKey(item, "time") && hasKey(item, "name");

    if (appearsFrame)
    {
      Frame f = parseFrameFromNode(item);
      frames_.push_back(std::move(f));
    }
    else
    {
      // 메타 블롭으로 보존 (round-trip을 위해 문자열로 덤프)
      MetaBlob mb;
      std::stringstream ss;
      ss << item;
      mb.rawYaml = ss.str();
      meta_blobs_.push_back(std::move(mb));
    }
  }
}

void MotionEditor::saveToFile(const std::string& path) const
{
  YAML::Node out = buildYamlFromAll(meta_blobs_, frames_);
  std::ofstream ofs(path);
  if (!ofs) throw std::runtime_error("MotionEditor: cannot open file to write: " + path);
  ofs << out; // yaml-cpp emits nice flow
}

std::vector<std::string> MotionEditor::listStepNames() const
{
  std::vector<std::string> names;
  names.reserve(frames_.size());
  for (const auto& f : frames_) names.push_back(f.name);
  return names;
}

std::optional<Frame> MotionEditor::getFrame(const std::string& step_name) const
{
  int idx = findFrameIndexByName(step_name);
  if (idx < 0) return std::nullopt;
  return frames_[idx];
}

void MotionEditor::editFourArmJoints(const std::string& step_name,
                                     const JointPosMap& joint_positions_rad)
{
  JointPosMap sub;
  auto put = [&](const char* j)
  {
    auto it = joint_positions_rad.find(j);
    if (it != joint_positions_rad.end()) sub[j] = it->second;
  };
  put("rotate_torso");
  put("rotate_0");
  put("rotate_1");
  put("rotate_2");
  put("rotate_3");
  put("rotate_5");

  if (sub.empty()) return; // 바꿀게 없으면 조용히 반환
  editJoints(step_name, sub, /*strict=*/false);
}

void MotionEditor::editJoints(const std::string& step_name,
                              const JointPosMap& joint_positions_rad,
                              bool strict)
{
  int idx = findFrameIndexByName(step_name);
  if (idx < 0) throw std::runtime_error("MotionEditor: step not found: " + step_name);

  Frame& f = frames_[idx];

  // id -> dxl 포인터 맵 만들기 (빠른 갱신)
  std::unordered_map<int, DxlValue*> id2dxl;
  id2dxl.reserve(f.dxl.size());
  for (auto& dv : f.dxl) id2dxl[dv.id] = &dv;

  for (const auto& [jname, qrad] : joint_positions_rad)
  {
    auto it = joint_to_id_.find(jname);
    if (it == joint_to_id_.end())
    {
      if (strict) throw std::runtime_error("Unknown joint name: " + jname);
      else continue; // 모르는 조인트명은 무시
    }
    int id = it->second;

    auto it2 = id2dxl.find(id);
    if (it2 == id2dxl.end())
    {
      // 해당 프레임 dxl에 없으면 새로 추가(일부 파일에 특정 id가 빠져있을 수도 있으므로)
      DxlValue dv; dv.id = id; dv.position = qrad;
      f.dxl.push_back(dv);
      id2dxl[id] = &f.dxl.back();
    }
    else
    {
      it2->second->position = qrad;
    }
  }
}

bool MotionEditor::approxEqual(double a, double b, double eps)
{
  return std::abs(a-b) <= eps * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

int MotionEditor::findFrameIndexByName(const std::string& step_name) const
{
  for (int i=0; i<(int)frames_.size(); ++i)
  {
    if (frames_[i].name == step_name) return i;
  }
  return -1;
}

// ===== YAML 변환 유틸 =====

Frame MotionEditor::parseFrameFromNode(const YAML::Node& n)
{
  Frame f;
  if (n["time"])     f.time = n["time"].as<int>();
  if (n["delay"])    f.delay = n["delay"].as<int>();
  if (n["repeat"])   f.repeat = n["repeat"].as<int>();
  if (n["name"])     f.name = n["name"].as<std::string>();
  if (n["selected"]) f.selected = n["selected"].as<bool>();

  if (!n["dxl"] || !n["dxl"].IsSequence())
  {
    throw std::runtime_error("MotionEditor: frame missing 'dxl' sequence: " + f.name);
  }

  for (const auto& elem : n["dxl"])
  {
    if (!elem.IsMap()) continue;
    DxlValue dv;
    dv.id = elem["id"].as<int>();
    dv.position = elem["position"].as<double>();
    f.dxl.push_back(dv);
  }
  return f;
}

YAML::Node MotionEditor::buildYamlFromAll(const std::vector<MetaBlob>& metas,
                                          const std::vector<Frame>& frames)
{
  YAML::Node out(YAML::NodeType::Sequence);

  // 1) 메타 항목들(로드한 rawYaml 그대로 다시 파싱해서 삽입)
  for (const auto& mb : metas)
  {
    YAML::Node m = YAML::Load(mb.rawYaml);
    out.push_back(m);
  }

  // 2) 프레임들
  for (const auto& f : frames)
  {
    YAML::Node node;
    node["time"]     = f.time;
    node["delay"]    = f.delay;
    node["repeat"]   = f.repeat;
    node["name"]     = f.name;
    node["selected"] = f.selected;

    YAML::Node dxl_node(YAML::NodeType::Sequence);
    for (const auto& dv : f.dxl)
    {
      YAML::Node one;
      one["id"] = dv.id;
      one["position"] = dv.position;
      dxl_node.push_back(one);
    }
    node["dxl"] = dxl_node;

    out.push_back(node);
  }

  return out;
}

}
// namespace SRCIRC2025_HUMANOID_LOCOMOTION
