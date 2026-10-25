#include "control_module/rl_controller.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <minizip/unzip.h>

namespace xyber_x1_infer::rl_control_module {

RLController::RLController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
}

void RLController::Init(const YAML::Node& cfg_node) {
  // 初始化 joint_names_
  joint_names_.clear();
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  publish_joint_names_ = cfg_node["publish_joint_list"]
      ? cfg_node["publish_joint_list"].as<std::vector<std::string>>()
      : joint_names_;
  publish_action_indices_.clear();
  for (const auto& name : publish_joint_names_) {
    auto iter = std::find(joint_names_.begin(), joint_names_.end(), name);
    if (iter == joint_names_.end()) {
      throw std::runtime_error("Unknown publish joint: " + name);
    }
    publish_action_indices_.push_back(static_cast<int>(std::distance(joint_names_.begin(), iter)));
  }
  // printf("joint_names_: ");
  // for (const auto &name : joint_names_) {
  //   printf("%s ", name.c_str());
  // }
  // printf("\n");
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);

  // 初始化 joint_conf_
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());
  // std::cout << "init_state: " << joint_conf_.init_state.transpose() << std::endl;
  // std::cout << "stiffness: " << joint_conf_.stiffness.transpose() << std::endl;
  // std::cout << "damping: " << joint_conf_.damping.transpose() << std::endl;

  // 其他 RL 参数
  // clang-format off
  walk_step_conf_.action_scale  = cfg_node["walk_step_conf"]["action_scale"].as<double>();
  walk_step_conf_.action_scales = cfg_node["walk_step_conf"]["action_scales"]
      ? cfg_node["walk_step_conf"]["action_scales"].as<std::vector<double>>()
      : std::vector<double>();
  walk_step_conf_.decimation    = cfg_node["walk_step_conf"]["decimation"].as<int32_t>();
  walk_step_conf_.cycle_time    = cfg_node["walk_step_conf"]["cycle_time"].as<double>();
  walk_step_conf_.sw_mode       = cfg_node["walk_step_conf"]["sw_mode"].as<bool>();
  walk_step_conf_.cmd_threshold = cfg_node["walk_step_conf"]["cmd_threshold"].as<double>();
  obs_scales_.lin_vel           = cfg_node["obs_scales"]["lin_vel"].as<double>();
  obs_scales_.ang_vel           = cfg_node["obs_scales"]["ang_vel"].as<double>();
  obs_scales_.dof_pos           = cfg_node["obs_scales"]["dof_pos"].as<double>();
  obs_scales_.dof_vel           = cfg_node["obs_scales"]["dof_vel"].as<double>();
  obs_scales_.quat              = cfg_node["obs_scales"]["quat"].as<double>();
  onnx_conf_.policy_file        = cfg_node["onnx_conf"]["policy_file"].as<std::string>();
  onnx_conf_.actions_size       = cfg_node["onnx_conf"]["actions_size"].as<int32_t>();
  onnx_conf_.observations_size  = cfg_node["onnx_conf"]["observations_size"].as<int32_t>();
  onnx_conf_.num_hist           = cfg_node["onnx_conf"]["num_hist"].as<int32_t>();
  onnx_conf_.observations_clip  = cfg_node["onnx_conf"]["observations_clip"].as<double>();
  onnx_conf_.actions_clip       = cfg_node["onnx_conf"]["actions_clip"].as<double>();
  if (!walk_step_conf_.action_scales.empty() &&
      walk_step_conf_.action_scales.size() != static_cast<size_t>(onnx_conf_.actions_size)) {
    throw std::runtime_error("action_scales size must equal actions_size");
  }
  reference_joint_pos_ = cfg_node["reference_joint_pos"]
      ? cfg_node["reference_joint_pos"].as<std::vector<float>>()
      : std::vector<float>(onnx_conf_.actions_size, 0.0F);
  reference_joint_vel_ = cfg_node["reference_joint_vel"]
      ? cfg_node["reference_joint_vel"].as<std::vector<float>>()
      : std::vector<float>(onnx_conf_.actions_size, 0.0F);
  reference_anchor_ori_ = cfg_node["reference_anchor_ori"]
      ? cfg_node["reference_anchor_ori"].as<std::vector<float>>()
      : std::vector<float>{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F};

  // Only the x1_Leo tracking policy uses the 29-DoF/154-observation
  // reference format. Keep the existing 12/14-DoF walking policies valid.
  if (onnx_conf_.actions_size == 29) {
    if (reference_joint_pos_.size() != 29 || reference_joint_vel_.size() != 29 ||
        reference_anchor_ori_.size() != 6) {
      throw std::runtime_error(
          "x1_Leo reference dimensions must be 29, 29, and 6; got " +
          std::to_string(reference_joint_pos_.size()) + ", " +
          std::to_string(reference_joint_vel_.size()) + ", " +
          std::to_string(reference_anchor_ori_.size()));
    }
  } else {
    if (reference_joint_pos_.size() != static_cast<size_t>(onnx_conf_.actions_size) ||
        reference_joint_vel_.size() != static_cast<size_t>(onnx_conf_.actions_size)) {
      reference_joint_pos_.assign(onnx_conf_.actions_size, 0.0F);
      reference_joint_vel_.assign(onnx_conf_.actions_size, 0.0F);
    }
  }
  lpf_conf_.wc                  = cfg_node["lpf_conf"]["wc"].as<double>();
  lpf_conf_.ts                  = cfg_node["lpf_conf"]["ts"].as<double>();
  auto paralle_list = cfg_node["lpf_conf"]["paralle_list"].as<std::vector<std::string>>();
  lpf_conf_.paralle_list        = std::set<std::string>(paralle_list.begin(), paralle_list.end());
  if (cfg_node["motion_file"]) {
    LoadMotionReference(cfg_node["motion_file"].as<std::string>());
  }
  LoadModel();
  // clang-format on
  actions_.resize(onnx_conf_.actions_size);
  observations_.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  last_actions_.resize(onnx_conf_.actions_size);
  last_actions_.setZero();
  propri_history_buffer_.resize(onnx_conf_.observations_size * onnx_conf_.num_hist);
  loop_count_ = 0;
  low_pass_filters_.clear();
  for (size_t i = 0; i < onnx_conf_.actions_size; ++i) {
    low_pass_filters_.emplace_back(100, 0.001);
  }
  propri_.joint_pos.resize(onnx_conf_.actions_size);
  propri_.joint_vel.resize(onnx_conf_.actions_size);
}

void RLController::RestartController() {
  is_first_frame_ = true;
  loop_count_ = 0;
  motion_finished_.store(false);
  has_motion_anchor_quat_ = false;
  in_transition_ = (onnx_conf_.actions_size == 29 && motion_frames_ > 0);
  transition_cycles_ = static_cast<int64_t>(std::max(1.0, transition_time_s_ * 1000.0));
  if (in_transition_) {
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    transition_start_pos_.resize(29);
    for (int i = 0; i < 29; ++i) transition_start_pos_(i) = joint_state_data_.position[i];
  }
}

void RLController::Update() {
  UpdateStateEstimation();
  if (onnx_conf_.actions_size == 29 && use_motion_reference_) {
    // First move from the currently held stand pose to motion frame 0 using
    // position targets only. The actor was trained with the robot already at
    // frame 0, so running it before this transition creates a large impulse.
    if (in_transition_) {
      if (loop_count_ == 0) {
        for (int i = 0; i < 29; ++i) {
          if (lpf_conf_.paralle_list.find(joint_names_[i]) == lpf_conf_.paralle_list.end()) {
            low_pass_filters_[i].init(propri_.joint_pos[i]);
          } else {
            low_pass_filters_[i].init(0);
          }
        }
      }
      const double alpha = std::min(1.0, static_cast<double>(loop_count_) /
          static_cast<double>(transition_cycles_));
      for (int i = 0; i < 29; ++i) {
        // Linear handover requested for Leo: at alpha=0 the measured stand
        // pose has weight 100%; at alpha=1 motion frame 0 has weight 100%.
        // The reference pose is absolute joint position, so interpolate in
        // joint-position space before converting to the policy action space.
        const double target = transition_start_pos_(i) * (1.0 - alpha) +
            motion_joint_pos_[i] * alpha;
        actions_[i] = static_cast<float>((target - joint_conf_.init_state(i)) /
            std::max(1e-6, walk_step_conf_.action_scales.empty()
                ? walk_step_conf_.action_scale : walk_step_conf_.action_scales[i]));
      }
      loop_count_++;
      if (loop_count_ >= transition_cycles_) {
        in_transition_ = false;
        loop_count_ = 0;
        last_actions_.setZero();
        is_first_frame_ = true;
      }
      return;
    }
    UpdateMotionReference();
  }
  // compute observation & actions
  if (loop_count_ % walk_step_conf_.decimation == 0) {
    ComputeObservation();
    ComputeActions();
  }
  loop_count_++;
}

my_ros2_proto::msg::JointCommand RLController::GetJointCmdData() {
  // During the Leo handover this controller publishes the interpolated
  // position target. It is listed after pd_leo_hold, so its command wins for
  // all 29 policy joints.
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = publish_joint_names_;
  joint_cmd.position.resize(publish_joint_names_.size());
  joint_cmd.velocity.resize(publish_joint_names_.size());
  joint_cmd.effort.resize(publish_joint_names_.size());
  joint_cmd.damping.resize(publish_joint_names_.size());
  joint_cmd.stiffness.resize(publish_joint_names_.size());

  // Keep all 29 actions for the next observation, including the three waist
  // actions that are not published because PD owns those joints.
  for (int ii = 0; ii < onnx_conf_.actions_size; ++ii) {
    last_actions_(ii, 0) = actions_[ii];
  }

  // Body joints are intentionally excluded from the published command and
  // remain controlled by pd_zero/pd_stand.
  for (size_t jj = 0; jj < publish_action_indices_.size(); ++jj) {
    const int ii = publish_action_indices_[jj];
    const double action_scale = walk_step_conf_.action_scales.empty()
        ? walk_step_conf_.action_scale : walk_step_conf_.action_scales[ii];
    scalar_t pos_des = actions_[ii] * action_scale + joint_conf_.init_state(ii);
    double stiffness = joint_conf_.stiffness(ii);
    double damping = joint_conf_.damping(ii);

    if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
      low_pass_filters_[ii].input(pos_des);
      double pos_des_lp = low_pass_filters_[ii].output();
      joint_cmd.position[jj] = pos_des_lp;
      joint_cmd.velocity[jj] = 0.0;
      joint_cmd.effort[jj] = 0.0;
      joint_cmd.stiffness[jj] = stiffness;
      joint_cmd.damping[jj] = damping;
    } else {
      double tau_des = stiffness * (pos_des - propri_.joint_pos[ii]) + damping * (0.0 - propri_.joint_vel[ii]);
      low_pass_filters_[ii].input(tau_des);
      double tau_des_lp = low_pass_filters_[ii].output();
      joint_cmd.position[jj] = 0.0;
      joint_cmd.velocity[jj] = 0.0;
      joint_cmd.effort[jj] = tau_des_lp;
      joint_cmd.stiffness[jj] = 0.0;
      joint_cmd.damping[jj] = 0.0;
    }
  }
  return joint_cmd;
}

namespace {

std::vector<uint8_t> ReadNpyFromNpz(const std::string& path, const std::string& name) {
  unzFile zip = unzOpen(path.c_str());
  if (!zip) throw std::runtime_error("Cannot open motion npz: " + path);
  if (unzLocateFile(zip, name.c_str(), 0) != UNZ_OK) {
    unzClose(zip);
    throw std::runtime_error("Missing array in motion npz: " + name);
  }
  unz_file_info info{};
  char filename[256]{};
  unzGetCurrentFileInfo(zip, &info, filename, sizeof(filename), nullptr, 0, nullptr, 0);
  if (unzOpenCurrentFile(zip) != UNZ_OK) { unzClose(zip); throw std::runtime_error("Cannot read npy: " + name); }
  std::vector<uint8_t> bytes(info.uncompressed_size);
  int got = unzReadCurrentFile(zip, bytes.data(), static_cast<unsigned>(bytes.size()));
  unzCloseCurrentFile(zip);
  unzClose(zip);
  if (got != static_cast<int>(bytes.size())) throw std::runtime_error("Incomplete npy: " + name);
  const bool valid_magic = bytes.size() >= 10 &&
      bytes[0] == 0x93 && bytes[1] == 'N' && bytes[2] == 'U' &&
      bytes[3] == 'M' && bytes[4] == 'P' && bytes[5] == 'Y';
  if (!valid_magic) {
    throw std::runtime_error("Invalid npy header: " + name);
  }
  const uint8_t major = bytes[6];
  size_t header_len = major == 1 ? bytes[8] | (bytes[9] << 8)
                                 : *reinterpret_cast<const uint32_t*>(bytes.data() + 8);
  size_t header_size = major == 1 ? 10 : 12;
  if (header_size + header_len > bytes.size()) throw std::runtime_error("Invalid npy size: " + name);
  std::string header(reinterpret_cast<const char*>(bytes.data() + header_size), header_len);
  if (header.find("'descr': '<f4'") == std::string::npos &&
      header.find("'descr': '<f8'") == std::string::npos)
    throw std::runtime_error("Only little-endian float npy is supported: " + name);
  return std::vector<uint8_t>(bytes.begin() + header_size + header_len, bytes.end());
}

template <typename T>
std::vector<float> NpyFloat32(const std::vector<uint8_t>& bytes) {
  std::vector<float> out(bytes.size() / sizeof(T));
  for (size_t i = 0; i < out.size(); ++i) out[i] = static_cast<float>(reinterpret_cast<const T*>(bytes.data())[i]);
  return out;
}

}  // namespace

void RLController::LoadMotionReference(const std::string& path) {
  if (onnx_conf_.actions_size != 29) return;
  auto pos_bytes = ReadNpyFromNpz(path, "joint_pos.npy");
  auto vel_bytes = ReadNpyFromNpz(path, "joint_vel.npy");
  auto quat_bytes = ReadNpyFromNpz(path, "body_quat_w.npy");
  auto pos = NpyFloat32<float>(pos_bytes);
  auto vel = NpyFloat32<float>(vel_bytes);
  auto quat = NpyFloat32<float>(quat_bytes);
  if (pos.size() % 29 != 0 || vel.size() != pos.size() || quat.size() != (pos.size() / 29) * 30 * 4)
    throw std::runtime_error("Invalid X1 motion array dimensions in " + path);
  motion_frames_ = static_cast<int>(pos.size() / 29);
  motion_joint_pos_ = std::move(pos);
  motion_joint_vel_ = std::move(vel);
  motion_anchor_quat_ = std::move(quat);
  auto fps_bytes = ReadNpyFromNpz(path, "fps.npy");
  // Leo_v3.npz stores fps as little-endian float64, while pose arrays are
  // float32. Decode it with the correct element width (otherwise an f8
  // scalar is interpreted as two unrelated f4 values).
  auto fps = NpyFloat32<double>(fps_bytes);
  if (fps.size() != 1 || !(fps[0] > 0.0) || !std::isfinite(fps[0]))
    throw std::runtime_error("Invalid fps array in " + path);
  motion_fps_ = fps[0];
  use_motion_reference_.store(true);
  std::cout << "Loaded motion reference [" << path << "], frames="
            << motion_frames_ << ", fps=" << motion_fps_ << std::endl;
}

void RLController::UpdateMotionReference() {
  if (motion_frames_ <= 0 || motion_fps_ <= 0.0) return;
  // ControlModule runs at 1000 Hz in the X1 configuration.
  const int frame = std::min(motion_frames_ - 1,
      static_cast<int>(loop_count_ * motion_fps_ / 1000.0));
  // Do not switch on the first update of the final frame. This gives the
  // controller one complete final-frame control interval and avoids a
  // one-frame discontinuity when returning to stand.
  if (loop_count_ >= static_cast<int64_t>(std::ceil(
          (motion_frames_ - 1) * 1000.0 / motion_fps_)) + 1) {
    motion_finished_.store(true);
  }
  reference_joint_pos_.assign(motion_joint_pos_.begin() + frame * 29,
                              motion_joint_pos_.begin() + (frame + 1) * 29);
  reference_joint_vel_.assign(motion_joint_vel_.begin() + frame * 29,
                              motion_joint_vel_.begin() + (frame + 1) * 29);
  const float* q = motion_anchor_quat_.data() + (frame * 30) * 4;
  Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]);
  if (!std::isfinite(quat.w()) || !std::isfinite(quat.x()) ||
      !std::isfinite(quat.y()) || !std::isfinite(quat.z()) ||
      quat.squaredNorm() < 1e-8F) {
    throw std::runtime_error("Invalid anchor quaternion in motion reference");
  }
  motion_anchor_quat_current_ = quaternion_t(
      static_cast<scalar_t>(quat.w()), static_cast<scalar_t>(quat.x()),
      static_cast<scalar_t>(quat.y()), static_cast<scalar_t>(quat.z())).normalized();
  has_motion_anchor_quat_ = true;
}

void RLController::LoadModel() {
  // create env
  std::shared_ptr<Ort::Env> onnxEnvPrt(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetInterOpNumThreads(1);
  session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt, onnx_conf_.policy_file.c_str(), sessionOptions);

  // get input and output info
  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();

  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t ii = 0; ii < session_ptr_->GetInputCount(); ++ii) {
    char* tempstring = new char[strlen(session_ptr_->GetInputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, session_ptr_->GetInputNameAllocated(ii, allocator).get());
    input_names_.push_back(tempstring);
    input_shapes_.push_back(session_ptr_->GetInputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }

  for (size_t ii = 0; ii < session_ptr_->GetOutputCount(); ++ii) {
    char* tempstring = new char[strlen(session_ptr_->GetOutputNameAllocated(ii, allocator).get()) + 1];
    strcpy(tempstring, session_ptr_->GetOutputNameAllocated(ii, allocator).get());
    output_names_.push_back(tempstring);
    output_shapes_.push_back(session_ptr_->GetOutputTypeInfo(ii).GetTensorTypeAndShapeInfo().GetShape());
  }
}

void RLController::UpdateStateEstimation() {
  {
    std::shared_lock<std::shared_mutex> lock(joint_state_mutex_);
    for (size_t ii = 0; ii < onnx_conf_.actions_size; ++ii) {
      std::string joint_name = joint_names_[ii];
      propri_.joint_pos(ii) = joint_state_data_.position[ii];
      propri_.joint_vel(ii) = joint_state_data_.velocity[ii];
    }
  }

  {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    propri_.base_ang_vel(0) = imu_data_.angular_velocity.x;
    propri_.base_ang_vel(1) = imu_data_.angular_velocity.y;
    propri_.base_ang_vel(2) = imu_data_.angular_velocity.z;

    vector3_t gravity_vector(0, 0, -1);
    quaternion_t quat;
    quat.x() = imu_data_.orientation.x;
    quat.y() = imu_data_.orientation.y;
    quat.z() = imu_data_.orientation.z;
    quat.w() = imu_data_.orientation.w;
    matrix_t inverse_rot = GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse();
    propri_.projected_gravity = inverse_rot * gravity_vector;
    propri_.base_euler_xyz = QuatToXyz(quat);
  }
}

void RLController::ComputeObservation() {
  vector_t propri_obs(onnx_conf_.observations_size);
  propri_obs.setZero();
  if (onnx_conf_.actions_size == 29 && onnx_conf_.observations_size == 154) {
    // unitree_rl_mjlab X1 tracking actor (has_state_estimation=False):
    // ref q(29), ref dq(29), ref anchor ori(6), base gyro(3), q(29), dq(29),
    // previous action(29).
    for (int i = 0; i < 29; ++i) {
      propri_obs(i) = reference_joint_pos_[i];
      propri_obs(29 + i) = reference_joint_vel_[i];
      propri_obs(2 * 29 + 6 + 3 + i) =
          (propri_.joint_pos(i) - joint_conf_.init_state(i)) * obs_scales_.dof_pos;
      propri_obs(2 * 29 + 6 + 3 + 29 + i) =
          propri_.joint_vel(i) * obs_scales_.dof_vel;
      propri_obs(2 * 29 + 6 + 3 + 2 * 29 + i) = last_actions_(i);
    }
    // motion_anchor_ori_b = R_body_world^T * R_motion_world, first two
    // columns flattened row-major, exactly as mjlab's matrix_from_quat(...)
    // [..., :2].reshape(...). Using the reference orientation directly here
    // makes the policy see world-frame values and is a common fall-down cause.
    quaternion_t current_anchor = quaternion_t::Identity();
    {
      std::shared_lock<std::shared_mutex> lock(imu_mutex_);
      current_anchor.w() = imu_data_.orientation.w;
      current_anchor.x() = imu_data_.orientation.x;
      current_anchor.y() = imu_data_.orientation.y;
      current_anchor.z() = imu_data_.orientation.z;
    }
    quaternion_t relative = current_anchor.conjugate() *
        (has_motion_anchor_quat_ ? motion_anchor_quat_current_ : quaternion_t::Identity());
    matrix_t ori = relative.normalized().toRotationMatrix();
    propri_obs(58) = ori(0, 0);
    propri_obs(59) = ori(0, 1);
    propri_obs(60) = ori(1, 0);
    propri_obs(61) = ori(1, 1);
    propri_obs(62) = ori(2, 0);
    propri_obs(63) = ori(2, 1);
    for (int i = 0; i < 3; ++i) {
      propri_obs(64 + i) = propri_.base_ang_vel(i) * obs_scales_.ang_vel;
    }
  } else {
    // Original walking policies: phase, command, relative q/dq and action.
    std::shared_lock<std::shared_mutex> lock(joy_mutex_);
    double phase = duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    if (walk_step_conf_.sw_mode) {
      double cmd_norm = std::sqrt(Square(joy_data_.linear.x) + Square(joy_data_.linear.y) + Square(joy_data_.angular.z));
      if (cmd_norm <= walk_step_conf_.cmd_threshold) phase = 0;
    }
    phase /= walk_step_conf_.cycle_time;
    vector_t native_obs(5 + onnx_conf_.actions_size * 3 + 6);
    native_obs << sin(2 * M_PI * phase), cos(2 * M_PI * phase),
                  joy_data_.linear.x * obs_scales_.lin_vel,
                  joy_data_.linear.y * obs_scales_.lin_vel,
                  joy_data_.angular.z,
                  (propri_.joint_pos - joint_conf_.init_state) * obs_scales_.dof_pos,
                  propri_.joint_vel * obs_scales_.dof_vel, last_actions_,
                  propri_.base_ang_vel * obs_scales_.ang_vel,
                  propri_.base_euler_xyz * obs_scales_.quat;
    const int count = std::min<int>(native_obs.size(), propri_obs.size());
    propri_obs.head(count) = native_obs.head(count);
  }

  if (is_first_frame_) {
    for (size_t ii = 0; ii < joint_names_.size(); ++ii) {
      if (lpf_conf_.paralle_list.find(joint_names_[ii]) == lpf_conf_.paralle_list.end()) {
        // serial
        low_pass_filters_[ii].init(propri_.joint_pos[ii]);
      } else {
        // parallel
        low_pass_filters_[ii].init(0);
      }
    }

    // The Leo tracking actor already has a complete 154-D observation and
    // num_hist=1.  The old walking-policy offsets (5 + 2*actions) must not
    // be used here: they overwrite the reference orientation, gyro, and
    // current joint position fields.  last_actions_ is initialized to zero,
    // so only the Leo action block needs no further special handling.
    if (!(onnx_conf_.actions_size == 29 && onnx_conf_.observations_size == 154)) {
      for (int ii = 5 + onnx_conf_.actions_size * 2;
           ii < 5 + onnx_conf_.actions_size * 3; ++ii) {
        propri_obs(ii, 0) = 0.0;
      }
    }

    for (int ii = 0; ii < onnx_conf_.num_hist; ++ii) {
      propri_history_buffer_.segment(ii * onnx_conf_.observations_size, onnx_conf_.observations_size) = propri_obs.cast<float>();
    }
    is_first_frame_ = false;
  }

  propri_history_buffer_.head(propri_history_buffer_.size() - onnx_conf_.observations_size) = propri_history_buffer_.tail(propri_history_buffer_.size() - onnx_conf_.observations_size);
  propri_history_buffer_.tail(onnx_conf_.observations_size) = propri_obs.cast<float>();

  for (int ii = 0; ii < (onnx_conf_.observations_size * onnx_conf_.num_hist); ++ii) {
    observations_[ii] = static_cast<float>(propri_history_buffer_[ii]);
  }
  // limit observations range
  scalar_t obs_min = -onnx_conf_.observations_clip;
  scalar_t obs_max = onnx_conf_.observations_clip;
  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                 [obs_min, obs_max](scalar_t x) { 
                   return std::max(obs_min, std::min(obs_max, x));
                 });
}

void RLController::ComputeActions() {
  // create input tensor object
  std::vector<Ort::Value> input_tensor;
  input_tensor.push_back(Ort::Value::CreateTensor<float>(memory_info_, observations_.data(), observations_.size(), input_shapes_[0].data(),input_shapes_[0].size()));

  std::vector<Ort::Value> output_values = session_ptr_->Run(Ort::RunOptions{}, input_names_.data(), input_tensor.data(), 1, output_names_.data(), 1);

  for (int i = 0; i < onnx_conf_.actions_size; ++i) {
    actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
  }
  // limit action range
  scalar_t action_min = -onnx_conf_.actions_clip;
  scalar_t action_max = onnx_conf_.actions_clip;
  std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                 [action_min, action_max](scalar_t x) {
                   return std::max(action_min, std::min(action_max, x));
                 });
}

} // namespace xyber_x1_infer::