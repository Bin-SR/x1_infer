#pragma once
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <memory>
#include <set>
#include <atomic>

#include "control_module/controller_base.h"
#include "control_module/rotation_tools.h"

namespace xyber_x1_infer::rl_control_module {

class RLController : public ControllerBase {
 public:
  RLController(const bool use_sim_handles);
  ~RLController() = default;

  void Init(const YAML::Node &cfg_node) override;
  void RestartController() override;

  void Update() override;
  my_ros2_proto::msg::JointCommand GetJointCmdData() override;

 private:
  void LoadModel();
  void UpdateStateEstimation();
  void ComputeObservation();
  void ComputeActions();
  void LoadMotionReference(const std::string& path);
  void UpdateMotionReference();

 private:
  struct WalkStepConf {
    double action_scale;
    std::vector<double> action_scales;
    int decimation;
    double cycle_time;
    bool sw_mode;
    double cmd_threshold;
  } walk_step_conf_;

  struct ObsScales {
    double lin_vel;
    double ang_vel;
    double dof_pos;
    double dof_vel;
    double quat;
  } obs_scales_;

  struct OnnxConf {
    std::string policy_file;
    int actions_size;
    int observations_size;
    int num_hist;
    double observations_clip;
    double actions_clip;
  } onnx_conf_;

  struct LPFConf {
    double wc;
    double ts;
    std::set<std::string> paralle_list;
  } lpf_conf_;

  // onnx
  std::unique_ptr<Ort::Session> session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  // Policy joint order follows the 29-DoF X1 training order. The optional
  // publish list excludes waist joints that remain under PD control.
  std::vector<std::string> publish_joint_names_;
  std::vector<int> publish_action_indices_;
  std::vector<float> reference_joint_pos_;
  std::vector<float> reference_joint_vel_;
  std::vector<float> reference_anchor_ori_;
  std::vector<float> motion_joint_pos_;
  std::vector<float> motion_joint_vel_;
  std::vector<float> motion_anchor_quat_;
  int motion_frames_{0};
  double motion_fps_{0.0};
  std::atomic_bool use_motion_reference_{false};

  // compute in algorithm
  std::vector<float> actions_;
  std::vector<float> observations_;
  vector_t last_actions_;
  // vector_t propri_history_buffer_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> propri_history_buffer_;
  struct Proprioception {
    vector_t joint_pos;
    vector_t joint_vel;
    vector3_t base_ang_vel;
    vector3_t base_euler_xyz;
    vector3_t projected_gravity;
  } propri_;

  // other
  int64_t loop_count_;
  std::vector<digital_lp_filter<double>> low_pass_filters_;
  std::atomic_bool is_first_frame_{true};
};

}  // namespace xyber_x1_infer::rl_control_module
