#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
// #include <mutex>
#include <shared_mutex>

namespace xyber_x1_infer::rl_control_module {

struct State {
  State(const std::string &state_name, const YAML::Node &node) {
    state_name_ = state_name;
    trigger_topic_ = node["trigger_topic"].as<std::string>();
    pre_states_ = node["pre_states"].as<std::vector<std::string>>();
    controllers_ = node["controllers"].as<std::vector<std::string>>();
  }

  std::string state_name_;
  std::string trigger_topic_;
  std::vector<std::string> pre_states_;
  std::vector<std::string> controllers_;
};

class StateMachine {
 public:
  StateMachine() = default;
  ~StateMachine() = default;

  void Init(const YAML::Node &node) {
    trigger_state_map_.clear();
    for (auto iter = node.begin(); iter != node.end(); iter++) {
      State state(iter->first.as<std::string>(), iter->second);
      trigger_state_map_[iter->second["trigger_topic"].as<std::string>()].push_back(state);
    }

    // 默认直接进入站立状态。idle 的关节刚度/阻尼为 0，作为初始状态
    // 会让仿真机器人在用户来得及触发状态切换前倒地。
    auto initial_state = node["stand"] ? node["stand"] : node.begin()->second;
    current_state_name_ = node["stand"] ? "stand" : node.begin()->first.as<std::string>();
    controllers_ = trigger_state_map_[initial_state["trigger_topic"].as<std::string>()][0].controllers_;
  }

  bool OnEvent(const std::string &trigger_topic) {
    for (const auto& to_trans_state : trigger_state_map_[trigger_topic]) {
      // A held keyboard/joystick button is published repeatedly. Do not
      // treat an event targeting the current state as a new transition;
      // otherwise leo -> leo repeatedly restarts the one-shot motion.
      if (to_trans_state.state_name_ == current_state_name_) {
        return false;
      }
      auto iter = std::find(to_trans_state.pre_states_.begin(), to_trans_state.pre_states_.end(), current_state_name_);

      if (iter != to_trans_state.pre_states_.end()) {
        current_state_name_ = to_trans_state.state_name_;
        std::unique_lock<std::shared_mutex> unique_lock(controllers_mutex_);
        controllers_ = to_trans_state.controllers_;
        return true;
      }
    }
    return false;
  }

  std::string GetCurrentState() {
    return current_state_name_;
  }

  std::vector<std::string> GetCurrentControllerNames() {
    std::shared_lock<std::shared_mutex> shared_lock(controllers_mutex_);
    return controllers_;
  }

  // Used by one-shot motion controllers: return to stand after the motion.
  bool ReturnToStandIf(const std::string& controller_name, bool finished) {
    if (!finished || current_state_name_ != "leo") return false;
    const auto& states = trigger_state_map_["/stand_mode"];
    for (const auto& state : states) {
      if (state.state_name_ != "stand") continue;
      if (std::find(state.pre_states_.begin(), state.pre_states_.end(), current_state_name_) == state.pre_states_.end()) continue;
      std::unique_lock<std::shared_mutex> lock(controllers_mutex_);
      current_state_name_ = "stand";
      controllers_ = state.controllers_;
      return true;
    }
    return false;
  }

 private:
  std::string current_state_name_;
  std::map<std::string, std::string> state_trigger_map_;
  std::map<std::string, std::vector<State>> trigger_state_map_;
  mutable std::shared_mutex controllers_mutex_;
  std::vector<std::string> controllers_;
};

}  // namespace xyber_x1_infer::rl_control_module
