// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "joy_stick_module/joy_stick_module.h"

#include <yaml-cpp/yaml.h>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

// #include "Empty.aimrt_rpc.srv.h"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

namespace xyber_x1_infer::joy_stick_module {

bool JoyStickModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  joy_ = std::make_shared<Joy>();

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["freq"].as<uint32_t>();

      // prepare executor
      executor_ = core_.GetExecutorManager().GetExecutor("joy_stick_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'joy_stick_pub_thread'.");

      if (cfg_node["float_pubs"]) {
        for (const auto& pub : cfg_node["float_pubs"]) {
          FloatPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<std_msgs::msg::Float32>(publisher.pub);
          float_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["twist_pubs"]) {
        for (const auto& pub : cfg_node["twist_pubs"]) {
          TwistPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.axis = pub["axis"].as<std::map<std::string, uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub);
          if (pub["velocity_limit_lb"] && pub["velocity_limit_ub"]) {
            publisher.pub_limiter = core_.GetChannelHandle().GetPublisher(publisher.topic_name + "_limiter");  // topic: /cmd_vel + _limiter
            aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub_limiter);

            auto lb = pub["velocity_limit_lb"].as<std::vector<double>>();
            auto ub = pub["velocity_limit_ub"].as<std::vector<double>>();
            array_t lb_array = Eigen::Map<array_t>(lb.data(), lb.size());
            array_t ub_array = Eigen::Map<array_t>(ub.data(), ub.size());
            limiter_ = std::make_shared<JoyVelLimiter>(pub["axis"].size(), 1.0 / freq_, lb_array, ub_array);
          }
          twist_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["rpc_clients"]) {
        for (const auto& rpc : cfg_node["rpc_clients"]) {
          ServiceClient clent;
          clent.service_name = rpc["service_name"].as<std::string>();
          clent.buttons = rpc["buttons"].as<std::vector<uint8_t>>();
          clent.interface_type = rpc["interface_type"].as<std::string>();
          srv_clients_.push_back(clent);
          // TODO: prepare rpc
          // std_srvs::srv::RegisterEmptyClientFunc(core_.GetRpcHandle(), clent.service_name);
        }
      }
    }
    AIMRT_INFO("Init succeeded.");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }
}

bool JoyStickModule::Start() {
  executor_.Execute([this]() { MainLoop(); });

  AIMRT_INFO("Started succeeded.");
  return true;
}

void JoyStickModule::Shutdown() {
  run_flag_ = false;
  stop_sig_.get_future().wait();

  AIMRT_INFO("Shutdown succeeded.");
}

void JoyStickModule::MainLoop() {
  // TODO: prepare rpc
  // std_srvs::srv::EmptySyncProxy rpc_proxy(core_.GetRpcHandle());
  std_msgs::msg::Float32 button_msgs;
  geometry_msgs::msg::Twist vel_msgs;
  // Mode commands are edge-triggered. Joy::joy_msg_ intentionally keeps the
  // selected button latched, but a latched mode must not be republished after
  // a one-shot Leo motion returns to stand.
  std::vector<bool> previous_float_active(float_pubs_.size(), false);
  while (run_flag_.load()) {
    JoyStruct joy_data;
    joy_->GetJoyData(joy_data);

    // A joystick may be absent at startup or disconnected at runtime. Treat
    // unavailable buttons/axes as inactive instead of indexing empty vectors.
    const auto button_pressed = [&joy_data](const std::vector<uint8_t>& buttons) {
      for (const auto button : buttons) {
        if (button >= joy_data.buttons.size() || joy_data.buttons[button] == 0) {
          return false;
        }
      }
      return true;
    };
    const auto axis_value = [&joy_data](uint8_t axis) {
      return axis < joy_data.axis.size() ? joy_data.axis[axis] : 0.0;
    };

    for (size_t i = 0; i < float_pubs_.size(); ++i) {
      const auto& float_pub = float_pubs_[i];
      const bool ret = button_pressed(float_pub.buttons);
      if (ret && !previous_float_active[i]) {
        aimrt::channel::Publish<std_msgs::msg::Float32>(float_pub.pub, button_msgs);
        // Mode commands are one-shot. Clear the latch immediately so a later
        // press of the same key creates a fresh rising edge.
        joy_->ClearButtons(float_pub.buttons);
        previous_float_active[i] = false;
      } else {
        previous_float_active[i] = ret;
      }
    }

    for (auto twist_pub : twist_pubs_) {
      const bool ret = button_pressed(twist_pub.buttons);
      if (ret && limiter_) {
        array_t target_pos;
        target_pos.resize(twist_pub.axis.size());
        int32_t idx = 0;

        if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
          vel_msgs.linear.x = axis_value(twist_pub.axis.at("linear-x"));
          target_pos[idx++] = vel_msgs.linear.x;
        }
        if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
          vel_msgs.linear.y = axis_value(twist_pub.axis.at("linear-y"));
          target_pos[idx++] = vel_msgs.linear.y;
        }
        if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
          vel_msgs.linear.z = axis_value(twist_pub.axis.at("linear-z"));
          target_pos[idx++] = vel_msgs.linear.z;
        }
        if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
          vel_msgs.angular.x = axis_value(twist_pub.axis.at("angular-x"));
          target_pos[idx++] = vel_msgs.angular.x;
        }
        if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
          vel_msgs.angular.y = axis_value(twist_pub.axis.at("angular-y"));
          target_pos[idx++] = vel_msgs.angular.y;
        }
        if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
          vel_msgs.angular.z = axis_value(twist_pub.axis.at("angular-z"));
          target_pos[idx++] = vel_msgs.angular.z;
        }
        aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub, vel_msgs);

        array_t state = limiter_->update(target_pos);
        idx = 0;
        if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
          vel_msgs.linear.x = state[idx++];
        }
        if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
          vel_msgs.linear.y = state[idx++]; 
        }
        if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
          vel_msgs.linear.z = state[idx++];
        }
        if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
          vel_msgs.angular.x = state[idx++];
        }
        if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
          vel_msgs.angular.y = state[idx++];
        }
        if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
          vel_msgs.angular.z = state[idx++];
        }
        aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub_limiter, vel_msgs);
      }
    }

    for (auto srv_client : srv_clients_) {
      const bool ret = button_pressed(srv_client.buttons);
      if (ret) {
        std::string cmd = "ros2 service call /" + srv_client.service_name + " " +
                          srv_client.interface_type + " > /dev/null &";
        int ret = system(cmd.data());
        AIMRT_INFO("Call /reset_word");
        // TODO: call rpc
        // rpc_proxy.SetServiceName("reset_world");
        // auto status = rpc_proxy.Empty(req, rsp);
        // if (status.OK()) {
        //   AIMRT_INFO("GetFooData success rsp: {}", std_srvs::srv::to_yaml(rsp));
        // } else {
        //   AIMRT_WARN("Call GetFooData failed, status: {}", status.ToString());
        // }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / freq_));
  }

  stop_sig_.set_value();
}

}  // namespace xyber_x1_infer::joy_stick_module
