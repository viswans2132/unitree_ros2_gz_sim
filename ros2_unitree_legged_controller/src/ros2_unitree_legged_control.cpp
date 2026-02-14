// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_unitree_legged_control/ros2_unitree_legged_control.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include <condition_variable>
#include <mutex>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_set.hpp>
#include <std_msgs/msg/string.hpp>


namespace ros2_unitree_legged_control
{

struct JointLimits {
  double effort{0.0};
  double vel{0.0};
  double lower{0.0};
  double upper{0.0};
};

class RobotDescriptionCache {
public:
  // Ensure cache is populated once (blocks only on first call)
  bool ensure_ready(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lk(mtx_);

    // already built
    if (ready_) {
      return true;
    }

    // another controller is building it -> wait
    if (fetch_in_progress_) {
      return cv_.wait_for(lk, timeout, [this]() { return ready_; });
    }

    // this controller becomes the builder
    fetch_in_progress_ = true;
    lk.unlock();

    // slow work outside lock
    std::string urdf;
    std::unordered_map<std::string, JointLimits> tmp;
    bool ok = fetch_robot_description_blocking_(node, urdf, timeout) &&
              parse_limits_(urdf, tmp, node->get_logger());

    // publish result
    lk.lock();
    if (ok) {
      limits_ = std::move(tmp);
      ready_ = true;
    }
    fetch_in_progress_ = false;
    lk.unlock();
    cv_.notify_all();

    return ok;
  }

  bool get_limits(const std::string& joint, JointLimits& out) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ready_) return false;
    auto it = limits_.find(joint);
    if (it == limits_.end()) return false;
    out = it->second;
    return true;
  }

private:
  bool fetch_robot_description_blocking_(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  std::string& urdf_out,
  std::chrono::milliseconds timeout)
{
  // 1) Optional param fallback
  std::string tmp;
  if (node->get_parameter("robot_description", tmp) && !tmp.empty()) {
    urdf_out = tmp;
    return true;
  }

  // 2) Create a TEMP node in the same rcl context (NOT in controller_manager executor)
  rclcpp::NodeOptions opts;
  opts.context(node->get_node_base_interface()->get_context());
  auto temp_node = std::make_shared<rclcpp::Node>("urdf_fetch_node", opts);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();

  auto sub = temp_node->create_subscription<std_msgs::msg::String>(
    "/robot_description", qos,
    [](std_msgs::msg::String::SharedPtr) {});

  rclcpp::WaitSet ws;
  ws.add_subscription(sub);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    auto remaining = timeout - std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start);

    auto res = ws.wait(remaining);
    if (res.kind() == rclcpp::WaitResultKind::Ready) {
      std_msgs::msg::String msg;
      rclcpp::MessageInfo info;
      if (sub->take(msg, info)) {
        urdf_out = msg.data;
        return !urdf_out.empty();
      }
    }
  }

  return false;
}


  bool parse_limits_(
    const std::string& urdf,
    std::unordered_map<std::string, JointLimits>& out,
    const rclcpp::Logger& logger)
  {
    urdf::Model model;
    if (!model.initString(urdf)) {
      RCLCPP_ERROR(logger, "URDF parse failed in cache.");
      return false;
    }

    // Iterate joints and cache limits
    for (const auto& kv : model.joints_) {
      const auto& joint_name = kv.first;
      const auto& joint = kv.second;

      if (!joint) continue;
      if (!joint->limits) continue;  // skip joints without <limit>

      JointLimits lim;
      lim.effort = joint->limits->effort;
      lim.vel    = joint->limits->velocity;

      // Only revolute/prismatic have bounds that matter
      lim.lower  = joint->limits->lower;
      lim.upper  = joint->limits->upper;

      out.emplace(joint_name, lim);
    }

    if (out.empty()) {
      RCLCPP_ERROR(logger, "No joint limits found while building cache.");
      return false;
    }
    return true;
  }

private:
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  bool ready_{false};
  bool fetch_in_progress_{false};
  std::unordered_map<std::string, JointLimits> limits_;
};

RobotDescriptionCache& global_cache()
{
  static RobotDescriptionCache cache;
  return cache;
}




UnitreeLeggedController::UnitreeLeggedController()
: controller_interface::ControllerInterface(),
  joints_command_subscriber_(nullptr), rt_controller_state_publisher_(nullptr), state_publisher_(nullptr)
{
  // memset(&lastCmd, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorCmd));
  // memset(&lastState, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorState));
  lastCmd = ros2_unitree_legged_msgs::msg::MotorCmd();
  lastState = ros2_unitree_legged_msgs::msg::MotorState();
  memset(&servoCmd, 0, sizeof(ServoCmd));
}

controller_interface::CallbackReturn UnitreeLeggedController::on_init()
{

  try
  {
    declare_parameters();
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // robot_desc_sub_ = get_node()->create_subscription<std_msgs::msg::String>("/robot_description", qos, 
    //                 [this](const std_msgs::msg::String::SharedPtr msg) 
    //                 {urdf_string_ = msg->data; 
    //                 urdf_received_.store(true, std::memory_order_release);
    //                 });


    // Initialize variables and pointers
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void UnitreeLeggedController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn UnitreeLeggedController::read_parameters()
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "param_listener does not exist");
    return controller_interface::CallbackReturn::ERROR;
  }

  params_ = param_listener_->get_params();
  if (params_.joint_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joint_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_name_ = params_.joint_name;
  limits_loaded_ = false;

  // Ensure global cache is ready (only first controller blocks)
  if (!global_cache().ensure_ready(get_node(), std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to build global joint limits cache from URDF.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get this controller's joint limits from cache
  JointLimits lim;
  if (!global_cache().get_limits(joint_name_, lim)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Joint '%s' not found in URDF limits cache (or missing <limit>).",
      joint_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Cache limits into RT-safe doubles for update()
  effort_limit_ = lim.effort;
  vel_limit_    = lim.vel;
  lower_limit_  = lim.lower;
  upper_limit_  = lim.upper;

  if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 ||
      !std::isfinite(vel_limit_)    || vel_limit_    <= 0.0 ||
      !std::isfinite(lower_limit_)  || !std::isfinite(upper_limit_) ||
      lower_limit_ >= upper_limit_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Invalid URDF limits for joint '%s': effort=%f vel=%f pos=[%f,%f]",
      joint_name_.c_str(), effort_limit_, vel_limit_, lower_limit_, upper_limit_);
    return controller_interface::CallbackReturn::ERROR;
  }

  limits_loaded_ = true;

  RCLCPP_INFO(get_node()->get_logger(),
    "Configured joint '%s' limits: effort=%g vel=%g pos=[%g,%g]",
    joint_name_.c_str(), effort_limit_, vel_limit_, lower_limit_, upper_limit_);

  return controller_interface::CallbackReturn::SUCCESS;
}



void UnitreeLeggedController::setCmdCallback(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg){
  lastCmd.mode = msg->mode;
  lastCmd.q = msg->q;
  lastCmd.kp = msg->kp;
  lastCmd.dq = msg->dq;
  lastCmd.kd = msg->kd;
  lastCmd.tau = msg->tau;

  rt_command_.writeFromNonRT(lastCmd); 
}

controller_interface::CallbackReturn UnitreeLeggedController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  
  // command callback
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/command", rclcpp::SystemDefaultsQoS(), std::bind(&UnitreeLeggedController::setCmdCallback, this, std::placeholders::_1));

  state_publisher_ = get_node()->create_publisher<StateType>(
    "~/state", rclcpp::SystemDefaultsQoS()
  );
  rt_controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<StateType>>(state_publisher_);


  // 1) Ensure cache exists (only first controller blocks)
  if (!global_cache().ensure_ready(get_node(), std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Timed out or failed building URDF cache from /robot_description");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 2) Read this controller’s joint limits from cache
  JointLimits lim;
  if (!global_cache().get_limits(joint_name_, lim)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Joint '%s' not found in URDF limit cache (or has no <limit>)",
      joint_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 3) Copy into RT-safe doubles (what your update() clamps against)
  effort_limit_ = lim.effort;
  vel_limit_    = lim.vel;
  lower_limit_  = lim.lower;
  upper_limit_  = lim.upper;

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UnitreeLeggedController::command_interface_configuration() const
{
  // controller_interface::InterfaceConfiguration command_interfaces_config;
  // command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // command_interfaces_config.names = command_interface_types_;
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);

  return {controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};

}

controller_interface::InterfaceConfiguration UnitreeLeggedController::state_interface_configuration()
  const
{ 
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};
}

controller_interface::CallbackReturn UnitreeLeggedController::on_activate(
  const rclcpp_lifecycle::State &)
{
  activation_time_ = get_node()->now();
  hold_initialized_ = false;
  ramp_done_ = false;

  if (!limits_loaded_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Refusing to activate: joint limits not loaded (URDF missing or invalid).");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 1) Validate we have expected interfaces
  if (state_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces available.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces available.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 2) Read initial position safely
  double init_pose = state_interfaces_[0].get_value();
  if (!std::isfinite(init_pose)) {
    RCLCPP_WARN(get_node()->get_logger(),
      "Initial joint state is not finite (q=%g). Using 0.0 as fallback.", init_pose);
    init_pose = 0.0;
  }

  // 3) Initialize controller internal state
  lastCmd.q = static_cast<float>(init_pose);
  lastState.q = static_cast<float>(init_pose);
  lastCmd.dq = 0.0f;
  lastState.dq = 0.0f;
  lastCmd.tau = 0.0f;
  lastState.tau_est = 0.0f;

  // 4) CRITICAL: initialize commanded effort to 0
  for (auto & cmd : command_interfaces_) {
    cmd.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn UnitreeLeggedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ = realtime_tools::RealtimeBuffer<CmdType>();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UnitreeLeggedController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 1e-6) {
    command_interfaces_[0].set_value(0.0);
    return controller_interface::return_type::OK;
  }

  // ---- 1) compute ramp alpha (one-shot after activation) ----
  double alpha = 1.0;
  if (!ramp_done_) {
    const double t = (time - activation_time_).seconds();
    if (t <= 0.0) {
      alpha = 0.0;
    } else if (ramp_sec_ > 1e-6) {
      alpha = std::clamp(t / ramp_sec_, 0.0, 1.0);
    } else {
      alpha = 1.0;
    }

    if (alpha >= 1.0) {
      ramp_done_ = true;
      alpha = 1.0;
    }
  }

  // ---- 2) read state safely ----
  const double currentPos = state_interfaces_[0].get_value();
  if (!std::isfinite(currentPos)) {
    command_interfaces_[0].set_value(0.0);
    publishState();
    return controller_interface::return_type::OK;
  }

  double currentVel = computeVel(
    currentPos,
    static_cast<double>(lastState.q),
    static_cast<double>(lastState.dq),
    dt);

  if (!std::isfinite(currentVel)) {
    currentVel = 0.0;
  }

  // ---- 3) read latest command (RT buffer) ----
  auto joint_commands = rt_command_.readFromRT();

  // ---- 4) If no command has arrived yet: HOLD pose (NOT zero torque) ----
  if (!joint_commands) {
    if (!hold_initialized_) {
      hold_initialized_ = true;
      hold_pos_ = currentPos;  // latch pose at first update with no command
    }

    // Simple PD hold
    double tau = hold_kp_ * (hold_pos_ - currentPos) + hold_kd_ * (0.0 - currentVel);

    if (!std::isfinite(tau)) {
      tau = 0.0;
    }

    // Apply startup ramp to avoid jerk at enable
    tau *= alpha;

    effortLimits(tau);
    command_interfaces_[0].set_value(tau);

    // Update lastState and publish
    lastState.q  = static_cast<float>(currentPos);
    lastState.dq = static_cast<float>(currentVel);

    const double tau_est = state_interfaces_[1].get_value();
    lastState.tau_est = std::isfinite(tau_est) ? static_cast<float>(tau_est) : 0.0f;

    publishState();
    return controller_interface::return_type::OK;
  }

  // ---- 5) Command exists: normal control ----
  hold_initialized_ = false;  // stop holding once real commands arrive
  lastCmd = *(joint_commands);

  // Validate command fields
  if (!std::isfinite(lastCmd.q)  || !std::isfinite(lastCmd.dq) ||
      !std::isfinite(lastCmd.kp) || !std::isfinite(lastCmd.kd) ||
      !std::isfinite(lastCmd.tau)) {
    command_interfaces_[0].set_value(0.0);
    publishState();
    return controller_interface::return_type::OK;
  }

  // Build servoCmd from lastCmd (make sure it's not stale)
  servoCmd.mode         = static_cast<uint8_t>(lastCmd.mode);
  servoCmd.pos          = static_cast<double>(lastCmd.q);
  servoCmd.vel          = static_cast<double>(lastCmd.dq);
  servoCmd.posStiffness = static_cast<double>(lastCmd.kp);
  servoCmd.velStiffness = static_cast<double>(lastCmd.kd);
  servoCmd.torque       = static_cast<double>(lastCmd.tau);

  positionLimits(servoCmd.pos);
  velocityLimits(servoCmd.vel);
  effortLimits(servoCmd.torque);

  double calcTorque = computeTorque(currentPos, currentVel, servoCmd);

  if (!std::isfinite(calcTorque)) {
    calcTorque = 0.0;
  }

  // Apply startup ramp only during the ramp window
  calcTorque *= alpha;

  effortLimits(calcTorque);
  command_interfaces_[0].set_value(calcTorque);

  // Update lastState and publish
  lastState.q  = static_cast<float>(currentPos);
  lastState.dq = static_cast<float>(currentVel);

  const double tau_est = state_interfaces_[1].get_value();
  lastState.tau_est = std::isfinite(tau_est) ? static_cast<float>(tau_est) : 0.0f;

  publishState();
  return controller_interface::return_type::OK;
}


void UnitreeLeggedController::publishState(){
  if (rt_controller_state_publisher_ && rt_controller_state_publisher_->trylock()) {
    rt_controller_state_publisher_->msg_.q = lastState.q;
    rt_controller_state_publisher_->msg_.dq = lastState.dq;
    rt_controller_state_publisher_->msg_.tau_est = lastState.tau_est;
    rt_controller_state_publisher_->unlockAndPublish();    
  }
}

void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

 void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}
void UnitreeLeggedController::positionLimits(double & position)
{
  if (!limits_loaded_) return;
  position = std::clamp(position, lower_limit_, upper_limit_);
}

void UnitreeLeggedController::velocityLimits(double & velocity)
{
  if (!limits_loaded_) return;
  velocity = std::clamp(velocity, -vel_limit_, vel_limit_);
}

void UnitreeLeggedController::effortLimits(double & effort)
{
  if (!limits_loaded_) return;
  effort = std::clamp(effort, -effort_limit_, effort_limit_);
}




}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_unitree_legged_control::UnitreeLeggedController, controller_interface::ControllerInterface)