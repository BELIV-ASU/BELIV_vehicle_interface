/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "UlcNode.hpp"

#include <chrono>
#include <dataspeed_ulc_can/dispatch.hpp>

using namespace dataspeed_dbw_common;

namespace dataspeed_ulc_can {

template <class T>
T UlcNode::overflowSaturation(double input, T limit_min, T limit_max, double scale_factor,
                              const std::string &input_name, const std::string &units) const {
  if (input < (limit_min * scale_factor)) {
    RCLCPP_WARN(get_logger(), "%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input,
                units.c_str(), limit_min * scale_factor, units.c_str());
    return limit_min;
  } else if (input > (limit_max * scale_factor)) {
    RCLCPP_WARN(get_logger(), "%s [%f %s] out of range -- saturating to %f %s", input_name.c_str(), input,
                units.c_str(), limit_max * scale_factor, units.c_str());
    return limit_max;
  } else {
    return input / scale_factor;
  }
}

// Last firmware versions before new ULC interface
PlatformMap OLD_ULC_FIRMWARE({
  {PlatformVersion(P_FCA_RU,      M_STEER, ModuleVersion(1,5,2))},
  {PlatformVersion(P_FCA_WK2,     M_STEER, ModuleVersion(1,3,2))},
  {PlatformVersion(P_FORD_C1,     M_STEER, ModuleVersion(1,2,2))},
  {PlatformVersion(P_FORD_CD4,    M_STEER, ModuleVersion(2,5,2))},
  {PlatformVersion(P_FORD_CD5,    M_STEER, ModuleVersion(1,1,2))},
  {PlatformVersion(P_FORD_GE1,    M_STEER, ModuleVersion(0,1,0))},
  {PlatformVersion(P_FORD_P5,     M_STEER, ModuleVersion(1,4,2))},
  {PlatformVersion(P_FORD_T6,     M_STEER, ModuleVersion(0,2,2))},
  {PlatformVersion(P_FORD_U6,     M_STEER, ModuleVersion(1,0,2))},
  {PlatformVersion(P_POLARIS_GEM, M_STEER, ModuleVersion(1,1,1))},
  {PlatformVersion(P_POLARIS_RZR, M_STEER, ModuleVersion(0,3,1))},
});

UlcNode::UlcNode(const rclcpp::NodeOptions &options) : rclcpp::Node("ulc_node", options), enable_(false) 
{
    /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;
  
  // Setup publishers
  pub_report_ = create_publisher<dataspeed_ulc_msgs::msg::UlcReport>("ulc_report", 2);
  pub_can_ = create_publisher<can_msgs::msg::Frame>("can_tx", 100);

  // Setup subscribers
  using std::placeholders::_1;
  using std::placeholders::_2;
  {
    auto bind = std::bind(&UlcNode::recvCan, this, _1);
    sub_can_ = create_subscription<can_msgs::msg::Frame>("can_rx", 100, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvUlcCmd, this, _1);
    sub_cmd_ = create_subscription<dataspeed_ulc_msgs::msg::UlcCmd>("ulc_cmd", 2, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvTwist, this, _1);
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 2, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvTwistStamped, this, _1);
    sub_twist_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 2, bind);
  }
  {
    auto latch_like_qos = rclcpp::QoS(2).transient_local();
    auto bind = std::bind(&UlcNode::recvEnable, this, _1);
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("dbw_enabled", latch_like_qos, bind);
  }
  
  // From autoware
  // control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
  //   "/control/command/control_cmd", 1, std::bind(&PacmodInterface::callbackControlCmd, this, _1));
  // gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
  //    "/control/command/gear_cmd", 1, std::bind(&PacmodInterface::callbackGearCmd, this, _1));
  //  turn_indicators_cmd_sub_ =
  //    create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
  //      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
  //      std::bind(&PacmodInterface::callbackTurnIndicatorsCommand, this, _1));
  //  hazard_lights_cmd_sub_ =
  //    create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
  //      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
  //      std::bind(&PacmodInterface::callbackHazardLightsCommand, this, _1));
  {
    auto bind = std::bind(&UlcNode::recvControlCmd, this, _1);
    control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>
    ("/control/command/control_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvGearCmd, this, _1);
    gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>
    ("/control/command/gear_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&UlcNode::recvTurnIndicatorsCmd, this, _1);
    turn_indicators_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>
    ("/control/command/turn_indicators_cmd", 1, bind);
  }

  {
    auto bind = std::bind(&UlcNode::recvHazardLightsCmd, this, _1);
    hazard_lights_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>
    ("/control/command/hazard_lights_cmd", 1, bind);
  }


   actuation_cmd_sub_ = create_subscription<ActuationCommandStamped>(
     "/control/command/actuation_cmd", 1,
     std::bind(&PacmodInterface::callbackActuationCmd, this, _1));
   emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
     "/control/command/emergency_cmd", 1,
     std::bind(&PacmodInterface::callbackEmergencyCmd, this, _1));
   control_mode_server_ = create_service<ControlModeCommand>(
     "input/control_mode_request", std::bind(&PacmodInterface::onControlModeRequest, this, _1, _2));

  // Initialize timestamp
  cmd_clock_ = rclcpp::Clock(RCL_ROS_TIME);
  cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Setup timer for config message retransmission
  double freq = declare_parameter<double>("config_frequency", 5.0);
  freq = std::clamp(freq, 5.0, 50.0);
  set_parameter(rclcpp::Parameter("config_frequency", freq));

  auto duration = std::chrono::microseconds(static_cast<uint32_t>(1000000.0 / freq));
  config_timer_ = create_wall_timer(duration, std::bind(&UlcNode::configTimerCb, this));
}

void UlcNode::recvEnable(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  enable_ = msg->data;
}

void UlcNode::recvUlcCmd(const dataspeed_ulc_msgs::msg::UlcCmd::ConstSharedPtr msg) {
  // Check for differences in acceleration and jerk limits
  bool diff = (msg->linear_accel != ulc_cmd_.linear_accel) || (msg->linear_decel != ulc_cmd_.linear_decel) ||
              (msg->lateral_accel != ulc_cmd_.lateral_accel) || (msg->angular_accel != ulc_cmd_.angular_accel) ||
              (msg->jerk_limit_throttle != ulc_cmd_.jerk_limit_throttle) || (msg->jerk_limit_brake != ulc_cmd_.jerk_limit_brake);

  ulc_cmd_ = *msg;

  // Publish command message
  sendCmdMsg(true);

  // Publish config message on change
  if (diff) {
    sendCfgMsg();
  }
}

void UlcNode::recvTwistCmd(const geometry_msgs::msg::Twist &msg) {
  // Populate command fields
  ulc_cmd_.linear_velocity = msg.linear.x;
  ulc_cmd_.accel_cmd = 0.0; // Not used when pedals_mode is SPEED_MODE
  ulc_cmd_.pedals_mode = dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE;
  ulc_cmd_.coast_decel = false;
  ulc_cmd_.yaw_command = msg.angular.z;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = false;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;
  ulc_cmd_.jerk_limit_throttle = 0;
  ulc_cmd_.jerk_limit_brake = 0;

  // Publish command message
  sendCmdMsg(false);
}

void UlcNode::recvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  recvTwistCmd(*msg);
}

void UlcNode::recvTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
  recvTwistCmd(msg->twist);
}

void UlcNode::recvAutowareTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  recvTwistCmd(msg->twist);
}

void UlcNode::recvControlCmd(
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  float COUNTS_PER_REV=125.5;
  
  // Populate command fields
  ulc_cmd_.linear_velocity = msg->longitudinal.speed;
  ulc_cmd_.accel_cmd = 0.0; // Not used when pedals_mode is SPEED_MODE
  ulc_cmd_.pedals_mode = dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE;
  ulc_cmd_.coast_decel = false;
  ulc_cmd_.yaw_command = msg->lateral.steering_tire_angle* COUNTS_PER_REV;
  ulc_cmd_.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;


  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_pedals = true;
  ulc_cmd_.enable_shifting = true;
  ulc_cmd_.enable_steering = true;
  ulc_cmd_.shift_from_park = false;
  ulc_cmd_.linear_accel = 0;
  ulc_cmd_.linear_decel = 0;
  ulc_cmd_.angular_accel = 0;
  ulc_cmd_.lateral_accel = 0;
  ulc_cmd_.jerk_limit_throttle = 0;
  ulc_cmd_.jerk_limit_brake = 0;

  // Publish command message
  sendCmdMsg(false);
}

void UlcNode::recvGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void UlcNode::recvTurnIndicatorsCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void UlcNode::recvHazardLightsCmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

void UlcNode::recvVehicleCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  //recvControlCmd(msg->ctrl_cmd);lateral
  recvAutowareTwistStamped(msg->twist_cmd);
}

void UlcNode::recvCan(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_ULC_REPORT:
        if (msg->dlc >= sizeof(MsgUlcReport)) {
          const MsgUlcReport *ptr = reinterpret_cast<const MsgUlcReport *>(msg->data.data());
          dataspeed_ulc_msgs::msg::UlcReport report;
          report.header.stamp = msg->header.stamp;
          report.speed_ref = (float)ptr->speed_ref * 0.02f;
          report.timeout = ptr->timeout;
          report.pedals_enabled = ptr->pedals_enabled;
          report.pedals_mode = ptr->pedals_mode;
          report.speed_meas = (float)ptr->speed_meas * 0.02f;
          report.override_latched = ptr->override;
          report.steering_enabled = ptr->steering_enabled;
          report.steering_mode = ptr->steering_mode;
          report.accel_ref = (float)ptr->accel_ref * 0.05f;
          report.accel_meas = (float)ptr->accel_meas * 0.05f;
          report.max_steering_angle = (float)ptr->max_steering_angle * 5.0f;
          report.coasting = ptr->coasting;
          report.max_steering_vel = (float)ptr->max_steering_vel * 8.0f;
          report.steering_preempted = ptr->steering_preempted;
          report.speed_preempted = ptr->speed_preempted;
          pub_report_->publish(report);
        }
        break;
      case ID_VERSION:
        if (msg->dlc >= sizeof(MsgVersion)) {
          const MsgVersion *ptr = (const MsgVersion*)msg->data.data();
          if ((Module)ptr->module == M_STEER) {
            const PlatformVersion version((Platform)ptr->platform, (Module)ptr->module, ptr->major, ptr->minor, ptr->build);
            const ModuleVersion old_ulc_version = OLD_ULC_FIRMWARE.get(version);
            const char * str_p = platformToString(version.p);
            const char * str_m = moduleToString(version.m);
            if (firmware_.get(version) != version.v) {
              firmware_.put(version);
              if (version <= old_ulc_version) {
                accel_mode_supported_ = false;
                RCLCPP_WARN(get_logger(), "Firmware %s %s  version %u.%u.%u does not support ULC acceleration interface mode.", str_p, str_m,
                        version.v.major(), version.v.minor(), version.v.build());
              }
            }
          }
        }
        break;
    }
  }
}

void UlcNode::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
    engage_cmd_ = true;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL) {
    engage_cmd_ = false;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

// rear door status in UlcNode
// void UlcNode::callbackRearDoor(
//   const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt)
// {
//   /* publish current door status */
//   door_status_pub_->publish(toAutowareDoorStatusMsg(*rear_door_rpt));
// }

void UlcNode::sendCmdMsg(bool cfg) {
  // Validate input fields
  if (validInputs(ulc_cmd_)) {
    if (cfg) {
      cmd_stamp_ = cmd_clock_.now();
    }
  } else {
    cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  // Build CAN message
  can_msgs::msg::Frame msg;
  msg.id = ID_ULC_CMD;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCmd);
  MsgUlcCmd *ptr = reinterpret_cast<MsgUlcCmd *>(msg.data.data());
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate enable bits
  if (enable_) {
    ptr->enable_pedals = ulc_cmd_.enable_pedals;
    ptr->enable_steering = ulc_cmd_.enable_steering;
    ptr->enable_shifting = ulc_cmd_.enable_shifting;
    ptr->shift_from_park = ulc_cmd_.shift_from_park;
  }

  // Populate command fields
  ptr->clear = ulc_cmd_.clear;
  ptr->pedals_mode = ulc_cmd_.pedals_mode;
  ptr->steering_mode = ulc_cmd_.steering_mode;
  ptr->coast_decel = ulc_cmd_.coast_decel;
  if (ulc_cmd_.pedals_mode == dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE) {
    ptr->lon_command =
        overflowSaturation(ulc_cmd_.linear_velocity, INT16_MIN, INT16_MAX, 0.0025, "ULC command speed", "m/s");
  } else if (ulc_cmd_.pedals_mode == dataspeed_ulc_msgs::msg::UlcCmd::ACCEL_MODE) {
    if (!accel_mode_supported_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Firmware does not support acceleration mode interface");
      return;
    }
    ptr->lon_command =
        overflowSaturation(ulc_cmd_.accel_cmd, INT16_MIN, INT16_MAX, 5e-4, "ULC command accel", "m/s^2");
  } else {
    ptr->lon_command = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Unsupported ULC pedal control mode [%d]",
                         ulc_cmd_.pedals_mode);
    cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE) {
    ptr->yaw_command =
        overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.00025, "ULC yaw rate command", "rad/s");
  } else if (ulc_cmd_.steering_mode == dataspeed_ulc_msgs::msg::UlcCmd::CURVATURE_MODE) {
    ptr->yaw_command =
        overflowSaturation(ulc_cmd_.yaw_command, INT16_MIN, INT16_MAX, 0.0000061, "ULC curvature command", "1/m");
  } else {
    ptr->yaw_command = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Unsupported ULC steering control mode [%d]",
                         ulc_cmd_.steering_mode);
    cmd_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return;
  }

  // Publish message
  pub_can_->publish(msg);
}

void UlcNode::sendCfgMsg() {
  // Build CAN message
  can_msgs::msg::Frame msg;
  msg.id = ID_ULC_CONFIG;
  msg.is_extended = false;
  msg.dlc = sizeof(MsgUlcCfg);
  MsgUlcCfg *ptr = reinterpret_cast<MsgUlcCfg *>(msg.data.data());
  memset(ptr, 0x00, sizeof(*ptr));

  // Populate acceleration and jerk limits
  ptr->linear_accel = overflowSaturation(ulc_cmd_.linear_accel, 0, UINT8_MAX, 0.025, "Linear accel limit", "m/s^2");
  ptr->linear_decel = overflowSaturation(ulc_cmd_.linear_decel, 0, UINT8_MAX, 0.025, "Linear decel limit", "m/s^2");
  ptr->lateral_accel = overflowSaturation(ulc_cmd_.lateral_accel, 0, UINT8_MAX, 0.05, "Lateral accel limit", "m/s^2");
  ptr->angular_accel = overflowSaturation(ulc_cmd_.angular_accel, 0, UINT8_MAX, 0.02, "Angular accel limit", "rad/s^2");
  ptr->jerk_limit_throttle = overflowSaturation(ulc_cmd_.jerk_limit_throttle, 0, UINT8_MAX, 0.1, "Throttle jerk limit", "m/s^3");
  ptr->jerk_limit_brake = overflowSaturation(ulc_cmd_.jerk_limit_brake, 0, UINT8_MAX, 0.1, "Brake jerk limit", "m/s^3");

  // Publish message
  pub_can_->publish(msg);

  // Reset timer
  config_timer_->reset();
}

void UlcNode::configTimerCb() {
  // Retransmit config message while command is valid
  if (cmd_clock_.now() - cmd_stamp_ < std::chrono::milliseconds(100)) {
    sendCfgMsg();
  }
}

bool UlcNode::validInputs(const dataspeed_ulc_msgs::msg::UlcCmd &cmd) const {
  bool valid = true;
  if (std::isnan(cmd.linear_velocity) && cmd.pedals_mode == dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE) {
    RCLCPP_WARN(get_logger(), "NaN input detected on speed input");
    valid = false;
  }
  if (std::isnan(cmd.accel_cmd) && cmd.pedals_mode == dataspeed_ulc_msgs::msg::UlcCmd::ACCEL_MODE) {
    RCLCPP_WARN(get_logger(), "NaN input detected on accel input");
    valid = false;
  }
  if (cmd.pedals_mode != dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE && cmd.pedals_mode != dataspeed_ulc_msgs::msg::UlcCmd::ACCEL_MODE) {
    RCLCPP_WARN(get_logger(), "Invalid pedals mode in command message");
    valid = false;
  }
  if (std::isnan(cmd.yaw_command)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on yaw command input");
    valid = false;
  }
  if (std::isnan(cmd.linear_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on linear accel input");
    valid = false;
  }
  if (std::isnan(cmd.linear_decel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on linear decel input");
    valid = false;
  }
  if (std::isnan(cmd.lateral_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on lateral accel input");
    valid = false;
  }
  if (std::isnan(cmd.angular_accel)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on angular accel input");
    valid = false;
  }
  if (std::isnan(cmd.jerk_limit_throttle)) {control
    RCLCPP_WARN(get_logger(), "NaN input detected on throttle jerk input");
    valid = false;
  }
  if (std::isnan(cmd.jerk_limit_brake)) {
    RCLCPP_WARN(get_logger(), "NaN input detected on brake jerk input");
    valid = false;
  }
  return valid;
}

} // namespace dataspeed_ulc_can

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dataspeed_ulc_can::UlcNode)
