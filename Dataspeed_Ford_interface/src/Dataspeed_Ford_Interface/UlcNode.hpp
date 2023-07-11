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
#pragma once

//For dataspeed
#include <can_msgs/msg/frame.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_cmd.hpp>
#include <dataspeed_ulc_msgs/msg/ulc_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <dataspeed_dbw_common/PlatformMap.hpp>

//For Autoware
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>

#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

namespace dataspeed_ulc_can {

class UlcNode : public rclcpp::Node {
public:
  using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;
  using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;
  using ControlModeCommand = autoware_auto_vehicle_msgs::srv::ControlModeCommand;
  UlcNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Callbacks
  void recvCan(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void recvUlcCmd(const dataspeed_ulc_msgs::msg::UlcCmd::ConstSharedPtr msg);
  void recvTwistCmd(const geometry_msgs::msg::Twist &msg);
  void recvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void recvTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void recvEnable(const std_msgs::msg::Bool::ConstSharedPtr msg);

  //Callabcks for Autoware
  void recvAutowareTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void recvControlCmd(autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void recvGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void recvTurnIndicatorsCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void recvHazardLightsCmd(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  
  void recvVehicleCmd();  //need to be revised

  void configTimerCb();

  // Transmit CAN messages
  void sendCmdMsg(bool cfg);
  void sendCfgMsg();

  // Helper functions
  bool validInputs(const dataspeed_ulc_msgs::msg::UlcCmd &cmd) const;

  template <class T>
  T overflowSaturation(double input, T limit_min, T limit_max,
                       double scale_factor, const std::string &input_name,
                       const std::string &units) const;

  // Subscribers
  rclcpp::Subscription<dataspeed_ulc_msgs::msg::UlcCmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;
    //rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  //rclcpp::Subscription<autoware_msgs::VehicleCmd>::SharedPtr sub_vehicle_cmd;

  // Publishers
  rclcpp::Publisher<dataspeed_ulc_msgs::msg::UlcReport>::SharedPtr pub_report_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::TimerBase::SharedPtr config_timer_;

  dataspeed_ulc_msgs::msg::UlcCmd ulc_cmd_;

    // To Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_pub_;
  rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;

  rclcpp::Clock cmd_clock_;
  rclcpp::Time cmd_stamp_;
  bool enable_ = false;
  bool accel_mode_supported_ = true;
  dataspeed_dbw_common::PlatformMap firmware_;

  //added from VT
  double current_speed_;
  bool enable_;
  bool active_;

    /* input values */
  ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
};

}  // namespace dataspeed_ulc_can
