/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
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

#include <rclcpp/rclcpp.hpp>

// Messages
#include <can_msgs/msg/frame.hpp>
#include <dataspeed_can_msg_filters/ApproximateTime.hpp>
#include <dbw_ford_msgs/msg/brake_cmd.hpp>
#include <dbw_ford_msgs/msg/brake_info_report.hpp>
#include <dbw_ford_msgs/msg/brake_report.hpp>
#include <dbw_ford_msgs/msg/driver_assist_report.hpp>
#include <dbw_ford_msgs/msg/fuel_level_report.hpp>
#include <dbw_ford_msgs/msg/gear_cmd.hpp>
#include <dbw_ford_msgs/msg/gear_report.hpp>
#include <dbw_ford_msgs/msg/misc_cmd.hpp>
#include <dbw_ford_msgs/msg/misc1_report.hpp>
#include <dbw_ford_msgs/msg/steering_cmd.hpp>
#include <dbw_ford_msgs/msg/steering_report.hpp>
#include <dbw_ford_msgs/msg/surround_report.hpp>
#include <dbw_ford_msgs/msg/throttle_cmd.hpp>
#include <dbw_ford_msgs/msg/throttle_info_report.hpp>
#include <dbw_ford_msgs/msg/throttle_report.hpp>
#include <dbw_ford_msgs/msg/tire_pressure_report.hpp>
#include <dbw_ford_msgs/msg/wheel_position_report.hpp>
#include <dbw_ford_msgs/msg/wheel_speed_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/empty.hpp>

// The following messages are deprecated
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Platform and module version map
#include <dataspeed_dbw_common/PlatformMap.hpp>

namespace dbw_ford_can {

class DbwNode : public rclcpp::Node {
public:
  DbwNode(const rclcpp::NodeOptions& options);

private:
  void timerCallback();
  void recvEnable(const std_msgs::msg::Empty::ConstSharedPtr);
  void recvDisable(const std_msgs::msg::Empty::ConstSharedPtr);
  void recvCAN(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void recvCanImu(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs);
  void recvCanGps(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs);
  void recvBrakeCmd(const dbw_ford_msgs::msg::BrakeCmd::ConstSharedPtr msg);
  void recvThrottleCmd(const dbw_ford_msgs::msg::ThrottleCmd::ConstSharedPtr msg);
  void recvSteeringCmd(const dbw_ford_msgs::msg::SteeringCmd::ConstSharedPtr msg);
  void recvGearCmd(const dbw_ford_msgs::msg::GearCmd::ConstSharedPtr msg);
  void recvMiscCmd(const dbw_ford_msgs::msg::MiscCmd::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  bool prev_enable_;
  bool enable_;
  bool override_brake_;
  bool override_throttle_;
  bool override_steering_;
  bool override_gear_;
  bool fault_brakes_;
  bool fault_throttle_;
  bool fault_steering_;
  bool fault_steering_cal_;
  bool fault_watchdog_;
  bool fault_watchdog_using_brakes_;
  bool fault_watchdog_warned_;
  bool timeout_brakes_;
  bool timeout_throttle_;
  bool timeout_steering_;
  bool enabled_brakes_;
  bool enabled_throttle_;
  bool enabled_steering_;
  bool gear_warned_;
  bool fault() const {
    return fault_brakes_ || fault_throttle_ || fault_steering_ || fault_steering_cal_ || fault_watchdog_;
  }
  bool override() const {
    return override_brake_ || override_throttle_ || override_steering_ || override_gear_;
  }
  bool clear() const {
    return enable_ && override();
  }
  bool enabled() const {
    return enable_ && !fault() && !override();
  }
  bool publishDbwEnabled(bool force = false);
  void enableSystem();
  void disableSystem();
  void buttonCancel();
  void overrideBrake(bool override, bool timeout);
  void overrideThrottle(bool override, bool timeout);
  void overrideSteering(bool override, bool timeout);
  void overrideGear(bool override);
  void timeoutBrake(bool timeout, bool enabled);
  void timeoutThrottle(bool timeout, bool enabled);
  void timeoutSteering(bool timeout, bool enabled);
  void faultBrakes(bool fault);
  void faultThrottle(bool fault);
  void faultSteering(bool fault);
  void faultSteeringCal(bool fault);
  void faultWatchdog(bool fault, uint8_t src, bool braking);
  void faultWatchdog(bool fault, uint8_t src = 0);

  enum {
    JOINT_FL = 0, // Front left wheel
    JOINT_FR,     // Front right wheel
    JOINT_RL,     // Rear left wheel
    JOINT_RR,     // Rear right wheel
    JOINT_SL,     // Steering left
    JOINT_SR,     // Steering right
    JOINT_COUNT,  // Number of joints
  };
  sensor_msgs::msg::JointState joint_state_;

  void publishJointStates(const rclcpp::Time& stamp, const dbw_ford_msgs::msg::WheelSpeedReport* wheels,
                          const dbw_ford_msgs::msg::SteeringReport* steering);

  // Licensing
  std::string vin_;
  std::string ldate_; // License date
  std::map<uint8_t, std::string> bdate_;

  // Firmware Versions
  dataspeed_dbw_common::PlatformMap firmware_;

  // Frame ID
  std::string frame_id_;

  // Command warnings
  bool warn_cmds_;

  // Buttons (enable/disable)
  bool buttons_;

  // Pedal LUTs (local/embedded)
  bool pedal_luts_;

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  // Joint states (enable/disable)
  bool enable_joint_states_;

  // Subscribed topics
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_disable_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<dbw_ford_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<dbw_ford_msgs::msg::ThrottleCmd>::SharedPtr sub_throttle_;
  rclcpp::Subscription<dbw_ford_msgs::msg::SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<dbw_ford_msgs::msg::GearCmd>::SharedPtr sub_gear_;
  rclcpp::Subscription<dbw_ford_msgs::msg::MiscCmd>::SharedPtr sub_misc_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<dbw_ford_msgs::msg::BrakeReport>::SharedPtr pub_brake_;
  rclcpp::Publisher<dbw_ford_msgs::msg::ThrottleReport>::SharedPtr pub_throttle_;
  rclcpp::Publisher<dbw_ford_msgs::msg::SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<dbw_ford_msgs::msg::GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<dbw_ford_msgs::msg::Misc1Report>::SharedPtr pub_misc_1_;
  rclcpp::Publisher<dbw_ford_msgs::msg::WheelSpeedReport>::SharedPtr pub_wheel_speeds_;
  rclcpp::Publisher<dbw_ford_msgs::msg::WheelPositionReport>::SharedPtr pub_wheel_positions_;
  rclcpp::Publisher<dbw_ford_msgs::msg::TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<dbw_ford_msgs::msg::FuelLevelReport>::SharedPtr pub_fuel_level_;
  rclcpp::Publisher<dbw_ford_msgs::msg::SurroundReport>::SharedPtr pub_surround_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sonar_cloud_;
  rclcpp::Publisher<dbw_ford_msgs::msg::BrakeInfoReport>::SharedPtr pub_brake_info_;
  rclcpp::Publisher<dbw_ford_msgs::msg::ThrottleInfoReport>::SharedPtr pub_throttle_info_;
  rclcpp::Publisher<dbw_ford_msgs::msg::DriverAssistReport>::SharedPtr pub_driver_assist_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_fix_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_gps_vel_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_gps_time_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vin_;      // Deprecated message
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sys_enable_; // Deprecated message

  // Time Synchronization
  dataspeed_can_msg_filters::ApproximateTime sync_imu_;
  dataspeed_can_msg_filters::ApproximateTime sync_gps_;
};

} // namespace dbw_ford_can
