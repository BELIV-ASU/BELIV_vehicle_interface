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

#include "DbwNode.hpp"

#include <dbw_ford_can/dispatch.hpp>
#include <dbw_ford_can/pedal_lut.hpp>
#include <dbw_ford_can/sonar_lut.hpp>
#include <unordered_set>

// Log once per unique identifier, similar to RCLCPP_INFO_ONCE()
#define DS_LOG_ONCE_ID(logger, log_macro, id, ...) \
  do {                                             \
    static std::unordered_set<int> set;            \
    if (RCUTILS_UNLIKELY(set.count(id) == 0)) {    \
      set.insert((id));                            \
      log_macro((logger), __VA_ARGS__);            \
    }                                              \
  } while (0)

#define DS_INFO_ONCE_ID(logger, id, ...) DS_LOG_ONCE_ID((logger), RCLCPP_INFO, (id), __VA_ARGS__)
#define DS_WARN_ONCE_ID(logger, id, ...) DS_LOG_ONCE_ID((logger), RCLCPP_WARN, (id), __VA_ARGS__)

namespace dbw_ford_can {
using namespace dataspeed_dbw_common;

// Latest firmware versions
PlatformMap FIRMWARE_LATEST({
  {PlatformVersion(P_FORD_C1,   M_TPEC,  ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_C1,   M_STEER, ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_C1,   M_SHIFT, ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_C1,   M_ABS,   ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_C1,   M_BOO,   ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_C1,   M_EPS,   ModuleVersion(1,3,3))},
  {PlatformVersion(P_FORD_CD4,  M_BPEC,  ModuleVersion(2,6,3))},
  {PlatformVersion(P_FORD_CD4,  M_TPEC,  ModuleVersion(2,6,3))},
  {PlatformVersion(P_FORD_CD4,  M_STEER, ModuleVersion(2,6,3))},
  {PlatformVersion(P_FORD_CD4,  M_SHIFT, ModuleVersion(2,6,3))},
  {PlatformVersion(P_FORD_CD5,  M_BOO,   ModuleVersion(1,2,3))},
  {PlatformVersion(P_FORD_CD5,  M_TPEC,  ModuleVersion(1,2,3))},
  {PlatformVersion(P_FORD_CD5,  M_STEER, ModuleVersion(1,2,3))},
  {PlatformVersion(P_FORD_GE1,  M_TPEC,  ModuleVersion(1,0,3))},
  {PlatformVersion(P_FORD_GE1,  M_STEER, ModuleVersion(1,0,3))},
  {PlatformVersion(P_FORD_GE1,  M_SHIFT, ModuleVersion(1,0,3))},
  {PlatformVersion(P_FORD_P5,   M_TPEC,  ModuleVersion(1,5,3))},
  {PlatformVersion(P_FORD_P5,   M_STEER, ModuleVersion(1,5,3))},
  {PlatformVersion(P_FORD_P5,   M_SHIFT, ModuleVersion(1,5,3))},
  {PlatformVersion(P_FORD_P5,   M_ABS,   ModuleVersion(1,5,3))},
  {PlatformVersion(P_FORD_P5,   M_BOO,   ModuleVersion(1,5,3))},
  {PlatformVersion(P_FORD_P702, M_TPEC,  ModuleVersion(0,1,3))},
  {PlatformVersion(P_FORD_P702, M_STEER, ModuleVersion(0,1,3))},
  {PlatformVersion(P_FORD_P702, M_SHIFT, ModuleVersion(0,1,3))},
  {PlatformVersion(P_FORD_T6,   M_TPEC,  ModuleVersion(0,3,3))},
  {PlatformVersion(P_FORD_T6,   M_STEER, ModuleVersion(0,3,3))},
  {PlatformVersion(P_FORD_T6,   M_SHIFT, ModuleVersion(0,3,3))},
  {PlatformVersion(P_FORD_U6,   M_TPEC,  ModuleVersion(1,1,3))},
  {PlatformVersion(P_FORD_U6,   M_STEER, ModuleVersion(1,1,3))},
  {PlatformVersion(P_FORD_U6,   M_SHIFT, ModuleVersion(1,1,3))},
  {PlatformVersion(P_FORD_U6,   M_ABS,   ModuleVersion(1,1,3))},
  {PlatformVersion(P_FORD_U6,   M_BOO,   ModuleVersion(1,1,3))},
});

using std::placeholders::_1;

DbwNode::DbwNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("dbw_node", options),
      sync_imu_(10, std::bind(&DbwNode::recvCanImu, this, _1), ID_REPORT_ACCEL, ID_REPORT_GYRO),
      sync_gps_(10, std::bind(&DbwNode::recvCanGps, this, _1), ID_REPORT_GPS1, ID_REPORT_GPS2, ID_REPORT_GPS3) {
  // Reduce synchronization delay
  sync_imu_.setInterMessageLowerBound(std::chrono::milliseconds(3));   // 10ms period
  sync_gps_.setInterMessageLowerBound(std::chrono::milliseconds(300)); // 1s period

  // Initialize enable state machine
  prev_enable_ = true;
  enable_ = false;
  override_brake_ = false;
  override_throttle_ = false;
  override_steering_ = false;
  override_gear_ = false;
  fault_brakes_ = false;
  fault_throttle_ = false;
  fault_steering_ = false;
  fault_steering_cal_ = false;
  fault_watchdog_ = false;
  fault_watchdog_using_brakes_ = false;
  fault_watchdog_warned_ = false;
  timeout_brakes_ = false;
  timeout_throttle_ = false;
  timeout_steering_ = false;
  enabled_brakes_ = false;
  enabled_throttle_ = false;
  enabled_steering_ = false;
  gear_warned_ = false;

  // Frame ID
  frame_id_ = declare_parameter<std::string>("frame_id", "base_footprint");

  // Warn on received commands
  warn_cmds_ = declare_parameter<bool>("warn_cmds", true);

  // Buttons (enable/disable)
  buttons_ = declare_parameter<bool>("buttons", true);

  // Pedal LUTs (local/embedded)
  pedal_luts_ = declare_parameter<bool>("pedal_luts", false);

  // Ackermann steering parameters
  acker_wheelbase_ = declare_parameter<double>("ackermann_wheelbase", 2.8498); // 112.2 inches
  acker_track_ = declare_parameter<double>("ackermann_track", 1.5824); // 62.3 inches
  steering_ratio_ = declare_parameter<double>("steering_ratio", 14.8);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl_joint"; // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr_joint"; // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl_joint"; // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr_joint"; // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl_joint";
  joint_state_.name[JOINT_SR] = "steer_fr_joint";

  // Setup Publishers

  pub_can_ = create_publisher<can_msgs::msg::Frame>("can_tx", 10);
  pub_brake_ = create_publisher<dbw_ford_msgs::msg::BrakeReport>("brake_report", 2);
  pub_throttle_ = create_publisher<dbw_ford_msgs::msg::ThrottleReport>("throttle_report", 2);
  pub_steering_ = create_publisher<dbw_ford_msgs::msg::SteeringReport>("steering_report", 2);
  pub_gear_ = create_publisher<dbw_ford_msgs::msg::GearReport>("gear_report", 2);
  pub_misc_1_ = create_publisher<dbw_ford_msgs::msg::Misc1Report>("misc_1_report", 2);
  pub_wheel_speeds_ = create_publisher<dbw_ford_msgs::msg::WheelSpeedReport>("wheel_speed_report", 2);
  pub_wheel_positions_ = create_publisher<dbw_ford_msgs::msg::WheelPositionReport>("wheel_position_report", 2);
  pub_tire_pressure_ = create_publisher<dbw_ford_msgs::msg::TirePressureReport>("tire_pressure_report", 2);
  pub_fuel_level_ = create_publisher<dbw_ford_msgs::msg::FuelLevelReport>("fuel_level_report", 2);
  pub_surround_ = create_publisher<dbw_ford_msgs::msg::SurroundReport>("surround_report", 2);
  pub_sonar_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("sonar_cloud", 2);
  pub_brake_info_ = create_publisher<dbw_ford_msgs::msg::BrakeInfoReport>("brake_info_report", 2);
  pub_throttle_info_ = create_publisher<dbw_ford_msgs::msg::ThrottleInfoReport>("throttle_info_report", 2);
  pub_driver_assist_ = create_publisher<dbw_ford_msgs::msg::DriverAssistReport>("driver_assist_report", 2);
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  pub_gps_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
  pub_gps_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>("gps/vel", 10);
  pub_gps_time_ = create_publisher<sensor_msgs::msg::TimeReference>("gps/time", 10);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);
  // Due to the way DDS works, this must also be set on the subscriber side
  auto latch_like_qos = rclcpp::QoS(1).transient_local();
  pub_vin_ = create_publisher<std_msgs::msg::String>("vin", latch_like_qos);
  pub_sys_enable_ = create_publisher<std_msgs::msg::Bool>("dbw_enabled", latch_like_qos);
  publishDbwEnabled();

  // Publish joint states if enabled
  enable_joint_states_ = declare_parameter<bool>("joint_states", true);
  if (enable_joint_states_) {
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  // Setup Subscribers
  {
    auto bind = std::bind(&DbwNode::recvEnable, this, _1);
    sub_enable_ = create_subscription<std_msgs::msg::Empty>("enable", 10, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvDisable, this, _1);
    sub_disable_ = create_subscription<std_msgs::msg::Empty>("disable", 10, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvCAN, this, _1);
    sub_can_ = create_subscription<can_msgs::msg::Frame>("can_rx", 100, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvBrakeCmd, this, _1);
    sub_brake_ = create_subscription<dbw_ford_msgs::msg::BrakeCmd>("brake_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvThrottleCmd, this, _1);
    sub_throttle_ = create_subscription<dbw_ford_msgs::msg::ThrottleCmd>("throttle_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvSteeringCmd, this, _1);
    sub_steering_ = create_subscription<dbw_ford_msgs::msg::SteeringCmd>("steering_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvGearCmd, this, _1);
    sub_gear_ = create_subscription<dbw_ford_msgs::msg::GearCmd>("gear_cmd", 1, bind);
  }
  {
    auto bind = std::bind(&DbwNode::recvMiscCmd, this, _1);
    sub_misc_ = create_subscription<dbw_ford_msgs::msg::MiscCmd>("misc_cmd", 1, bind);
  }

  // Setup Timer
  {
    auto bind = std::bind(&DbwNode::timerCallback, this);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), bind);
  }
}

void DbwNode::recvEnable(const std_msgs::msg::Empty::ConstSharedPtr) {
  enableSystem();
}

void DbwNode::recvDisable(const std_msgs::msg::Empty::ConstSharedPtr) {
  disableSystem();
}

void DbwNode::recvCAN(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  sync_imu_.processMsg(msg);
  sync_gps_.processMsg(msg);
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    switch (msg->id) {
      case ID_BRAKE_REPORT:
        if (msg->dlc >= sizeof(MsgBrakeReport)) {
          const MsgBrakeReport *ptr = reinterpret_cast<const MsgBrakeReport *>(msg->data.data());
          faultBrakes(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC, ptr->WDCBRK);
          dbw_ford_msgs::msg::BrakeReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->BTYPE == 0 || firmware_.hasValid(M_BPEC)) {
            // Brake pedal emulator for hybrid electric vehicles
            out.pedal_input  = (float)ptr->PI / UINT16_MAX;
            out.pedal_cmd    = (float)ptr->PC / UINT16_MAX;
            out.pedal_output = (float)ptr->PO / UINT16_MAX;
            out.torque_input  = brakeTorqueFromPedal(out.pedal_input);
            out.torque_cmd    = brakeTorqueFromPedal(out.pedal_cmd);
            out.torque_output = brakeTorqueFromPedal(out.pedal_output);
          } else if (ptr->BTYPE == 1 || firmware_.hasValid(M_ABS)) {
            // ACC/AEB braking for non-hybrid vehicles
            out.torque_input = ptr->PI;
            out.decel_cmd    = ptr->PC * 1e-3f;
            out.decel_output = ptr->PO * 1e-3f;
          } else if (ptr->BTYPE == 2) {
            // Brake pedal actuator for vehicles without brake-by-wire
            out.torque_input = ptr->PI;
            out.torque_cmd = ptr->PC;
            out.torque_output = ptr->PO;
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Unsupported brake report type: %u", ptr->BTYPE);
          }
          out.boo_cmd = ptr->BC ? true : false;
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.watchdog_braking = ptr->WDCBRK ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          timeoutBrake(ptr->TMOUT, ptr->ENABLED);
          overrideBrake(ptr->OVERRIDE, out.timeout);
          pub_brake_->publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Brake fault.    FLT1: %s FLT2: %s FLTPWR: %s",
                                 ptr->FLT1 ? "true, " : "false,",
                                 ptr->FLT2 ? "true, " : "false,",
                                 ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_THROTTLE_REPORT:
        if (msg->dlc >= sizeof(MsgThrottleReport)) {
          const MsgThrottleReport *ptr = reinterpret_cast<const MsgThrottleReport *>(msg->data.data());
          faultThrottle(ptr->FLT1 || ptr->FLT2);
          faultWatchdog(ptr->FLTWDC, ptr->WDCSRC);
          dbw_ford_msgs::msg::ThrottleReport out;
          out.header.stamp = msg->header.stamp;
          out.pedal_input  = (float)ptr->PI / UINT16_MAX;
          out.pedal_cmd    = (float)ptr->PC / UINT16_MAX;
          out.pedal_output = (float)ptr->PO / UINT16_MAX;
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.driver = ptr->DRIVER ? true : false;
          out.watchdog_counter.source = ptr->WDCSRC;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_ch1 = ptr->FLT1 ? true : false;
          out.fault_ch2 = ptr->FLT2 ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          timeoutThrottle(ptr->TMOUT, ptr->ENABLED);
          overrideThrottle(ptr->OVERRIDE, out.timeout);
          pub_throttle_->publish(out);
          if (ptr->FLT1 || ptr->FLT2 || ptr->FLTPWR) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Throttle fault. FLT1: %s FLT2: %s FLTPWR: %s",
                                 ptr->FLT1 ? "true, " : "false,",
                                 ptr->FLT2 ? "true, " : "false,",
                                 ptr->FLTPWR ? "true" : "false");
          }
        }
        break;

      case ID_STEERING_REPORT:
        if (msg->dlc >= sizeof(MsgSteeringReport)) {
          const MsgSteeringReport *ptr = reinterpret_cast<const MsgSteeringReport *>(msg->data.data());
          faultSteering(ptr->FLTBUS1 || ptr->FLTBUS2);
          faultSteeringCal(ptr->FLTCAL && (uint16_t)ptr->ANGLE == 0x8000);
          faultWatchdog(ptr->FLTWDC);
          dbw_ford_msgs::msg::SteeringReport out;
          out.header.stamp = msg->header.stamp;
          if ((uint16_t)ptr->ANGLE == 0x8000) {
            out.steering_wheel_angle = NAN;
          } else {
            out.steering_wheel_angle = (float)ptr->ANGLE * (float)(0.1 * M_PI / 180);
          }
          out.steering_wheel_cmd_type = ptr->TMODE ? dbw_ford_msgs::msg::SteeringReport::CMD_TORQUE
                                                   : dbw_ford_msgs::msg::SteeringReport::CMD_ANGLE;
          if ((uint16_t)ptr->CMD == 0xC000) {
            out.steering_wheel_cmd = NAN;
          } else if (out.steering_wheel_cmd_type == dbw_ford_msgs::msg::SteeringReport::CMD_ANGLE) {
            out.steering_wheel_cmd = (float)ptr->CMD * (float)(0.1 * M_PI / 180);
          } else {
            out.steering_wheel_cmd = (float)ptr->CMD / 128.0f;
          }
          if ((uint8_t)ptr->TORQUE == 0x80) {
            out.steering_wheel_torque = NAN;
          } else {
            out.steering_wheel_torque = (float)ptr->TORQUE * (float)0.0625;
          }
          if ((uint16_t)ptr->VEH_VEL == 0x8000) {
            out.speed = NAN;
          } else {
            out.speed = (float)ptr->VEH_VEL * (float)(0.01 / 3.6);
          }
          out.enabled = ptr->ENABLED ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_wdc = ptr->FLTWDC ? true : false;
          out.fault_bus1 = ptr->FLTBUS1 ? true : false;
          out.fault_bus2 = ptr->FLTBUS2 ? true : false;
          out.fault_calibration = ptr->FLTCAL ? true : false;
          out.fault_power = ptr->FLTPWR ? true : false;
          out.timeout = ptr->TMOUT ? true : false;
          timeoutSteering(ptr->TMOUT, ptr->ENABLED);
          overrideSteering(ptr->OVERRIDE, out.timeout);
          pub_steering_->publish(out);
          geometry_msgs::msg::TwistStamped twist;
          twist.header.stamp = out.header.stamp;
          twist.header.frame_id = frame_id_;
          twist.twist.linear.x = out.speed;
          twist.twist.angular.z = out.speed * tan(out.steering_wheel_angle / steering_ratio_) / acker_wheelbase_;
          pub_twist_->publish(twist);
          if (enable_joint_states_) {
            publishJointStates(msg->header.stamp, NULL, &out);
          }
          if (ptr->FLTBUS1 || ptr->FLTBUS2 || ptr->FLTPWR) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Steering fault. FLT1: %s FLT2: %s FLTPWR: %s",
                                 ptr->FLTBUS1 ? "true, " : "false,",
                                 ptr->FLTBUS2 ? "true, " : "false,",
                                 ptr->FLTPWR ? "true" : "false");
          } else if (ptr->FLTCAL && (uint16_t)ptr->ANGLE == 0x8000) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3,
                "Steering calibration fault. Drive at least 25 mph for at least 10 seconds in a straight line.");
          } else if (ptr->FLTCAL) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3,
                "Steering configuration fault. Contact support@dataspeedinc.com if not resolved in a few minutes.");
          }
        }
        break;

      case ID_GEAR_REPORT:
        if (msg->dlc >= 1) {
          const MsgGearReport *ptr = reinterpret_cast<const MsgGearReport *>(msg->data.data());
          overrideGear(ptr->OVERRIDE);
          dbw_ford_msgs::msg::GearReport out;
          out.header.stamp = msg->header.stamp;
          out.state.gear = ptr->STATE;
          out.cmd.gear = ptr->CMD;
          out.ready = ptr->READY ? true : false;
          out.override = ptr->OVERRIDE ? true : false;
          out.fault_bus = ptr->FLTBUS ? true : false;
          if (msg->dlc >= sizeof(MsgGearReport)) {
            out.reject.value = ptr->REJECT;
            if (out.reject.value == dbw_ford_msgs::msg::GearReject::NONE) {
              gear_warned_ = false;
            } else if (!gear_warned_) {
              gear_warned_ = true;
              switch (out.reject.value) {
                case dbw_ford_msgs::msg::GearReject::SHIFT_IN_PROGRESS:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Shift in progress");
                  break;
                case dbw_ford_msgs::msg::GearReject::OVERRIDE:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Override on brake, throttle, or steering");
                  break;
                case dbw_ford_msgs::msg::GearReject::ROTARY_LOW:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Rotary shifter can't shift to Low");
                  break;
                case dbw_ford_msgs::msg::GearReject::ROTARY_PARK:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Rotary shifter can't shift out of Park");
                  break;
                case dbw_ford_msgs::msg::GearReject::VEHICLE:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Rejected by vehicle, try pressing the brakes");
                  break;
                case dbw_ford_msgs::msg::GearReject::UNSUPPORTED:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: Unsupported gear command");
                  break;
                case dbw_ford_msgs::msg::GearReject::FAULT:
                  RCLCPP_WARN(get_logger(), "Gear shift rejected: System in fault state");
                  break;
              }
            }
          }
          pub_gear_->publish(out);
        }
        break;

      case ID_MISC_REPORT:
        if (msg->dlc >= 3) {
          const MsgMiscReport *ptr = reinterpret_cast<const MsgMiscReport *>(msg->data.data());
          if (buttons_) {
            if (ptr->btn_cc_gap_inc || ptr->btn_cc_cncl) {
              buttonCancel();
            } else if ((ptr->btn_cc_set_dec && ptr->btn_cc_gap_dec)
                    || (ptr->btn_cc_set_inc && ptr->btn_cc_off)
                    || (ptr->btn_cc_set_dec && ptr->btn_cc_res)) {
              enableSystem();
            }
          }
          dbw_ford_msgs::msg::Misc1Report out;
          out.header.stamp = msg->header.stamp;
          out.turn_signal.value = ptr->turn_signal;
          out.high_beam_headlights = ptr->head_light_hi ? true : false;
          out.wiper.status = ptr->wiper_front;
          out.ambient_light.status = ptr->light_ambient;
          out.btn_cc_on = ptr->btn_cc_on ? true : false;
          out.btn_cc_off = ptr->btn_cc_off ? true : false;
          out.btn_cc_on_off = ptr->btn_cc_on_off ? true : false;
          out.btn_cc_res = ptr->btn_cc_res ? true : false;
          out.btn_cc_cncl = ptr->btn_cc_cncl ? true : false;
          out.btn_cc_res_cncl = ptr->btn_cc_res_cncl ? true : false;
          out.btn_cc_res_inc = ptr->btn_cc_res_inc ? true : false;
          out.btn_cc_res_dec = ptr->btn_cc_res_dec ? true : false;
          out.btn_cc_set_inc = ptr->btn_cc_set_inc ? true : false;
          out.btn_cc_set_dec = ptr->btn_cc_set_dec ? true : false;
          out.btn_cc_gap_inc = ptr->btn_cc_gap_inc ? true : false;
          out.btn_cc_gap_dec = ptr->btn_cc_gap_dec ? true : false;
          out.btn_la_on_off = ptr->btn_la_on_off ? true : false;
          out.fault_bus = ptr->FLTBUS ? true : false;
          if (msg->dlc >= 5) {
            out.door_driver = ptr->door_driver ? true : false;
            out.door_passenger = ptr->door_passenger ? true : false;
            out.door_rear_left = ptr->door_rear_left ? true : false;
            out.door_rear_right = ptr->door_rear_right ? true : false;
            out.door_hood = ptr->door_hood ? true : false;
            out.door_trunk = ptr->door_trunk ? true : false;
            out.passenger_detect = ptr->pasngr_detect ? true : false;
            out.passenger_airbag = ptr->pasngr_airbag ? true : false;
            out.buckle_driver = ptr->buckle_driver ? true : false;
            out.buckle_passenger = ptr->buckle_pasngr ? true : false;
            out.btn_ld_ok = ptr->btn_ld_ok ? true : false;
            out.btn_ld_up = ptr->btn_ld_up ? true : false;
            out.btn_ld_down = ptr->btn_ld_down ? true : false;
            out.btn_ld_left = ptr->btn_ld_left ? true : false;
            out.btn_ld_right = ptr->btn_ld_right ? true : false;
          }
          if (msg->dlc >= 8) {
            out.btn_rd_ok = ptr->btn_rd_ok ? true : false;
            out.btn_rd_up = ptr->btn_rd_up ? true : false;
            out.btn_rd_down = ptr->btn_rd_down ? true : false;
            out.btn_rd_left = ptr->btn_rd_left ? true : false;
            out.btn_rd_right = ptr->btn_rd_right ? true : false;
            out.btn_vol_inc = ptr->btn_vol_inc ? true : false;
            out.btn_vol_dec = ptr->btn_vol_dec ? true : false;
            out.btn_mute = ptr->btn_mute ? true : false;
            out.btn_media = ptr->btn_media ? true : false;
            out.btn_prev = ptr->btn_prev ? true : false;
            out.btn_next = ptr->btn_next ? true : false;
            out.btn_speak = ptr->btn_speak ? true : false;
            out.btn_call_start = ptr->btn_call_start ? true : false;
            out.btn_call_end = ptr->btn_call_end ? true : false;
          }
          if ((msg->dlc >= 8) && (ptr->outside_air_temp < 0xFE)) {
            out.outside_temperature = ((float)ptr->outside_air_temp * 0.5f) - 40.0f;
          } else {
            out.outside_temperature = NAN;
          }
          pub_misc_1_->publish(out);
        }
        break;

      case ID_REPORT_WHEEL_SPEED:
        if (msg->dlc >= sizeof(MsgReportWheelSpeed)) {
          const MsgReportWheelSpeed *ptr = reinterpret_cast<const MsgReportWheelSpeed *>(msg->data.data());
          dbw_ford_msgs::msg::WheelSpeedReport out;
          out.header.stamp = msg->header.stamp;
          if ((uint16_t)ptr->front_left == 0x8000) {
            out.front_left = NAN;
          } else {
            out.front_left = (float)ptr->front_left * 0.01f;
          }
          if ((uint16_t)ptr->front_right == 0x8000) {
            out.front_right = NAN;
          } else {
            out.front_right = (float)ptr->front_right * 0.01f;
          }
          if ((uint16_t)ptr->rear_left == 0x8000) {
            out.rear_left = NAN;
          } else {
            out.rear_left = (float)ptr->rear_left * 0.01f;
          }
          if ((uint16_t)ptr->rear_right == 0x8000) {
            out.rear_right = NAN;
          } else {
            out.rear_right = (float)ptr->rear_right * 0.01f;
          }
          pub_wheel_speeds_->publish(out);
          if (enable_joint_states_) {
            publishJointStates(msg->header.stamp, &out, NULL);
          }
        }
        break;

      case ID_REPORT_WHEEL_POSITION:
        if (msg->dlc >= sizeof(MsgReportWheelPosition)) {
          const MsgReportWheelPosition *ptr = reinterpret_cast<const MsgReportWheelPosition *>(msg->data.data());
          dbw_ford_msgs::msg::WheelPositionReport out;
          out.header.stamp = msg->header.stamp;
          out.front_left  = ptr->front_left;
          out.front_right = ptr->front_right;
          out.rear_left   = ptr->rear_left;
          out.rear_right  = ptr->rear_right;
          pub_wheel_positions_->publish(out);
        }
        break;

      case ID_REPORT_TIRE_PRESSURE:
        if (msg->dlc >= sizeof(MsgReportTirePressure)) {
          const MsgReportTirePressure *ptr = reinterpret_cast<const MsgReportTirePressure *>(msg->data.data());
          dbw_ford_msgs::msg::TirePressureReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->front_left == 0xFFFF) {
            out.front_left = NAN;
          } else {
            out.front_left = (float)ptr->front_left;
          }
          if (ptr->front_right == 0xFFFF) {
            out.front_right = NAN;
          } else {
            out.front_right = (float)ptr->front_right;
          }
          if (ptr->rear_left == 0xFFFF) {
            out.rear_left = NAN;
          } else {
            out.rear_left = (float)ptr->rear_left;
          }
          if (ptr->rear_right == 0xFFFF) {
            out.rear_right = NAN;
          } else {
            out.rear_right = (float)ptr->rear_right;
          }
          pub_tire_pressure_->publish(out);
        }
        break;

      case ID_REPORT_FUEL_LEVEL:
        if (msg->dlc >= 2) {
          const MsgReportFuelLevel *ptr = reinterpret_cast<const MsgReportFuelLevel *>(msg->data.data());
          dbw_ford_msgs::msg::FuelLevelReport out;
          out.header.stamp = msg->header.stamp;
          out.fuel_level = (float)ptr->fuel_level * 0.108696f;
          if (msg->dlc >= sizeof(MsgReportFuelLevel)) {
            out.battery_12v = (float)ptr->battery_12v * 0.0625f;
            out.battery_hev = (float)ptr->battery_hev * 0.5f;
            out.odometer = (float)ptr->odometer * 0.1f;
          }
          pub_fuel_level_->publish(out);
        }
        break;

      case ID_REPORT_SURROUND:
        if (msg->dlc >= sizeof(MsgReportSurround)) {
          const MsgReportSurround *ptr = reinterpret_cast<const MsgReportSurround *>(msg->data.data());
          dbw_ford_msgs::msg::SurroundReport out;
          out.header.stamp = msg->header.stamp;
          out.cta_left_alert = ptr->l_cta_alert ? true : false;
          out.cta_right_alert = ptr->r_cta_alert ? true : false;
          out.cta_left_enabled = ptr->l_cta_enabled ? true : false;
          out.cta_right_enabled = ptr->r_cta_enabled ? true : false;
          out.blis_left_alert = ptr->l_blis_alert ? true : false;
          out.blis_right_alert = ptr->r_blis_alert ? true : false;
          out.blis_left_enabled = ptr->l_blis_enabled ? true : false;
          out.blis_right_enabled = ptr->r_blis_enabled ? true : false;
          out.sonar_enabled = ptr->sonar_enabled ? true : false;
          out.sonar_fault = ptr->sonar_fault ? true : false;
          if (out.sonar_enabled) {
            out.sonar[0] = sonarMetersFromBits(ptr->sonar_00);
            out.sonar[1] = sonarMetersFromBits(ptr->sonar_01);
            out.sonar[2] = sonarMetersFromBits(ptr->sonar_02);
            out.sonar[3] = sonarMetersFromBits(ptr->sonar_03);
            out.sonar[4] = sonarMetersFromBits(ptr->sonar_04);
            out.sonar[5] = sonarMetersFromBits(ptr->sonar_05);
            out.sonar[6] = sonarMetersFromBits(ptr->sonar_06);
            out.sonar[7] = sonarMetersFromBits(ptr->sonar_07);
            out.sonar[8] = sonarMetersFromBits(ptr->sonar_08);
            out.sonar[9] = sonarMetersFromBits(ptr->sonar_09);
            out.sonar[10] = sonarMetersFromBits(ptr->sonar_10);
            out.sonar[11] = sonarMetersFromBits(ptr->sonar_11);
          }
          pub_surround_->publish(out);
          sensor_msgs::msg::PointCloud2 cloud;
          sonarBuildPointCloud2(cloud, out);
          pub_sonar_cloud_->publish(cloud);
        }
        break;

      case ID_REPORT_BRAKE_INFO:
        if (msg->dlc >= sizeof(MsgReportBrakeInfo)) {
          const MsgReportBrakeInfo *ptr = reinterpret_cast<const MsgReportBrakeInfo *>(msg->data.data());
          dbw_ford_msgs::msg::BrakeInfoReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->brake_torque_request == 0xFFF) {
            out.brake_torque_request = NAN;
          } else {
            out.brake_torque_request = (float)ptr->brake_torque_request * 4.0f;
          }
          if (ptr->brake_torque_actual == 0xFFF) {
            out.brake_torque_actual = NAN;
          } else {
            out.brake_torque_actual = (float)ptr->brake_torque_actual * 4.0f;
          }
          if ((uint16_t)ptr->wheel_torque == 0xE000) {
            out.wheel_torque_actual = NAN;
          } else {
            out.wheel_torque_actual = (float)ptr->wheel_torque * 4.0f;
          }
          if ((uint16_t)ptr->accel_over_ground_est == 0xE00) {
            out.accel_over_ground = NAN;
          } else {
            out.accel_over_ground = (float)ptr->accel_over_ground_est * 0.035f;
          }
          out.brake_pedal_qf.value = ptr->bped_qf;
          out.hsa.status = ptr->hsa_stat;
          out.hsa.mode = ptr->hsa_mode;
          out.abs_active = ptr->abs_active ? true : false;
          out.abs_enabled = ptr->abs_enabled ? true : false;
          out.stab_active = ptr->stab_active ? true : false;
          out.stab_enabled = ptr->stab_enabled ? true : false;
          out.trac_active = ptr->trac_active ? true : false;
          out.trac_enabled = ptr->trac_enabled ? true : false;
          out.parking_brake.status = ptr->parking_brake;
          out.stationary = ptr->stationary;
          pub_brake_info_->publish(out);
          if (ptr->bped_qf != dbw_ford_msgs::msg::QualityFactor::OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Brake pedal limp-home: %u", ptr->bped_qf);
          }
        }
        break;

      case ID_REPORT_THROTTLE_INFO:
        if (msg->dlc >= sizeof(MsgReportThrottleInfo)) {
          const MsgReportThrottleInfo *ptr = reinterpret_cast<const MsgReportThrottleInfo *>(msg->data.data());
          dbw_ford_msgs::msg::ThrottleInfoReport out;
          out.header.stamp = msg->header.stamp;
          if (ptr->throttle_pc == 0x3FF) {
            out.throttle_pc = NAN;
          } else {
            out.throttle_pc = (float)ptr->throttle_pc * 1e-3f;
          }
          if ((uint8_t)ptr->throttle_rate == 0x80) {
            out.throttle_rate = NAN;
          } else {
            out.throttle_rate = (float)ptr->throttle_rate * 4e-4f;
          }
          out.throttle_pedal_qf.value = ptr->aped_qf;
          if (ptr->engine_rpm == 0xFFFF) {
            out.engine_rpm = NAN;
          } else {
            out.engine_rpm = (float)ptr->engine_rpm * 0.25f;
          }
          out.gear_num.num = ptr->gear_num;
          out.ignition.value = ptr->ign_stat;
          if ((uint16_t)ptr->batt_curr == 0xE000) {
            out.batt_curr = NAN;
          } else {
            out.batt_curr = (float)ptr->batt_curr * 0.0625f;
          }
          pub_throttle_info_->publish(out);
          if (ptr->aped_qf != dbw_ford_msgs::msg::QualityFactor::OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Throttle pedal limp-home: %u", ptr->aped_qf);
          }
        }
        break;

      case ID_REPORT_DRIVER_ASSIST:
        if (msg->dlc >= sizeof(MsgReportDriverAssist)) {
          const MsgReportDriverAssist *ptr = reinterpret_cast<const MsgReportDriverAssist *>(msg->data.data());
          dbw_ford_msgs::msg::DriverAssistReport out;
          out.header.stamp = msg->header.stamp;
          out.decel = (float)ptr->decel * 0.0625f;
          out.decel_src = ptr->decel_src;
          out.fcw_enabled = ptr->fcw_enabled;
          out.fcw_active = ptr->fcw_active;
          out.aeb_enabled = ptr->aeb_enabled;
          out.aeb_precharge = ptr->aeb_precharge;
          out.aeb_braking = ptr->aeb_braking;
          out.acc_enabled = ptr->acc_enabled;
          out.acc_braking = ptr->acc_braking;
          pub_driver_assist_->publish(out);
          if (out.fcw_active) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Forward collision warning activated!");
          }
          if (out.aeb_braking) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5e3, "Automatic emergency braking activated!");
          }
        }
        break;

      case ID_LICENSE:
        if (msg->dlc >= sizeof(MsgLicense)) {
          const MsgLicense *ptr = reinterpret_cast<const MsgLicense *>(msg->data.data());
          const Module module = ptr->module ? (Module)ptr->module : M_STEER; // Legacy steering firmware reports zero for module
          const char *str_m = moduleToString(module);
          RCLCPP_DEBUG(get_logger(), "LICENSE(%x,%02X,%s)", ptr->module, ptr->mux, str_m);
          if (ptr->ready) {
            DS_WARN_ONCE_ID(get_logger(), module, "Licensing: %s ready", str_m);
            // ROS_INFO_ONCE_ID(module, "Licensing: %s ready", str_m);
            if (ptr->trial) {
              DS_WARN_ONCE_ID(
                  get_logger(), module,
                  "Licensing: %s one or more features licensed as a counted trial. Visit "
                  "https://www.dataspeedinc.com/products/maintenance-subscription/ to request a full license.",
                  str_m);
            }
            if (ptr->expired) {
              DS_WARN_ONCE_ID(get_logger(), module,
                              "Licensing: %s one or more feature licenses expired due to the firmware build date",
                              str_m);
            }
          } else if (module == M_STEER) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10e3, "Licensing: Waiting for VIN...");
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10e3, "Licensing: Waiting for required info...");
          }
          if (ptr->mux == LIC_MUX_LDATE0) {
            if (ldate_.size() == 0) {
              ldate_.push_back(ptr->ldate0.ldate0);
              ldate_.push_back(ptr->ldate0.ldate1);
              ldate_.push_back(ptr->ldate0.ldate2);
              ldate_.push_back(ptr->ldate0.ldate3);
              ldate_.push_back(ptr->ldate0.ldate4);
              ldate_.push_back(ptr->ldate0.ldate5);
            }
          } else if (ptr->mux == LIC_MUX_LDATE1) {
            if (ldate_.size() == 6) {
              ldate_.push_back(ptr->ldate1.ldate6);
              ldate_.push_back(ptr->ldate1.ldate7);
              ldate_.push_back(ptr->ldate1.ldate8);
              ldate_.push_back(ptr->ldate1.ldate9);
              RCLCPP_INFO(get_logger(), "Licensing: %s license string date: %s", str_m, ldate_.c_str());
            }
          } else if (ptr->mux == LIC_MUX_MAC) {
            RCLCPP_INFO_ONCE(get_logger(), "Licensing: %s MAC: %02X:%02X:%02X:%02X:%02X:%02X", str_m,
                             ptr->mac.addr0, ptr->mac.addr1,
                             ptr->mac.addr2, ptr->mac.addr3,
                             ptr->mac.addr4, ptr->mac.addr5);
          } else if (ptr->mux == LIC_MUX_BDATE0) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 0) {
              bdate.push_back(ptr->bdate0.date0);
              bdate.push_back(ptr->bdate0.date1);
              bdate.push_back(ptr->bdate0.date2);
              bdate.push_back(ptr->bdate0.date3);
              bdate.push_back(ptr->bdate0.date4);
              bdate.push_back(ptr->bdate0.date5);
            }
          } else if (ptr->mux == LIC_MUX_BDATE1) {
            std::string &bdate = bdate_[module];
            if (bdate.size() == 6) {
              bdate.push_back(ptr->bdate1.date6);
              bdate.push_back(ptr->bdate1.date7);
              bdate.push_back(ptr->bdate1.date8);
              bdate.push_back(ptr->bdate1.date9);
              RCLCPP_INFO(get_logger(), "Licensing: %s firmware build date: %s", str_m, bdate.c_str());
            }
          } else if (ptr->mux == LIC_MUX_VIN0) {
            if (vin_.size() == 0) {
              vin_.push_back(ptr->vin0.vin00);
              vin_.push_back(ptr->vin0.vin01);
              vin_.push_back(ptr->vin0.vin02);
              vin_.push_back(ptr->vin0.vin03);
              vin_.push_back(ptr->vin0.vin04);
              vin_.push_back(ptr->vin0.vin05);
            }
          } else if (ptr->mux == LIC_MUX_VIN1) {
            if (vin_.size() == 6) {
              vin_.push_back(ptr->vin1.vin06);
              vin_.push_back(ptr->vin1.vin07);
              vin_.push_back(ptr->vin1.vin08);
              vin_.push_back(ptr->vin1.vin09);
              vin_.push_back(ptr->vin1.vin10);
              vin_.push_back(ptr->vin1.vin11);
            }
          } else if (ptr->mux == LIC_MUX_VIN2) {
            if (vin_.size() == 12) {
              vin_.push_back(ptr->vin2.vin12);
              vin_.push_back(ptr->vin2.vin13);
              vin_.push_back(ptr->vin2.vin14);
              vin_.push_back(ptr->vin2.vin15);
              vin_.push_back(ptr->vin2.vin16);
              std_msgs::msg::String msg;
              msg.data = vin_;
              pub_vin_->publish(msg);
              RCLCPP_INFO(get_logger(), "Licensing: VIN: %s", vin_.c_str());
            }
          } else if ((LIC_MUX_F0 <= ptr->mux) && (ptr->mux <= LIC_MUX_F7)) {
            constexpr std::array<const char *, 8> NAME = {"BASE", "CONTROL", "SENSORS", "REMOTE", "", "", "", ""};
            constexpr std::array<bool, 8> WARN = {true, true, true, false, true, true, true, true};
            const size_t i = ptr->mux - LIC_MUX_F0;
            const int id = module * NAME.size() + i;
            const std::string name = strcmp(NAME[i], "") ? NAME[i] : std::string(1, '0' + i);
            if (ptr->license.enabled) {
              DS_INFO_ONCE_ID(get_logger(), id, "Licensing: %s feature '%s' enabled%s", str_m, name.c_str(),
                              ptr->license.trial ? " as a counted trial" : "");
            } else if (ptr->ready && !WARN[i]) {
              DS_INFO_ONCE_ID(get_logger(), id, "Licensing: %s feature '%s' not licensed.", str_m, name.c_str());
            } else if (ptr->ready) {
              DS_WARN_ONCE_ID(get_logger(), id,
                              "Licensing: %s feature '%s' not licensed. Visit "
                              "https://www.dataspeedinc.com/products/maintenance-subscription/ to request a license.",
                              str_m, name.c_str());
            }
            if (ptr->ready && (module == M_STEER) && (ptr->license.trial || (!ptr->license.enabled && WARN[i]))) {
              RCLCPP_INFO_ONCE(get_logger(), "Licensing: Feature '%s' trials used: %u, remaining: %u", name.c_str(),
                               ptr->license.trials_used, ptr->license.trials_left);
            }
          }
        }
        break;

      case ID_VERSION:
        if (msg->dlc >= sizeof(MsgVersion)) {
          const MsgVersion *ptr = reinterpret_cast<const MsgVersion *>(msg->data.data());
          const PlatformVersion version((Platform)ptr->platform, (Module)ptr->module, ptr->major, ptr->minor, ptr->build);
          const ModuleVersion latest = FIRMWARE_LATEST.get(version);
          const char *str_p = platformToString(version.p);
          const char *str_m = moduleToString(version.m);
          if (firmware_.get(version) != version.v) {
            firmware_.put(version);
            if (latest.valid()) {
              RCLCPP_INFO(get_logger(), "Detected %s %s firmware version %u.%u.%u", str_p, str_m, ptr->major, ptr->minor, ptr->build);
            } else {
              RCLCPP_WARN(
                  get_logger(),
                  "Detected %s %s firmware version %u.%u.%u, which is unsupported. Platform: 0x%02X, Module: %u", str_p,
                  str_m, ptr->major, ptr->minor, ptr->build, ptr->platform, ptr->module);
            }
            if (version < latest) {
              RCLCPP_WARN(get_logger(), "Firmware %s %s has old  version %u.%u.%u, updating to %u.%u.%u is suggested.",
                          str_p, str_m, version.v.major(), version.v.minor(), version.v.build(), latest.major(),
                          latest.minor(), latest.build());
            }
          }
        }
        break;

      case ID_BRAKE_CMD:
        RCLCPP_WARN_EXPRESSION(
            get_logger(),
            warn_cmds_ && !(reinterpret_cast<const MsgBrakeCmd *>(msg->data.data()))->RES1
                       && !(reinterpret_cast<const MsgBrakeCmd *>(msg->data.data()))->RES2,
            "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Brake. Id: 0x%03X",
            ID_BRAKE_CMD);
        break;
      case ID_THROTTLE_CMD:
        RCLCPP_WARN_EXPRESSION(
            get_logger(),
            warn_cmds_ && !(reinterpret_cast<const MsgThrottleCmd *>(msg->data.data()))->RES1
                       && !(reinterpret_cast<const MsgThrottleCmd *>(msg->data.data()))->RES2,
            "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Throttle. Id: 0x%03X",
            ID_THROTTLE_CMD);
        break;
      case ID_STEERING_CMD:
        RCLCPP_WARN_EXPRESSION(
            get_logger(),
            warn_cmds_ && !(reinterpret_cast<const MsgSteeringCmd *>(msg->data.data()))->RES1
                       && !(reinterpret_cast<const MsgSteeringCmd *>(msg->data.data()))->RES2,
            "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Steering. Id: 0x%03X",
            ID_STEERING_CMD);
        break;
      case ID_GEAR_CMD:
        RCLCPP_WARN_EXPRESSION(
            get_logger(),
            warn_cmds_ && !(reinterpret_cast<const MsgGearCmd *>(msg->data.data()))->RES1
                       && !(reinterpret_cast<const MsgGearCmd *>(msg->data.data()))->RES2,
            "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Shifting. Id: 0x%03X",
            ID_GEAR_CMD);
        break;
      case ID_MISC_CMD:
        RCLCPP_WARN_EXPRESSION(
            get_logger(), warn_cmds_,
            "DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Turn Signals. Id: 0x%03X",
            ID_MISC_CMD);
        break;
    }
  }
#if 0
  RCLCPP_INFO(get_logger(), "ena: %s, clr: %s, brake: %s, throttle: %s, steering: %s, gear: %s",
           enabled() ? "true " : "false",
           clear() ? "true " : "false",
           override_brake_ ? "true " : "false",
           override_throttle_ ? "true " : "false",
           override_steering_ ? "true " : "false",
           override_gear_ ? "true " : "false"
       );
#endif
}

void DbwNode::recvCanImu(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs) {
  assert(msgs.size() == 2);
  assert(msgs[0]->id == ID_REPORT_ACCEL);
  assert(msgs[1]->id == ID_REPORT_GYRO);
  if ((msgs[0]->dlc >= sizeof(MsgReportAccel))
   && (msgs[1]->dlc >= sizeof(MsgReportGyro))) {
    const MsgReportAccel *ptr_accel = reinterpret_cast<const MsgReportAccel *>(msgs[0]->data.data());
    const MsgReportGyro *ptr_gyro = reinterpret_cast<const MsgReportGyro *>(msgs[1]->data.data());
    sensor_msgs::msg::Imu out;
    out.header.stamp = msgs[0]->header.stamp;
    out.header.frame_id = frame_id_;
    out.orientation_covariance[0] = -1; // Orientation not present
    if ((uint16_t)ptr_accel->accel_long == 0x8000) {
      out.linear_acceleration.x = NAN;
    } else {
      out.linear_acceleration.x = (double)ptr_accel->accel_long * 0.01;
    }
    if ((uint16_t)ptr_accel->accel_lat == 0x8000) {
      out.linear_acceleration.y = NAN;
    } else {
      out.linear_acceleration.y = (double)ptr_accel->accel_lat * -0.01;
    }
    if ((uint16_t)ptr_accel->accel_vert == 0x8000) {
      out.linear_acceleration.z = NAN;
    } else {
      out.linear_acceleration.z = (double)ptr_accel->accel_vert * -0.01;
    }
    if ((uint16_t)ptr_gyro->gyro_roll == 0x8000) {
      out.angular_velocity.x = NAN;
    } else {
      out.angular_velocity.x = (double)ptr_gyro->gyro_roll * 0.0002;
    }
    if ((uint16_t)ptr_gyro->gyro_yaw == 0x8000) {
      out.angular_velocity.z = NAN;
    } else {
      out.angular_velocity.z = (double)ptr_gyro->gyro_yaw * 0.0002;
    }
    pub_imu_->publish(out);
  }
#if 0
  RCLCPP_INFO(get_logger(), "Time: %u.%u, %u.%u, delta: %fms",
           msgs[0]->header.stamp.sec, msgs[0]->header.stamp.nsec,
           msgs[1]->header.stamp.sec, msgs[1]->header.stamp.nsec,
           labs((msgs[1]->header.stamp - msgs[0]->header.stamp).toNSec()) / 1000000.0);
#endif
}

void DbwNode::recvCanGps(const std::vector<can_msgs::msg::Frame::ConstSharedPtr> &msgs) {
  assert(msgs.size() == 3);
  assert(msgs[0]->id == ID_REPORT_GPS1);
  assert(msgs[1]->id == ID_REPORT_GPS2);
  assert(msgs[2]->id == ID_REPORT_GPS3);
  if ((msgs[0]->dlc >= sizeof(MsgReportGps1))
   && (msgs[1]->dlc >= sizeof(MsgReportGps2))
   && (msgs[2]->dlc >= sizeof(MsgReportGps3))) {
    const MsgReportGps1 *ptr1 = reinterpret_cast<const MsgReportGps1 *>(msgs[0]->data.data());
    const MsgReportGps2 *ptr2 = reinterpret_cast<const MsgReportGps2 *>(msgs[1]->data.data());
    const MsgReportGps3 *ptr3 = reinterpret_cast<const MsgReportGps3 *>(msgs[2]->data.data());

    sensor_msgs::msg::NavSatFix msg_fix;
    msg_fix.header.stamp = msgs[0]->header.stamp;
    msg_fix.latitude = (double)ptr1->latitude / 3e6;
    msg_fix.longitude = (double)ptr1->longitude / 3e6;
    msg_fix.altitude = (double)ptr3->altitude * 0.25;
    msg_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    msg_fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    switch (ptr3->quality) {
      case 0:
      default:
        msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case 1:
      case 2:
        msg_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }
    pub_gps_fix_->publish(msg_fix);

    geometry_msgs::msg::TwistStamped msg_vel;
    msg_vel.header.stamp = msgs[0]->header.stamp;
    double heading = (double)ptr3->heading * (0.01 * M_PI / 180);
    double speed = (double)ptr3->speed * 0.44704;
    msg_vel.twist.linear.x = cos(heading) * speed;
    msg_vel.twist.linear.y = sin(heading) * speed;
    pub_gps_vel_->publish(msg_vel);

    sensor_msgs::msg::TimeReference msg_time;
    struct tm unix_time;
    unix_time.tm_year = ptr2->utc_year + 100; // [1900] <-- [2000]
    unix_time.tm_mon = ptr2->utc_month - 1;   // [0-11] <-- [1-12]
    unix_time.tm_mday = ptr2->utc_day;        // [1-31] <-- [1-31]
    unix_time.tm_hour = ptr2->utc_hours;      // [0-23] <-- [0-23]
    unix_time.tm_min = ptr2->utc_minutes;     // [0-59] <-- [0-59]
    unix_time.tm_sec = ptr2->utc_seconds;     // [0-59] <-- [0-59]
    msg_time.header.stamp = msgs[0]->header.stamp;
    msg_time.time_ref.sec = timegm(&unix_time);
    msg_time.time_ref.nanosec = 0;
    pub_gps_time_->publish(msg_time);

#if 0
    RCLCPP_INFO(get_logger(), "UTC Time: %04d-%02d-%02d %02d:%02d:%02d",
             2000 + ptr2->utc_year, ptr2->utc_month, ptr2->utc_day,
             ptr2->utc_hours, ptr2->utc_minutes, ptr2->utc_seconds);
#endif
  }
#if 0
  RCLCPP_INFO(get_logger(), "Time: %u.%u, %u.%u, %u.%u, delta: %fms",
           msgs[0]->header.stamp.sec, msgs[0]->header.stamp.nsec,
           msgs[1]->header.stamp.sec, msgs[1]->header.stamp.nsec,
           msgs[2]->header.stamp.sec, msgs[2]->header.stamp.nsec,
           std::max(std::max(
               labs((msgs[1]->header.stamp - msgs[0]->header.stamp).toNSec()),
               labs((msgs[2]->header.stamp - msgs[1]->header.stamp).toNSec())),
               labs((msgs[0]->header.stamp - msgs[2]->header.stamp).toNSec())) / 1000000.0);
#endif
}

void DbwNode::recvBrakeCmd(const dbw_ford_msgs::msg::BrakeCmd::ConstSharedPtr msg) {
  can_msgs::msg::Frame out;
  out.id = ID_BRAKE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgBrakeCmd);
  MsgBrakeCmd *ptr = reinterpret_cast<MsgBrakeCmd *>(out.data.data());
  memset(ptr, 0x00, sizeof(*ptr));
  bool fwd_abs = firmware_.hasValid(M_ABS); // Does the ABS braking module exist?
  bool fwd_bpe = true;
  bool fwd = !pedal_luts_; // Forward command type, or apply pedal LUTs locally
  fwd |= fwd_abs; // The local pedal LUTs are for the BPEC module, the ABS module requires forwarding
  fwd &= fwd_bpe; // Only modern BPEC firmware supports forwarding the command type
  switch (msg->pedal_cmd_type) {
    case dbw_ford_msgs::msg::BrakeCmd::CMD_NONE:
      break;
    case dbw_ford_msgs::msg::BrakeCmd::CMD_PEDAL:
      ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_PEDAL;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd * UINT16_MAX, 0, UINT16_MAX);
      if (!firmware_.hasValid(M_BPEC) && firmware_.hasValid(M_ABS)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Module ABS does not support brake command type PEDAL");
      }
      break;
    case dbw_ford_msgs::msg::BrakeCmd::CMD_PERCENT:
      if (fwd) {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_PERCENT;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd * UINT16_MAX, 0, UINT16_MAX);
      } else {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_PEDAL;
        ptr->PCMD = std::clamp<float>(brakePedalFromPercent(msg->pedal_cmd) * UINT16_MAX, 0, UINT16_MAX);
      }
      break;
    case dbw_ford_msgs::msg::BrakeCmd::CMD_TORQUE:
      if (fwd) {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_TORQUE;
        ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      } else {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_PEDAL;
        ptr->PCMD = std::clamp<float>(brakePedalFromTorque(msg->pedal_cmd) * UINT16_MAX, 0, UINT16_MAX);
      }
      if (!firmware_.hasValid(M_BPEC) && firmware_.hasValid(M_ABS)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Module ABS does not support brake command type TORQUE");
      }
      break;
    case dbw_ford_msgs::msg::BrakeCmd::CMD_TORQUE_RQ:
      // CMD_TORQUE_RQ must be forwarded, there is no local implementation
      ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_TORQUE_RQ;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd, 0, UINT16_MAX);
      if (!firmware_.hasValid(M_BPEC) && firmware_.hasValid(M_ABS)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Module ABS does not support brake command type TORQUE_RQ");
      }
      break;
    case dbw_ford_msgs::msg::BrakeCmd::CMD_DECEL:
      // CMD_DECEL must be forwarded, there is no local implementation
      ptr->CMD_TYPE = dbw_ford_msgs::msg::BrakeCmd::CMD_DECEL;
      ptr->PCMD = std::clamp<float>(msg->pedal_cmd * 1e3f, 0, 10e3);
      if (!firmware_.hasValid(M_ABS) && firmware_.hasValid(M_BPEC)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1e3, "Module BPEC does not support brake command type DECEL");
      }
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown brake command type: %u", msg->pedal_cmd_type);
      break;
  }
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_->publish(out);
}

void DbwNode::recvThrottleCmd(const dbw_ford_msgs::msg::ThrottleCmd::ConstSharedPtr msg) {
  can_msgs::msg::Frame out;
  out.id = ID_THROTTLE_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgThrottleCmd);
  MsgThrottleCmd *ptr = reinterpret_cast<MsgThrottleCmd *>(out.data.data());
  memset(ptr, 0x00, sizeof(*ptr));
  bool fwd = !pedal_luts_; // Forward command type, or apply pedal LUTs locally
  float cmd = 0.0;
  switch (msg->pedal_cmd_type) {
    case dbw_ford_msgs::msg::ThrottleCmd::CMD_NONE:
      break;
    case dbw_ford_msgs::msg::ThrottleCmd::CMD_PEDAL:
      ptr->CMD_TYPE = dbw_ford_msgs::msg::ThrottleCmd::CMD_PEDAL;
      cmd = msg->pedal_cmd;
      break;
    case dbw_ford_msgs::msg::ThrottleCmd::CMD_PERCENT:
      if (fwd) {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::ThrottleCmd::CMD_PERCENT;
        cmd = msg->pedal_cmd;
      } else {
        ptr->CMD_TYPE = dbw_ford_msgs::msg::ThrottleCmd::CMD_PEDAL;
        cmd = throttlePedalFromPercent(msg->pedal_cmd);
      }
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown throttle command type: %u", msg->pedal_cmd_type);
      break;
  }
  ptr->PCMD = std::clamp<float>(cmd * UINT16_MAX, 0, UINT16_MAX);
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_->publish(out);
}

void DbwNode::recvSteeringCmd(const dbw_ford_msgs::msg::SteeringCmd::ConstSharedPtr msg) {
  can_msgs::msg::Frame out;
  out.id = ID_STEERING_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgSteeringCmd);
  MsgSteeringCmd *ptr = reinterpret_cast<MsgSteeringCmd *>(out.data.data());
  memset(ptr, 0x00, sizeof(*ptr));
  switch (msg->cmd_type) {
    case dbw_ford_msgs::msg::SteeringCmd::CMD_ANGLE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_angle_cmd * (float)(180 / M_PI * 10), -INT16_MAX, INT16_MAX);
      if (fabsf(msg->steering_wheel_angle_velocity) > 0) {
        ptr->SVEL = std::clamp<float>(roundf(fabsf(msg->steering_wheel_angle_velocity) * (float)(180 / M_PI / 4)), 1, 254);
      }
      ptr->CMD_TYPE = dbw_ford_msgs::msg::SteeringCmd::CMD_ANGLE;
      break;
    case dbw_ford_msgs::msg::SteeringCmd::CMD_TORQUE:
      ptr->SCMD = std::clamp<float>(msg->steering_wheel_torque_cmd * 128, -INT16_MAX, INT16_MAX);
      ptr->CMD_TYPE = dbw_ford_msgs::msg::SteeringCmd::CMD_TORQUE;
      if (!firmware_.hasValid(M_EPS) && firmware_.hasValid(M_STEER)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Module STEER does not support steering command type TORQUE");
      }
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown steering command type: %u", msg->cmd_type);
      break;
  }
  if (enabled() && msg->enable) {
    ptr->EN = 1;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  if (msg->ignore) {
    ptr->IGNORE = 1;
  }
  if (msg->quiet) {
    ptr->QUIET = 1;
  }
  if (msg->alert) {
    ptr->ALERT = 1;
  }
  ptr->COUNT = msg->count;
  pub_can_->publish(out);
}

void DbwNode::recvGearCmd(const dbw_ford_msgs::msg::GearCmd::ConstSharedPtr msg) {
  can_msgs::msg::Frame out;
  out.id = ID_GEAR_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgGearCmd);
  MsgGearCmd *ptr = reinterpret_cast<MsgGearCmd *>(out.data.data());
  memset(ptr, 0x00, sizeof(*ptr));
  if (enabled()) {
    ptr->GCMD = msg->cmd.gear;
  }
  if (clear() || msg->clear) {
    ptr->CLEAR = 1;
  }
  pub_can_->publish(out);
}

void DbwNode::recvMiscCmd(const dbw_ford_msgs::msg::MiscCmd::ConstSharedPtr msg) {
  can_msgs::msg::Frame out;
  out.id = ID_MISC_CMD;
  out.is_extended = false;
  out.dlc = sizeof(MsgMiscCmd);
  MsgMiscCmd *ptr = reinterpret_cast<MsgMiscCmd *>(out.data.data());
  memset(ptr, 0x00, sizeof(*ptr));
  if (enabled()) {
    ptr->TRNCMD = msg->cmd.value;
    ptr->PBRKCMD = msg->pbrk.cmd;
  }
  pub_can_->publish(out);
}

bool DbwNode::publishDbwEnabled(bool force)
{
  bool en = enabled();
  bool change = prev_enable_ != en;
  if (change || force) {
    std_msgs::msg::Bool msg;
    msg.data = en;
    pub_sys_enable_->publish(msg);
  }
  prev_enable_ = en;
  return change;
}

void DbwNode::timerCallback() {
  // Publish status periodically, in addition to latched and on change
  if (publishDbwEnabled(true)) {
    RCLCPP_WARN(get_logger(), "DBW system enable status changed unexpectedly");
  }

  // Clear override statuses if necessary
  if (clear()) {
    can_msgs::msg::Frame out;
    out.is_extended = false;

    if (override_brake_) {
      out.id = ID_BRAKE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.data(), 0x00, 8);
      reinterpret_cast<MsgBrakeCmd *>(out.data.data())->CLEAR = 1;
      pub_can_->publish(out);
    }

    if (override_throttle_) {
      out.id = ID_THROTTLE_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.data(), 0x00, 8);
      reinterpret_cast<MsgThrottleCmd *>(out.data.data())->CLEAR = 1;
      pub_can_->publish(out);
    }

    if (override_steering_) {
      out.id = ID_STEERING_CMD;
      out.dlc = 4; // Sending the full eight bytes will fault the watchdog counter (if enabled)
      memset(out.data.data(), 0x00, 8);
      reinterpret_cast<MsgSteeringCmd *>(out.data.data())->CLEAR = 1;
      pub_can_->publish(out);
    }

    if (override_gear_) {
      out.id = ID_GEAR_CMD;
      out.dlc = sizeof(MsgGearCmd);
      memset(out.data.data(), 0x00, 8);
      reinterpret_cast<MsgGearCmd *>(out.data.data())->CLEAR = 1;
      pub_can_->publish(out);
    }
  }
}

void DbwNode::enableSystem() {
  if (!enable_) {
    if (fault()) {
      if (fault_steering_cal_) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Steering calibration fault.");
      }
      if (fault_brakes_) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Braking fault.");
      }
      if (fault_throttle_) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Throttle fault.");
      }
      if (fault_steering_) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Steering fault.");
      }
      if (fault_watchdog_) {
        RCLCPP_WARN(get_logger(), "DBW system not enabled. Watchdog fault.");
      }
    } else {
      enable_ = true;
      if (publishDbwEnabled()) {
        RCLCPP_INFO(get_logger(), "DBW system enabled.");
      } else {
        RCLCPP_INFO(get_logger(), "DBW system enable requested. Waiting for ready.");
      }
    }
  }
}

void DbwNode::disableSystem() {
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    RCLCPP_WARN(get_logger(), "DBW system disabled.");
  }
}

void DbwNode::buttonCancel() {
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    RCLCPP_WARN(get_logger(), "DBW system disabled. Cancel button pressed.");
  }
}

void DbwNode::overrideBrake(bool override, bool timeout) {
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_brake_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(get_logger(), "DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::overrideThrottle(bool override, bool timeout) {
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_throttle_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(get_logger(), "DBW system disabled. Driver override on brake/throttle pedal.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::overrideSteering(bool override, bool timeout) {
  bool en = enabled();
  if (en && timeout) {
    override = false;
  }
  if (en && override) {
    enable_ = false;
  }
  override_steering_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(get_logger(), "DBW system disabled. Driver override on steering wheel.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::overrideGear(bool override) {
  bool en = enabled();
  if (en && override) {
    enable_ = false;
  }
  override_gear_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_WARN(get_logger(), "DBW system disabled. Driver override on shifter.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::timeoutBrake(bool timeout, bool enabled) {
  if (!timeout_brakes_ && enabled_brakes_ && timeout && !enabled) {
    RCLCPP_WARN(get_logger(), "Brake subsystem disabled after 100ms command timeout");
  }
  timeout_brakes_ = timeout;
  enabled_brakes_ = enabled;
}

void DbwNode::timeoutThrottle(bool timeout, bool enabled) {
  if (!timeout_throttle_ && enabled_throttle_ && timeout && !enabled) {
    RCLCPP_WARN(get_logger(), "Throttle subsystem disabled after 100ms command timeout");
  }
  timeout_throttle_ = timeout;
  enabled_throttle_ = enabled;
}

void DbwNode::timeoutSteering(bool timeout, bool enabled) {
  if (!timeout_steering_ && enabled_steering_ && timeout && !enabled) {
    RCLCPP_WARN(get_logger(), "Steering subsystem disabled after 100ms command timeout");
  }
  timeout_steering_ = timeout;
  enabled_steering_ = enabled;
}

void DbwNode::faultBrakes(bool fault) {
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_brakes_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(get_logger(), "DBW system disabled. Braking fault.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::faultThrottle(bool fault) {
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_throttle_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(get_logger(), "DBW system disabled. Throttle fault.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::faultSteering(bool fault) {
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(get_logger(), "DBW system disabled. Steering fault.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::faultSteeringCal(bool fault) {
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_cal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(get_logger(), "DBW system disabled. Steering calibration fault.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src, bool braking) {
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_watchdog_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      RCLCPP_ERROR(get_logger(), "DBW system disabled. Watchdog fault.");
    } else {
      RCLCPP_INFO(get_logger(), "DBW system enabled.");
    }
  }
  if (braking && !fault_watchdog_using_brakes_) {
    RCLCPP_WARN(get_logger(), "Watchdog event: Alerting driver and applying brakes.");
  } else if (!braking && fault_watchdog_using_brakes_) {
    RCLCPP_INFO(get_logger(), "Watchdog event: Driver has successfully taken control.");
  }
  if (fault && src && !fault_watchdog_warned_) {
    switch (src) {
      case dbw_ford_msgs::msg::WatchdogCounter::OTHER_BRAKE:
        RCLCPP_WARN(get_logger(), "Watchdog event: Fault determined by brake controller");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::OTHER_THROTTLE:
        RCLCPP_WARN(get_logger(), "Watchdog event: Fault determined by throttle controller");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::OTHER_STEERING:
        RCLCPP_WARN(get_logger(), "Watchdog event: Fault determined by steering controller");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::BRAKE_COUNTER:
        RCLCPP_WARN(get_logger(), "Watchdog event: Brake command counter failed to increment");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::BRAKE_DISABLED:
        RCLCPP_WARN(get_logger(), "Watchdog event: Brake transition to disabled while in gear or moving");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::BRAKE_COMMAND:
        RCLCPP_WARN(get_logger(), "Watchdog event: Brake command timeout after 100ms");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::BRAKE_REPORT:
        RCLCPP_WARN(get_logger(), "Watchdog event: Brake report timeout after 100ms");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::THROTTLE_COUNTER:
        RCLCPP_WARN(get_logger(), "Watchdog event: Throttle command counter failed to increment");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::THROTTLE_DISABLED:
        RCLCPP_WARN(get_logger(), "Watchdog event: Throttle transition to disabled while in gear or moving");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::THROTTLE_COMMAND:
        RCLCPP_WARN(get_logger(), "Watchdog event: Throttle command timeout after 100ms");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::THROTTLE_REPORT:
        RCLCPP_WARN(get_logger(), "Watchdog event: Throttle report timeout after 100ms");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::STEERING_COUNTER:
        RCLCPP_WARN(get_logger(), "Watchdog event: Steering command counter failed to increment");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::STEERING_DISABLED:
        RCLCPP_WARN(get_logger(), "Watchdog event: Steering transition to disabled while in gear or moving");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::STEERING_COMMAND:
        RCLCPP_WARN(get_logger(), "Watchdog event: Steering command timeout after 100ms");
        break;
      case dbw_ford_msgs::msg::WatchdogCounter::STEERING_REPORT:
        RCLCPP_WARN(get_logger(), "Watchdog event: Steering report timeout after 100ms");
        break;
    }
    fault_watchdog_warned_ = true;
  } else if (!fault) {
    fault_watchdog_warned_ = false;
  }
  fault_watchdog_using_brakes_ = braking;
  if (fault && !fault_watchdog_using_brakes_ && fault_watchdog_warned_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2e3,
                         "Watchdog event: Press left OK button on the steering wheel or cycle power to clear event.");
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src) {
  faultWatchdog(fault, src, fault_watchdog_using_brakes_); // No change to 'using brakes' status
}

void DbwNode::publishJointStates(const rclcpp::Time &stamp, const dbw_ford_msgs::msg::WheelSpeedReport *wheels,
                                 const dbw_ford_msgs::msg::SteeringReport *steering) {
  double dt = (stamp - joint_state_.header.stamp).seconds();
  if (wheels) {
    if (std::isfinite(wheels->front_left)) {
      joint_state_.velocity[JOINT_FL] = wheels->front_left;
    }
    if (std::isfinite(wheels->front_right)) {
      joint_state_.velocity[JOINT_FR] = wheels->front_right;
    }
    if (std::isfinite(wheels->rear_left)) {
      joint_state_.velocity[JOINT_RL] = wheels->rear_left;
    }
    if (std::isfinite(wheels->rear_right)) {
      joint_state_.velocity[JOINT_RR] = wheels->rear_right;
    }
  }
  if (steering) {
    if (std::isfinite(steering->steering_wheel_angle)) {
      const double L = acker_wheelbase_;
      const double W = acker_track_;
      const double r = L / tan(steering->steering_wheel_angle / steering_ratio_);
      joint_state_.position[JOINT_SL] = atan(L / (r - W / 2));
      joint_state_.position[JOINT_SR] = atan(L / (r + W / 2));
    }
  }
  if (dt < 0.5) {
    for (size_t i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(joint_state_.position[i] + dt * joint_state_.velocity[i], 2 * M_PI);
    }
  }
  joint_state_.header.stamp = stamp;
  pub_joint_states_->publish(joint_state_);
}

} // namespace dbw_ford_can

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dbw_ford_can::DbwNode)
