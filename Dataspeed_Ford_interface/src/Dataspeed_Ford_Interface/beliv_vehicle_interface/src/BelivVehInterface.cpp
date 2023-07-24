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

#include "BelivVehInterface.hpp"

namespace dbw_ford_can{

BelivVehInterface::BelivVehInterface()
  :rclcpp::Node("dbw_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /* set up parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");   //Frame ID
  wheelbase = vehicle_info_.wheel_base_m;
  steering_ratio = 14.6;

  /* subscriber */
  //from DbwNode
  sub_brake_ = create_subscription<dbw_ford_msgs::msg::BrakeReport>("/vehicle/brake_report", 2);
  sub_throttle_ = create_subscription<dbw_ford_msgs::msg::ThrottleReport>("/vehicle/throttle_report", 2);
  sub_steering_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::SteeringReport>>(
    this, "/vehicle/steering_report");
  sub_gear_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::GearReport>>(
    this, "/vehicle/gear_report");
  sub_misc_1_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::Misc1Report>>(
    this, "misc_1_report");
  sub_wheel_speeds_ = create_subscription<dbw_ford_msgs::msg::WheelSpeedReport>("/vehicle/wheel_speed_report", 2);
  sub_wheel_positions_ = create_subscription<dbw_ford_msgs::msg::WheelPositionReport>("/vehicle/wheel_position_report", 2);
  sub_tire_pressure_ = create_subscription<dbw_ford_msgs::msg::TirePressureReport>("/vehicle/tire_pressure_report", 2);
  sub_fuel_level_ = create_subscription<dbw_ford_msgs::msg::FuelLevelReport>("/vehicle/fuel_level_report", 2);
  sub_surround_ = create_subscription<dbw_ford_msgs::msg::SurroundReport>("/vehicle/surround_report", 2);
  sub_sonar_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("/vehicle/sonar_cloud", 2);
  sub_brake_info_ = create_subscription<dbw_ford_msgs::msg::BrakeInfoReport>("/vehicle/brake_info_report", 2);
  sub_throttle_info_ = create_subscription<dbw_ford_msgs::msg::ThrottleInfoReport>("/vehicle/throttle_info_report", 2);
  sub_driver_assist_ = create_subscription<dbw_ford_msgs::msg::DriverAssistReport>("/vehicle/driver_assist_report", 2);
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("/vehicle/imu/data_raw", 10);
  sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>("/vehicle/gps/fix", 10);
  sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>("/vehicle/gps/vel", 10);
  sub_gps_time_ = create_subscription<sensor_msgs::msg::TimeReference>("/vehicle/gps/time", 10);
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("/vehicle/twist", 10);   

  //from Autoware
  sub_control_cmd_ = std::make_unique<message_filters::Subscriber<utoware_auto_control_msgs::msg::AckermannControlCommand>>(
    this, "/control/command/control_cmd");
  sub_gear_cmd_ = create_subscription<GearCommand>("input/gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_gear_cmd_ = *msg; });
  sub_manual_gear_cmd_ = create_subscription<GearCommand>("input/manual_gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_manual_gear_cmd_ = *msg; });
  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>("input/turn_indicators_command", QoS{1},std::bind(&SimplePlanningSimulator::on_turn_indicators_cmd, this, _1));
  sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>("input/hazard_lights_command", QoS{1},std::bind(&SimplePlanningSimulator::on_hazard_lights_cmd, this, _1));

  //from UlcNode
  sub_ulc_rpt_ = std::make_unique<message_filters::Subscriber<dataspeed_ulc_msgs::UlcReport>>(
    this, "/vehicle/ulc_report");

  //synchronizer
  beliv_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<BelivFeedbacksSyncPolicy>>(
    BelivFeedbacksSyncPolicy(10), *sub_steering_, *sub_gear_, *sub_misc_1_, *sub_ulc_rpt_,*sub_control_cmd_);
  beliv_feedbacks_sync_->registerCallback(std::bind(
    &BelivVehInterface::callbackBelivRpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));


  /* publisher */
  //to UlcNode
  pub_ulc_cmd_=create_publisher<dataspeed_ulc_msgs::msg::UlcCmd>("/vehicle/ulc_cmd", 2);

  //to Autoware
  pub_control_mode_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  pub_vehicle_twist_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  pub_steering_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  pub_gear_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  pub_turn_indicators_status_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  pub_hazard_lights_status_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  pub_actuation_status_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1);
  pub_steering_wheel_status_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  pub_door_status_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);
}

void BelivVehInterface::callbackInterface(
  const dbw_ford_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
  const dbw_ford_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr misc1_rpt,
  const dataspeed_ulc_msgs::UlcReport::ConstSharePtr ulc_rpt,
  const autoware_auto_control_msgs::msg::AckermannControlCommand ackermann_cmd)
{
  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  sub_steering_ptr_ = steering_rpt;
  sub_gear_ptr_ = gear_rpt;
  sub_misc1_ptr_ = misc1_rpt;
  sub_ulc_rpt_ptr_ = ulc_rpt;
  control_cmd_ptr_ = ackermann_cmd;

  const double current_velocity = sub_teering_ptr.speed;
  const double current_steer_wheel =
    sub_teering_ptr.steering_wheel_angle;  // current vehicle steering wheel angle [rad]
  const double current_steer = current_steer_wheel / / steering_ratio;

  /* publish steering wheel status */
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_msg.data = current_steer_wheel;
    pub_steering_wheel_status_->publish(steering_wheel_status_msg);    
  }

  /* publish current steering status */
  {
    autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    pub_steering_status_->publish(steer_msg);
  }

  /*publish Velocity Report*/
  {
    autoware_auto_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s] yaw rate
    pub_vehicle_twist_->publish(twist);
  }

  /* publish vehicle status control_mode */
  {
    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;

    if (sub_ulc_rpt_ptr_->pedals_enabled && sub_ulc_rpt_ptr_->steering_enabled && !sub_ulc_rpt_ptr_->override_latched) {
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else {
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
    }

    pub_control_mode_->publish(control_mode_msg);
  }

  /* publish current shift */
  {
    autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(*sub_gear_ptr_);
    if (opt_gear_report) {
      gear_report_msg.report = *opt_gear_report;
      pub_gear_status_->publish(gear_report_msg);
    }
  }

  /*publish TurnIndicatorsReport*/
  {
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*sub_misc1_ptr);
    pub_turn_indicators_status_->publish(turn_msg);

    autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*sub_misc1_ptr);
    pub_hazard_lights_status_->publish(hazard_msg);
  }

  {
    acker_wheelbase_ = 2.98; // 112.2 inches
    track_width = 1.61 ;
    dataspeed_ulc_msgs::msg::UlcCmd ulc_cmd;

  // Populate command fields
    ulc_cmd.accel_cmd = 0.0; // Not used when pedals_mode is SPEED_MODE
    ulc_cmd.pedals_mode = dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE;
    ulc_cmd.coast_decel = false;
    ulc_cmd.yaw_command = current_velocity * tan(control_cmd_ptr_->lateral.steering_tire_angle) / acker_wheelbase_;
    ulc_cmd.steering_mode = dataspeed_ulc_msgs::msg::UlcCmd::YAW_RATE_MODE;


    // Set other fields to default values
    ulc_cmd.clear = false;
    ulc_cmd.enable_pedals = true;
    ulc_cmd.enable_shifting = true;
    ulc_cmd.enable_steering = true;
    ulc_cmd.shift_from_park = false;
    ulc_cmd.linear_accel = 0;
    ulc_cmd.linear_decel = 0;
    ulc_cmd.angular_accel = 0;
    ulc_cmd.lateral_accel = 0;
    ulc_cmd.jerk_limit_throttle = 0;
    ulc_cmd.jerk_limit_brake = 0;

    // Publish command message
    pub_ulc_cmd_->publish(ucl_cmd);
}
}

std::optional<int32_t> BelivVehInterface::toAutowareShiftReport(
  const dbw_ford_msgs::msg::GearReport &gear_rpt)
{
  using aGearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  using dGearReport = dbw_ford_msgs::msg::GearReport;

  if (gear_rpt.gear == dGearReport::PARK) {
    return aGearReport::PARK;
  }
  if (gear_rpt.gear == dGearReport::REVERSE) {
    return aGearReport::REVERSE;
  }
  if (gear_rpt.gear == dGearReport::NEUTRAL) {
    return aGearReport::NEUTRAL;
  }
  if (gear_rpt.gear == dGearReport::DRIVE) {
    return aGearReport::DRIVE;
  }
  if (gear_rpt.gear == dGearReport::LOW) {
    return acosf32::LOW;
  }
  return {};
}

int32_t BelivVehInterface::toAutowareTurnIndicatorsReport(
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr &misc1_rpt;)
{
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using dbw_ford_msgs::msg::Misc1Report;
  using dbw_ford_msgs::msg::TurnSinal;

  if (misc1_rpt->TurnSignal.value == dbw_ford_msgs::msg::RIGHT){
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (misc1_rpt->TurnSignal.value == dbw_ford_msgs::msg::LEFT){
        return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (misc1_rpt->TurnSignal.value == dbw_ford_msgs::msg::NONE){
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}

int32_t DbwNode::toAutowareHazardLightsReport(
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr &misc1_rpt;)
{
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using dbw_ford_msgs::msg::Misc1Report;
  using dbw_ford_msgs::msg::TurnSinal;

  if (misc1_rpt->TurnSignal.value == dbw_ford_msgs::msg::HAZARD) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}
}