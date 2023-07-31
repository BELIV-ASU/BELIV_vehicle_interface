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

#include "beliv_veh_interface.hpp"

namespace dbw_ford_can{

BelivVehInterface::BelivVehInterface()
  :rclcpp::Node("beliv_veh_interface")
{
  /* set up parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");   //Frame ID
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);
  wheel_base_ = 2.98;
  steering_ratio_ = 14.6;

  /* subscriber */
  using std::placeholders::_1;
  using std::placeholders::_2;

  //from DbwNode
  sub_brake_ = create_subscription<dbw_ford_msgs::msg::BrakeReport>("/vehicle/brake_report", rclcpp::QoS{2}, 
    std::bind(&BelivVehInterface::callbackBrakeRpt, this, _1));
//sub_throttle_ = create_subscription<dbw_ford_msgs::msg::ThrottleReport>("/vehicle/throttle_report", 2);
  sub_steering_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::SteeringReport>>(
    this, "/vehicle/steering_report");
  sub_gear_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::GearReport>>(
    this, "/vehicle/gear_report");
  sub_misc_1_ = std::make_unique<message_filters::Subscriber<dbw_ford_msgs::msg::Misc1Report>>(
    this, "/vehicle/misc_1_report");
/*  sub_wheel_speeds_ = create_subscription<dbw_ford_msgs::msg::WheelSpeedReport>("/vehicle/wheel_speed_report", 2);
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
 */
  //from Autoware
  sub_control_cmd_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&BelivVehInterface::callbackControlCmd, this, _1));
  //sub_control_cmd_ = std::make_unique<message_filters::Subscriber<Autoware_auto_control_msgs::msg::AckermannControlCommand>>(
    //this, "/control/command/control_cmd");
  //sub_gear_cmd_ = create_subscription<GearCommand>("input/gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_gear_cmd_ = *msg; });
  //sub_manual_gear_cmd_ = create_subscription<GearCommand>("input/manual_gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_manual_gear_cmd_ = *msg; });
/*   
  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>("input/turn_indicators_command", QoS{1},std::bind(&SimplePlanningSimulator::on_turn_indicators_cmd, this, _1));
  sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>("input/hazard_lights_command", QoS{1},std::bind(&SimplePlanningSimulator::on_hazard_lights_cmd, this, _1));
 */
  sub_emergency_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&BelivVehInterface::callbackEmergencyCmd, this, _1));
  control_mode_server_ = create_service<ControlModeCommand>(
    "input/control_mode_request", std::bind(&BelivVehInterface::onControlModeRequest, this, _1, _2));

  //from UlcNode
  sub_ulc_rpt_ = std::make_unique<message_filters::Subscriber<dataspeed_ulc_msgs::msg::UlcReport>>(
    this, "/vehicle/ulc_report");

  //from DbwNode
  sub_enable_ = create_subscription<std_msgs::msg::Bool>("/vehicle/dbw_enabled", rclcpp::QoS(2).transient_local(),
    std::bind(&BelivVehInterface::publishCommands, this, _1));   

  //synchronizer
  beliv_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<BelivFeedbacksSyncPolicy>>(
    BelivFeedbacksSyncPolicy(10), *sub_steering_, *sub_gear_, *sub_misc_1_, *sub_ulc_rpt_);
  
  beliv_feedbacks_sync_->registerCallback(std::bind(
    &BelivVehInterface::callbackInterface, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4));

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
/*   pub_actuation_status_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1); */
  pub_steering_wheel_status_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  pub_door_status_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);


  // Timer
  //const auto period_ns = rclcpp::Rate(loop_rate_).period();
  //timer_ = rclcpp::create_timer(
    //this, get_clock(), period_ns, std::bind(&BelivVehInterface::publishCommands, this));

}

void BelivVehInterface::callbackInterface(
  const dbw_ford_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
  const dbw_ford_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr misc1_rpt,
  const dataspeed_ulc_msgs::msg::UlcReport::ConstSharedPtr ulc_rpt)
{
  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  is_dbw_rpt_received_ = true;
  sub_steering_ptr_ = steering_rpt;
  sub_gear_ptr_ = gear_rpt;
  sub_misc1_ptr_ = misc1_rpt;
  sub_ulc_rpt_ptr_ = ulc_rpt;

  const double current_velocity = sub_steering_ptr_->speed;
  const double current_steer_wheel = sub_steering_ptr_->steering_wheel_angle;  // current vehicle steering wheel angle [rad]
  const double current_steer = current_steer_wheel / steering_ratio_;

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

    if (sub_enable_) {
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
      gear_report_msg.report = opt_gear_report;
      pub_gear_status_->publish(gear_report_msg);
    }
  }

  /*publish TurnIndicatorsReport*/
  {
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*sub_misc1_ptr_);
    pub_turn_indicators_status_->publish(turn_msg);

    autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*sub_misc1_ptr_);
    pub_hazard_lights_status_->publish(hazard_msg);
  }

  {
    acker_wheelbase_ = 2.98; // 112.2 inches
    track_width = 1.61;
    dataspeed_ulc_msgs::msg::UlcCmd ulc_cmd;

  // Populate command fields
    ulc_cmd.linear_velocity = control_cmd_ptr_->longitudinal.speed;
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
    pub_ulc_cmd_->publish(ulc_cmd);
}
}

int32_t BelivVehInterface::toAutowareShiftReport(
  const dbw_ford_msgs::msg::GearReport::ConstSharedPtr &gear_rpt)
{
  using autoware_auto_vehicle_msgs::msg::GearReport;
  using dbw_ford_msgs::msg::Gear;

  if (gear_rpt->state.PARK) {
    return static_cast<int32_t>(GearReport::PARK);
  }
  if (gear_rpt->state.REVERSE) {
    return static_cast<int32_t>(GearReport::REVERSE);
  }
  if (gear_rpt->state.NEUTRAL){
    return static_cast<int32_t>(GearReport::NEUTRAL);
  }
  if (gear_rpt->state.DRIVE) {
    return static_cast<int32_t>(GearReport::DRIVE);
  }
  if (gear_rpt->state.LOW) {
    return static_cast<int32_t>(GearReport::LOW);
  }

  //return {};
}


void BelivVehInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void BelivVehInterface::callbackBrakeRpt(
  const dbw_ford_msgs::msg::BrakeReport::ConstSharedPtr rpt)
{
  sub_brake_ptr_ = rpt;
}

void BelivVehInterface::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
    //sub_enable_ = true;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL) {
    //sub_enable_ = false;
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

void BelivVehInterface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}


int32_t BelivVehInterface::toAutowareTurnIndicatorsReport(
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr &misc1_rpt)
{
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using dbw_ford_msgs::msg::Misc1Report;
  using dbw_ford_msgs::msg::TurnSignal;

  if (misc1_rpt->turn_signal.value == dbw_ford_msgs::msg::TurnSignal::RIGHT){
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (misc1_rpt->turn_signal.value == dbw_ford_msgs::msg::TurnSignal::LEFT){
        return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (misc1_rpt->turn_signal.value == dbw_ford_msgs::msg::TurnSignal::NONE){
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}



int32_t BelivVehInterface::toAutowareHazardLightsReport(
  const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr &misc1_rpt)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using dbw_ford_msgs::msg::Misc1Report;
  using dbw_ford_msgs::msg::TurnSignal;

  if (misc1_rpt->turn_signal.value == dbw_ford_msgs::msg::TurnSignal::HAZARD) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}

}
