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

namesoace dbw_ford_can{

BelivVehInterface::BelivVehInterface()
        :rclcpp::Node("dbw_interface")
{
    /* set up parameters */
    base_frame_id_ = declare_parameter("base_frame_id", "base_link");   //Frame ID

    /* subscriber */
    //from DbwNode
    sub_can_ = create_subscription<can_msgs::msg::Frame>("can_tx", 10);
    sub_brake_ = create_subscription<dbw_ford_msgs::msg::BrakeReport>("brake_report", 2);
    sub_throttle_ = create_subscription<dbw_ford_msgs::msg::ThrottleReport>("throttle_report", 2);
    sub_steering_ = create_subscription<dbw_ford_msgs::msg::SteeringReport>("steering_report", 2);
    sub_gear_ = create_subscription<dbw_ford_msgs::msg::GearReport>("gear_report", 2);
    sub_misc_1_ = create_subscription<dbw_ford_msgs::msg::Misc1Report>("misc_1_report", 2);
    sub_wheel_speeds_ = create_subscription<dbw_ford_msgs::msg::WheelSpeedReport>("wheel_speed_report", 2);
    sub_wheel_positions_ = create_subscription<dbw_ford_msgs::msg::WheelPositionReport>("wheel_position_report", 2);
    sub_tire_pressure_ = create_subscription<dbw_ford_msgs::msg::TirePressureReport>("tire_pressure_report", 2);
    sub_fuel_level_ = create_subscription<dbw_ford_msgs::msg::FuelLevelReport>("fuel_level_report", 2);
    sub_surround_ = create_subscription<dbw_ford_msgs::msg::SurroundReport>("surround_report", 2);
    sub_sonar_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("sonar_cloud", 2);
    sub_brake_info_ = create_subscription<dbw_ford_msgs::msg::BrakeInfoReport>("brake_info_report", 2);
    sub_throttle_info_ = create_subscription<dbw_ford_msgs::msg::ThrottleInfoReport>("throttle_info_report", 2);
    sub_driver_assist_ = create_subscription<dbw_ford_msgs::msg::DriverAssistReport>("driver_assist_report", 2);
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>("gps/vel", 10);
    sub_gps_time_ = create_subscription<sensor_msgs::msg::TimeReference>("gps/time", 10);
    sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("twist", 10);    

    /* publisher */
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

void BelivVehInterface::recvDbwRpt(
    const dbw_ford_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
    const dbw_ford_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
    const dbw_ford_msgs::msg::ThrottleReport::ConstSharedPtr throttle_rpt,
    const dbw_ford_msgs::msg::BrakeReport::ConstSharedPtr brake_rpt,
    const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr misc1_rpt)
{
    std_msgs::msg::Header header;
    header.frame_id = base_frame_id_;
    header.stamp = get_clock()->now();

    gear_rpt_ptr_ = steer_wheel_rpt;
    steering_rpt_ptr = steering_rpt;
    throttle_rpt_ptr = throttle_rpt;
    brake_rpt_ptr_ = brake_rpt;
    misc1_rpt_ptr_ = global_rpt;

    const double current_velocity = steering_rpt.speed;
    const double current_steer_wheel =
        steering_rpt.steering_wheel_angle;  // current vehicle steering wheel angle [rad]
    const double adaptive_gear_ratio =
        calculateVariableGearRatio(current_velocity, current_steer_wheel);
    const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;


    /* publish steering wheel status */
    {
        SteeringWheelStatusStamped steering_wheel_status_msg;
        steering_wheel_status_msg.stamp = header.stamp;
        steering_wheel_status_msg.data = current_steer_wheel;
        pub_steering_wheel_status_->publish(steering_wheel_status_msg);
    }


  /* publish vehicle status control_mode */
    {
        autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = header.stamp;

        if (report->pedals_enabled && report->steering_enabled && !report->override_latched) {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        } else {
            control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
        }

        pub_pub_control_mode_->publish(control_mode_msg);
    }

    /* publish vehicle status twist */
    {
        autoware_auto_vehicle_msgs::msg::VelocityReport twist;
        twist.header = header;
        twist.longitudinal_velocity = current_velocity;                                 // [m/s]
        twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s] yaw rate
        pub_vehicle_twist_->publish(twist);
    }

    /* publish current shift */
    {
        autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
        gear_report_msg.stamp = header.stamp;
        const auto opt_gear_report = toAutowareShiftReport(*gear_cmd_rpt_ptr_);
        if (opt_gear_report) {
            gear_report_msg.report = *opt_gear_report;
            pub_gear_status_->publish(gear_report_msg);
        }
    }

    /* publish current status */
    {
        autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
        steer_msg.stamp = header.stamp;
        steer_msg.steering_tire_angle = current_steer;
        pub_steering_status_->publish(steer_msg);
    }


    /*publish control mode*/
    {
        autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.stamp = header.stamp;
        if (report->override_latched == 0){

        }
        pub_control_mode_->publish(control_mode_msg);
    }

    /*publish Velocity Report*/
    {
        autoware_auto_vehicle_msgs::msg::VelocityReport twist;
        twist.header = header;
        twist.longitudinal_velocity = current_velocity;                                 // [m/s]
        twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s] yaw rate
        pub_vehicle_twist_->publish(twist);
    }

    /*publish Gear Report*/
    {
        autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
        gear_report_msg.stamp = header.stamp;
        const auto opt_gear_report = toAutowareShiftReport(*gear_cmd_rpt_ptr_);
        if (opt_gear_report) {
          gear_report_msg.report = *opt_gear_report;
          pub_gear_status_->publish(gear_report_msg);
        }
    }

    /*publish TurnIndicatorsReport*/
    {
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
        turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*turn_rpt);
    pub_turn_indicators_status_->publish(turn_msg);

    autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*turn_rpt);
    pub_hazard_lights_status_->publish(hazard_msg);
    }

    /*publish ActuationStatusStamped*/
    {
        ActuationStatusStamped actuation_status;
        actuation_status.header = header;
        actuation_status.status.accel_status = accel_rpt_ptr_->output;
        actuation_status.status.brake_status = brake_rpt_ptr_->output;
        actuation_status.status.steer_status = current_steer;
        pub_actuation_status_->publish(actuation_status);  
    }

    /*publish SteeringWheelStatusStamped*/
    {
        SteeringWheelStatusStamped steering_wheel_status_msg;
        steering_wheel_status_msg.stamp = header.stamp;
        steering_wheel_status_msg.data = current_steer_wheel;
        pub_steering_wheel_status_->publish(steering_wheel_status_msg);
    }

}

}