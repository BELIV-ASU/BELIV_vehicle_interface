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

namespace dbw_ford_can {
    
class BelivVehInterface : public rclcpp::Node
{
public:
    BelivVehInterface();

private:
    void recvDbwRpt(
        const dbw_ford_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
        const dbw_ford_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
        const dbw_ford_msgs::msg::ThrottleReport::ConstSharedPtr throttle_rpt,
        const dbw_ford_msgs::msg::BrakeReport::ConstSharedPtr brake_rpt,
        const dbw_ford_msgs::msg::Misc1Report::ConstSharedPtr misc1_rpt);
        
    /* Set up parameters */
    std::string base_frame_id_;

    /* Subscription */
    //from dbwNode
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Subscription<dbw_ford_msgs::msg::BrakeReport>::SharedPtr sub_brake_;
    rclcpp::Subscription<dbw_ford_msgs::msg::ThrottleReport>::SharedPtr sub_throttle_;
    rclcpp::Subscription<dbw_ford_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
    rclcpp::Subscription<dbw_ford_msgs::msg::GearReport>::SharedPtr sub_gear_;
    rclcpp::Subscription<dbw_ford_msgs::msg::Misc1Report>::SharedPtr sub_misc_1_;
    rclcpp::Subscription<dbw_ford_msgs::msg::WheelSpeedReport>::SharedPtr sub_wheel_speeds_;
    rclcpp::Subscription<dbw_ford_msgs::msg::WheelPositionReport>::SharedPtr sub_wheel_positions_;
    rclcpp::Subscription<dbw_ford_msgs::msg::TirePressureReport>::SharedPtr sub_tire_pressure_;
    rclcpp::Subscription<dbw_ford_msgs::msg::FuelLevelReport>::SharedPtr sub_fuel_level_;
    rclcpp::Subscription<dbw_ford_msgs::msg::SurroundReport>::SharedPtr sub_surround_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar_cloud_;
    rclcpp::Subscription<dbw_ford_msgs::msg::BrakeInfoReport>::SharedPtr sub_brake_info_;
    rclcpp::Subscription<dbw_ford_msgs::msg::ThrottleInfoReport>::SharedPtr sub_throttle_info_;
    rclcpp::Subscription<dbw_ford_msgs::msg::DriverAssistReport>::SharedPtr sub_driver_assist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_gps_vel_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_gps_time_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vin_;      // Deprecated message
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_sys_enable_; // Deprecated message

    /* Publisher */
    // To Autoware
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
        pub_control_mode_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vehicle_twist_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
        pub_steering_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
        pub_turn_indicators_status_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
        pub_hazard_lights_status_;
    rclcpp::Publisher<ActuationStatusStamped>::SharedPtr pub_actuation_status_;
    rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr pub_steering_wheel_status_;
    rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::pub_SharedPtr pub_door_status_;   
    }
}