#include <dataspeed_dbw_msgs/msg/brake_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/brake_report.hpp>
#include <dataspeed_dbw_msgs/msg/gear_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/gear_report.hpp>
#include <dataspeed_dbw_msgs/msg/steering_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/steering_report.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_cmd.hpp>
#include <dataspeed_dbw_msgs/msg/throttle_report.hpp>

#include <dbw_polaris_msgs/msg/brake_cmd.hpp>
#include <dbw_polaris_msgs/msg/brake_report.hpp>
#include <dbw_polaris_msgs/msg/gear_cmd.hpp>
#include <dbw_polaris_msgs/msg/gear_report.hpp>
#include <dbw_polaris_msgs/msg/steering_cmd.hpp>
#include <dbw_polaris_msgs/msg/steering_report.hpp>
#include <dbw_polaris_msgs/msg/throttle_cmd.hpp>
#include <dbw_polaris_msgs/msg/throttle_report.hpp>

#include <rclcpp/rclcpp.hpp>

#include "generic_message.hpp"

namespace dataspeed_dbw_gateway {

namespace common_ns = dataspeed_dbw_msgs::msg;
namespace vehicle_ns = dbw_polaris_msgs::msg;

// Message Name, Message Topic
#define MESSAGE_LIST(C, R) \
/* DsMsg          ds_msg           VehicleMsg      vehicle_msg   */ \
C(BrakeCmd,       brake_cmd,       BrakeCmd,       brake_cmd)       \
R(BrakeReport,    brake_report,    BrakeReport,    brake_report)    \
C(GearCmd,        gear_cmd,        GearCmd,        gear_cmd)        \
R(GearReport,     gear_report,     GearReport,     gear_report)     \
C(SteeringCmd,    steering_cmd,    SteeringCmd,    steering_cmd)    \
R(SteeringReport, steering_report, SteeringReport, steering_report) \
C(ThrottleCmd,    throttle_cmd,    ThrottleCmd,    throttle_cmd)    \
R(ThrottleReport, throttle_report, ThrottleReport, throttle_report) \

#define EMPTY(dname, dtopic, vname, vtopic)
#define MESSAGE_LIST_CMD(X) MESSAGE_LIST(X, EMPTY)
#define MESSAGE_LIST_RPT(X) MESSAGE_LIST(EMPTY, X)

/*
 * Forward messages from topic `ds/cmd` to `vehicle/cmd` and convert types
 * Forward messages from topic `vehicle/report` to `ds/report` and convert types
 */
class PolarisGateway : public rclcpp::Node {
public:
  PolarisGateway(const rclcpp::NodeOptions &options) : rclcpp::Node("gateway", options) {
    // QOS options
    const auto QOS = rclcpp::QoS(2);

    // Node namespaces
    auto node_vh = this; // Launch in the vehicle namespace
    auto node_ds = create_sub_node("ds");

    // Publish and subscribe
    #define CMD_PUB_SUB(dname, dtopic, vname, vtopic) \
    pub_vh_##vtopic = node_vh->create_publisher  <vehicle_ns::vname>(#vtopic, QOS); \
    sub_ds_##dtopic = node_ds->create_subscription<common_ns::dname>(#dtopic, QOS, \
        [this](common_ns::dname::ConstSharedPtr msg) { onMessage(this->pub_vh_##vtopic, msg); });
    #define RPT_PUB_SUB(dname, dtopic, vname, vtopic) \
    pub_ds_##dtopic = node_ds->create_publisher    <common_ns::dname>(#dtopic, QOS); \
    sub_vh_##vtopic = node_vh->create_subscription<vehicle_ns::vname>(#vtopic, QOS, \
        [this](vehicle_ns::vname::ConstSharedPtr msg) { onMessage(this->pub_ds_##dtopic, msg); });
    MESSAGE_LIST(CMD_PUB_SUB, RPT_PUB_SUB)
    #undef CMD_PUB_SUB
    #undef RPT_PUB_SUB
  }

private:
  // Convert message to other type and publish
  template <typename In, typename Out>
  void onMessage(std::shared_ptr<rclcpp::Publisher<Out>> pub, const std::shared_ptr<const In> msg) {
    auto out = std::make_unique<Out>();
    ros2_generic_message::Message<In> msg_in(msg.get());
    ros2_generic_message::Message<Out> msg_out(out.get());
    msg_out.set(msg_in);
    
    pub->publish(std::move(out));
  }

  // Publishers and subscribers
  #define DECLARE_PUB_SUB_CMD(dname, dtopic, vname, vtopic) \
  rclcpp::Subscription<common_ns::dname>::SharedPtr sub_ds_##dtopic; \
  rclcpp::Publisher  <vehicle_ns::vname>::SharedPtr pub_vh_##vtopic;
  #define DECLARE_PUB_SUB_RPT(dname, dtopic, vname, vtopic) \
  rclcpp::Subscription<vehicle_ns::vname>::SharedPtr sub_vh_##vtopic; \
  rclcpp::Publisher    <common_ns::dname>::SharedPtr pub_ds_##dtopic;
  MESSAGE_LIST(DECLARE_PUB_SUB_CMD, DECLARE_PUB_SUB_RPT)
  #undef DECLARE_PUB_SUB_CMD
  #undef DECLARE_PUB_SUB_RPT
};

} // namespace dataspeed_dbw_gateway

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dataspeed_dbw_gateway::PolarisGateway)
