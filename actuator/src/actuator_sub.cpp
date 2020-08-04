#include "actuator_sub.hpp"

using std::placeholders::_1;

Actuator_Sub::Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID) : Node(nodeName+std::to_string(nodeID)) {
    // Internal variables
    _team = team;
    _id = id;

    // Configure callback group to avoid concurrency issues
    _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Command table:
    rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr command_element;
    auto opt_1 = rclcpp::SubscriptionOptions();
    opt_1.callback_group = _callback_group;
    _subsTable1 = this->create_subscription<ctr_msgs::msg::Command>("actuator/command/"+_team+"_"+std::to_string(_id),
                                                                10, std::bind(&Actuator_Sub::send_cmd, this, _1), opt_1);

    // Velocity table:
    rclcpp::Subscription<ctr_msgs::msg::Velocity>::SharedPtr velocity_element;
    auto opt_2 = rclcpp::SubscriptionOptions();
    opt_2.callback_group = _callback_group;
    _subsTable2 = this->create_subscription<ctr_msgs::msg::Command>("actuator/velocity/"+_team+"_"+std::to_string(_id),
                                                                10, std::bind(&Actuator_Sub::send_velocity, this, _1), opt_2);
}
