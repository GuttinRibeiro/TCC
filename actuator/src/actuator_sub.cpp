#include "actuator_sub.hpp"

using std::placeholders::_1;

Actuator_Sub::Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID) : Node(nodeName+std::to_string(nodeID)) {
    // Internal variables
    _team = team;
    _id = id;
    std::string robotToken = team+"_"+std::to_string(id);

    // Configure callback group to avoid concurrency issues
    _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Command table:
    rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr command_element;
    auto opt_1 = rclcpp::SubscriptionOptions();
    opt_1.callback_group = _callback_group;
    opt_1.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    opt_1.topic_stats_options.publish_topic = "actuator/command/"+robotToken+"/statistics";
    _subsTable1 = this->create_subscription<ctr_msgs::msg::Command>("actuator/command/"+robotToken,
                                                                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                                                                std::bind(&Actuator_Sub::send_cmd, this, _1), opt_1);

    // Velocity table:
    rclcpp::Subscription<ctr_msgs::msg::Velocity>::SharedPtr velocity_element;
    auto opt_2 = rclcpp::SubscriptionOptions();
    opt_2.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    opt_2.topic_stats_options.publish_topic = "actuator/velocity/"+robotToken+"/statistics";
    opt_2.callback_group = _callback_group;
    _subsTable2 = this->create_subscription<ctr_msgs::msg::Velocity>("actuator/velocity/"+robotToken,
                                                                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                                                                std::bind(&Actuator_Sub::send_velocity, this, _1), opt_2);
}
