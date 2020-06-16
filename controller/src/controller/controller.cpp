#include <inttypes.h>
#include <memory>
#include "controller.hpp"

Controller::Controller(std::string team, int id) : Node("controller_"+team+"_"+std::to_string(id)){
  _team = team;
  _id = (qint8) id;

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_vision = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_external_agent = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize ROS 2 interaces
  // Actuator
  auto act_opt = rclcpp::PublisherOptions();
  act_opt.callback_group = _callback_group_actuator;
  _pubActuator = this->create_publisher<ctr_msgs::msg::Command>("command/"+robotToken, rclcpp::QoS(10), act_opt);

  // External Agent
  _serviceExternalAgent = this->create_service<ctr_msgs::srv::State>("state_service/"+robotToken,
                                                                     std::bind(&Controller::updateState, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                     rmw_qos_profile_services_default,
                                                                     _callback_group_actuator);

  // Vision
  auto vision_opt = rclcpp::SubscriptionOptions();
  vision_opt.callback_group = _callback_group_vision;
  _subVision = this->create_subscription<vision::msg::Visionpkg>("vision/"+robotToken, rclcpp::QoS(10),
                                                                 std::bind(&Controller::updateWorldMap, this, std::placeholders::_1),
                                                                 vision_opt);
}

void Controller::updateWorldMap(const vision::msg::Visionpkg::SharedPtr msg) {
  //Do something with stuff
}
