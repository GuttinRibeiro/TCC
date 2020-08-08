#include "controller.hpp"
#include "../utils/utils.hpp"

using namespace std::chrono_literals;

Controller::Controller(std::string team, int id, int frequency) : Entity ("controller_"+team+"_"+std::to_string(id), frequency){
  // Internal
  _team = team;
  _id = (qint8) id;

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_navigation = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_external_agent = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Initialize ROS 2 interaces
  // Actuator
  auto act_opt = rclcpp::PublisherOptions();
  act_opt.callback_group = _callback_group_actuator;
  _pubActuator = this->create_publisher<ctr_msgs::msg::Command>("actuator/command/"+robotToken, rclcpp::QoS(10), act_opt);

  // Navigation
  auto nav_opt = rclcpp::PublisherOptions();
  nav_opt.callback_group = _callback_group_navigation;
  _pubNavigation = this->create_publisher<ctr_msgs::msg::Navigation>("navigation/motion_specification/"+robotToken,
                                                                     rclcpp::QoS(10), nav_opt);
  // External Agent
  _serviceExternalAgent = this->create_service<ctr_msgs::srv::State>("state_service/"+robotToken,
                                                                     std::bind(&Controller::updateState, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                     rmw_qos_profile_services_default,
                                                                     _callback_group_external_agent);
  // Map
  _clientPositionRequest = this->create_client<ctr_msgs::srv::Elementrequest>("map_service/"+robotToken+"/position",
                                                                               rmw_qos_profile_services_default,
                                                                               _callback_group_map);

  _clientInfoRequest = this->create_client<ctr_msgs::srv::Inforequest>("map_service/"+robotToken+"/info",
                                                                       rmw_qos_profile_services_default,
                                                                       _callback_group_map);

  _clientFieldRequest = this->create_client<ctr_msgs::srv::Fieldinformationrequest>("map_service/"+robotToken+"/field",
                                                                                    rmw_qos_profile_services_default,
                                                                                    _callback_group_map);

  // Info bus
  _ib = new InfoBus(_id, _team, &_clientPositionRequest, &_clientInfoRequest, &_clientFieldRequest);
}

Controller::~Controller() {
  delete _ib;
}

void Controller::send_command(const ctr_msgs::msg::Command &msg) {
  _pubActuator->publish(msg);
}

void Controller::goToLookTo(Vector destination, float orientation) {
  if(destination.isUnknown()) {
    std::cout << "[Controller] Desired destination is unknown!\n";
    return;
  }

  orientation = Utils::wrapToTwoPi(orientation);
  _pubNavigation->publish(encodeNavMessage(destination, orientation));
}

void Controller::goTo(Vector destination) {
  if(destination.isUnknown()) {
    std::cout << "[Controller] Desired destination is unknown!\n";
    return;
  }

  float orientation = Utils::wrapToTwoPi(infoBus()->myOrientation());
  _pubNavigation->publish(encodeNavMessage(destination, orientation));
}

void Controller::lookTo(float orientation) {
  orientation = Utils::wrapToTwoPi(orientation);
  _pubNavigation->publish(encodeNavMessage(infoBus()->myPosition(), orientation));
}
