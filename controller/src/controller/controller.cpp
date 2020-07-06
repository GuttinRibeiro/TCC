#include <inttypes.h>
#include <memory>
#include <chrono>
#include "controller.hpp"

using namespace std::chrono_literals;

Controller::Controller(std::string team, int id, int frequency) : Node("controller_"+team+"_"+std::to_string(id)){
  _team = team;
  _id = (qint8) id;
  _ib = new InfoBus(_id, _team, &_clientPositionRequest, &_clientInfoRequest, &_clientFieldRequest);

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_external_agent = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_controller = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize ROS 2 interaces
  // Actuator
  auto act_opt = rclcpp::PublisherOptions();
  act_opt.callback_group = _callback_group_actuator;
  _pubActuator = this->create_publisher<ctr_msgs::msg::Command>("command/"+robotToken, rclcpp::QoS(10), act_opt);

  // Controller
  _timerCtr = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Controller::run, this), _callback_group_controller);

  // External Agent
  _serviceExternalAgent = this->create_service<ctr_msgs::srv::State>("state_service/"+robotToken,
                                                                     std::bind(&Controller::updateState, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                     rmw_qos_profile_services_default,
                                                                     _callback_group_actuator);
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
}

Controller::~Controller() {
  delete _ib;
}

void Controller::sendCommand(const ctr_msgs::msg::Command *message) {
  _pubActuator->publish(*message);
}
