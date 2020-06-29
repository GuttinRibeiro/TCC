#include <inttypes.h>
#include <memory>
#include <chrono>
#include "controller.hpp"

using namespace std::chrono_literals;

Controller::Controller(std::string team, int id, Field *field, int frequency) : Node("controller_"+team+"_"+std::to_string(id)){
  _team = team;
  _id = (qint8) id;
//  _ib = new InfoBus(_wm, _id, _team);

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_external_agent = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_controller = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

}

Controller::~Controller() {

}

void Controller::sendCommand(const ctr_msgs::msg::Command *message) {
  _pubActuator->publish(*message);
}
