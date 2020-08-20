#include "control_algorithm.hpp"
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

Control_Algorithm::Control_Algorithm(rclcpp::Node *owner, std::string name) {
  if(owner == nullptr)  {
    std::cout << "[Control algorithm] Invalid node pointer received\n";
    return;
  }

  _node = owner;
  _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _name = name;
}

Control_Algorithm::~Control_Algorithm() {

}

void Control_Algorithm::declareFloatParameter(std::string variableName, float defaultValue, std::function<void(void)> callback, int frequency) {
  _node->declare_parameter<float>(name()+"/"+variableName, defaultValue);
//  _node->create_wall_timer(std::chrono::milliseconds(1000/frequency), callback, _callback_group);
  _node->create_wall_timer(12ms, callback, _callback_group);

  std::cout << "Parameter declared\n";
}

bool Control_Algorithm::getFloatParameter(std::string variableName, float *parameter) {
  return _node->get_parameter(variableName, *parameter);
}
