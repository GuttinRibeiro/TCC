#ifndef CONTROL_ALGORITHM_HPP
#define CONTROL_ALGORITHM_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <functional>

class Control_Algorithm {
private:
  rclcpp::Node *_node;
  rclcpp::CallbackGroup::SharedPtr _callback_group;

protected:
  std::string _name;

public:
  Control_Algorithm(rclcpp::Node *owner, std::string name);
  virtual ~Control_Algorithm();
  virtual float iterate(float error) = 0;
  std::string name() const {return _name;}

  void declareFloatParameter(std::string variableName, float defaultValue, std::function<void(void)> callback, int frequency = 30);
  bool getFloatParameter(std::string variableName, float *parameter);
};

#endif // CONTROL_ALGORITHM_HPP
