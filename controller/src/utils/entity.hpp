#ifndef ENTITY_HPP
#define ENTITY_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

class Entity : public rclcpp::Node {
private:
  [[noreturn]] void execute(int frequency);
private:
  virtual void run() = 0;
  virtual void configure() = 0;
  virtual std::string name() = 0;
public:
  Entity(std::string name, int frequency);
};

#endif // ENTITY_HPP
