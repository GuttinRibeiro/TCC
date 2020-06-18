#include <inttypes.h>
#include <memory>
#include <chrono>
#include "controller.hpp"

using namespace std::chrono_literals;

Controller::Controller(std::string team, int id, Field *field, int frequency) : Node("controller_"+team+"_"+std::to_string(id)){
  _team = team;
  _id = (qint8) id;
  _wm = new WorldMap(5*1000/frequency);
  _ib = new InfoBus(_wm, _id, _team);
  _time_now = 0.0;
  clock_gettime(CLOCK_REALTIME, &_start);

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_vision = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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

  // Vision
  auto vision_opt = rclcpp::SubscriptionOptions();
  vision_opt.callback_group = _callback_group_vision;
  _subVision = this->create_subscription<vision::msg::Visionpkg>("vision/"+robotToken, rclcpp::QoS(10),
                                                                 std::bind(&Controller::visionCallback, this, std::placeholders::_1),
                                                                 vision_opt);
  _timerVision = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Controller::updateWorldMap, this), _callback_group_vision);
}

void Controller::visionCallback(const vision::msg::Visionpkg::SharedPtr msg) {
  // Synchronize internal timer using messages timestamp
  _time_now = msg->timestamp;

  //Update ball position:
  if(msg->balls.size() > 0) {
    _wm->updateElement(Groups::BALL, 0, 0.01, 0.0, Vector(msg->balls.at(0).x, msg->balls.at(0).y, msg->timestamp, false));
  }

  //Update robots:
  while(msg->robots.size() > 0) {
    auto robot = msg->robots.at(msg->robots.size()-1);
    msg->robots.pop_back();
    // Check team color:
    if(robot.team == "blue") {
      //TODO: enviar orientação
      _wm->updateElement(Groups::BLUE, robot.id, 0.09, 0.0, Vector(robot.x, robot.y, msg->timestamp, false));
    } else if(robot.team == "yellow") {
      //TODO: enviar orientação
      _wm->updateElement(Groups::YELLOW, robot.id, 0.09, 0.0, Vector(robot.x, robot.y, msg->timestamp, false));
    }
  }

  // Init timer
  clock_gettime(CLOCK_REALTIME, &_start);
}

void Controller::updateWorldMap() {
  // Stop timer
  clock_gettime(CLOCK_REALTIME, &_stop);
  // Count how many seconds have passed since last packet received
  _time_now += ((_stop.tv_sec*1E9+_stop.tv_nsec)-(_start.tv_sec*1E9+_start.tv_nsec))/1E9;
  // Check all timestamps and remove old information
  _wm->checkElements(_time_now);
}
