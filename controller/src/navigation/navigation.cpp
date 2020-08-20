#include "navigation.hpp"
#include "../utils/utils.hpp"

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"

#include <ctime>
#include <math.h>
#include <chrono>

Navigation::Navigation(std::string team, int id, int frequency) : rclcpp::Node("navigation_"+team+"_"+std::to_string(id)), Entity(frequency) {
  _frequency = frequency;
  _team = team;
  _id = (qint8)id;
  std::string robotToken = team+"_"+std::to_string(id);
  _destination = Vector();
  _orientation = 0.0;

  // Create callback groups for multi-threading execution
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_server = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_actuator= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_nav_messages = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Actuator
  auto act_opt = rclcpp::PublisherOptions();
  act_opt.callback_group = _callback_group_actuator;
  _pubActuator = this->create_publisher<ctr_msgs::msg::Velocity>("actuator/velocity/"+robotToken, rclcpp::QoS(10), act_opt);

  // Map
  _clientElementRequest = this->create_client<ctr_msgs::srv::Elementrequest>("map_service/"+robotToken+"/position",
                                                                               rmw_qos_profile_services_default,
                                                                               _callback_group_map);

  _clientInfoRequest = this->create_client<ctr_msgs::srv::Inforequest>("map_service/"+robotToken+"/info",
                                                                       rmw_qos_profile_services_default,
                                                                       _callback_group_map);

  _clientFieldRequest = this->create_client<ctr_msgs::srv::Fieldinformationrequest>("map_service/"+robotToken+"/field",
                                                                                    rmw_qos_profile_services_default,
                                                                                    _callback_group_map);

  auto path_opt = rclcpp::PublisherOptions();
  path_opt.callback_group = _callback_group_map;
  _pubPath = this->create_publisher<ctr_msgs::msg::Path>("vision/path/"+robotToken, rclcpp::QoS(10),
                                                         path_opt);

  // Controller subscriber
  auto ctr_opt = rclcpp::SubscriptionOptions();
  ctr_opt.callback_group = _callback_group_nav_messages;
  _subNavMessages = this->create_subscription<ctr_msgs::msg::Navigation>("navigation/motion_specification/"+robotToken, rclcpp::QoS(10),
                                                                         std::bind(&Navigation::callback, this, std::placeholders::_1), ctr_opt);

  // Robot constraints
  _maxLinearSpeed = 2.25;
  _maxLinearAcceleration = 1.75;
  _maxAngularSpeed = 220.0/180.0*M_PI;
  _maxAngularAcceleration = 1.75/2.25*_maxAngularSpeed;
  _callback_group_parameters = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->declare_parameter<float>("maxAngSpeed", _maxAngularSpeed);
  this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Navigation::getMaxAngSpeed, this), _callback_group_parameters);
  this->declare_parameter<float>("maxLinSpeed", _maxLinearSpeed);
  this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Navigation::getMaxLinSpeed, this), _callback_group_parameters);
  this->declare_parameter<float>("maxAngAcceleration", _maxAngularAcceleration);
  this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Navigation::getMaxAngAcceleration, this), _callback_group_parameters);
  this->declare_parameter<float>("maxLinAcceleration", _maxLinearAcceleration);
  this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Navigation::getMaxLinAcceleration, this), _callback_group_parameters);
  this->start();
}

Navigation::~Navigation() {
  delete _ib;
  delete _navAlg;
  delete _linCtrAlg;
  delete  _angCtrAlg;
}

void Navigation::sendVelocity(float vx, float vy, float vang) {
  auto message = ctr_msgs::msg::Velocity();
  message.vx = vx;
  message.vy = vy;
  message.vang = vang;
  _pubActuator->publish(message);
}

void Navigation::configure() {
  // Info bus
  _ib = new InfoBus(_id, _team, &_clientElementRequest, &_clientInfoRequest, &_clientFieldRequest);

  // Allocate a nav alg
  _navAlg = new PF(_ib, _id);

  // Allocate and configure control algorithms
  _linCtrAlg = new Discrete_PID(this, "linear_ctr_alg");
  this->set_parameter(rclcpp::Parameter(_linCtrAlg->name()+"/"+"kp", 3.0));
  this->set_parameter(rclcpp::Parameter(_linCtrAlg->name()+"/"+"ki", 0.0));
  this->set_parameter(rclcpp::Parameter(_linCtrAlg->name()+"/"+"kd", 0.5));
  this->set_parameter(rclcpp::Parameter(_linCtrAlg->name()+"/"+"T0", 1.0/_frequency));
  _angCtrAlg = new Discrete_PID(this, "angular_ctr_alg");
  this->set_parameter(rclcpp::Parameter(_angCtrAlg->name()+"/"+"kp", 3.0));
  this->set_parameter(rclcpp::Parameter(_angCtrAlg->name()+"/"+"ki", 0.0));
  this->set_parameter(rclcpp::Parameter(_angCtrAlg->name()+"/"+"kd", 0.5));
  this->set_parameter(rclcpp::Parameter(_angCtrAlg->name()+"/"+"T0", 1.0/_frequency));
}

void Navigation::callback(ctr_msgs::msg::Navigation::SharedPtr msg) {
  _orientation = msg->orientation;
  _destination.setX(msg->destination.x);
  _destination.setY(msg->destination.y);
  _destination.setZ(msg->destination.z);
  _destination.setIsUnknown(false);

  // Configure nav alg
  _navAlg->avoidBall(msg->avoidball);
  _navAlg->avoidEnemies(msg->avoidenemies);
  _navAlg->avoidAllies(msg->avoidallies);
}

ctr_msgs::msg::Path Navigation::generatePathMessage(QLinkedList<Vector> path) const {
  ctr_msgs::msg::Path msg;
  while(path.size() > 0) {
    ctr_msgs::msg::Position pos;
    Vector currentPos = path.takeFirst();
    pos.x = currentPos.x();
    pos.y = currentPos.y();
    pos.z = currentPos.z();
    pos.isvalid = true;
    msg.path.push_back(pos);
  }

  return msg;
}

void Navigation::run() {
//  std::cout << "Navigation is running!\n";
//  std::cout << "[Navigation] PID error: " << _linCtrAlg->iterate(10.0) << "\n";

//  timespec start, stop;
//  clock_gettime(CLOCK_REALTIME, &start);
  // Wait for a valid destination
  if(_destination.isUnknown()) {
    return;
  }
  _navAlg->setDestination(_destination);
  _navAlg->setOrientation(_orientation);
  QLinkedList<Vector> path = _navAlg->path();
  if(path.size() > 0) {
    _pubPath->publish(generatePathMessage(path));
  } else {
    std::cout << "[Navigation] Empty path received\n";
  }
//  clock_gettime(CLOCK_REALTIME, &stop);
//  auto elapsed = ((stop.tv_sec*1E9+stop.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec))/1E6; // in ms
//  std::cout << "[Navigation] Effective loop frequency: " << 1000/(elapsed) << " Hz\n";
}

void Navigation::getMaxAngSpeed() {
  this->get_parameter("maxAngSpeed", _maxAngularSpeed);
  std::cout << "[Navigation] New maxAngSpeed: " << _maxAngularSpeed << "\n";
}

void Navigation::getMaxLinSpeed() {
  this->get_parameter("maxLinSpeed", _maxLinearSpeed);
  std::cout << "[Navigation] New maxLinSpeed: " << _maxLinearSpeed << "\n";
}

void Navigation::getMaxAngAcceleration() {
  this->get_parameter("maxAngAcceleration", _maxAngularAcceleration);
  std::cout << "[Navigation] New maxAngAcceleration: " << _maxAngularAcceleration<< "\n";
}

void Navigation::getMaxLinAcceleration() {
  this->get_parameter("maxLinAcceleration", _maxLinearAcceleration);
  std::cout << "[Navigation] New maxLinAcceleration: " << _maxAngularAcceleration << "\n";
}
