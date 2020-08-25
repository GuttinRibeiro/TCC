#include "navigation.hpp"
#include "../utils/utils.hpp"

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"

#include <ctime>
#include <math.h>

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
//  _maxLinearSpeed = 2.25;
//  _maxLinearAcceleration = 1.75;
//  _maxAngularSpeed = 220.0/180.0*M_PI;
//  _maxAngularAcceleration = 1.75/2.25*_maxAngularSpeed;
  _maxLinearSpeed = 0.1;
  _maxLinearAcceleration = 0.01;
  _maxAngularSpeed = 0.2;
  _maxAngularAcceleration = 0.8*_maxAngularSpeed;
  _lastLinSpeed = 0.0;
  _lastAngSpeed = 0.0;
  _mutex.unlock();
  this->start();
}

Navigation::~Navigation() {
  this->remove_on_set_parameters_callback(_handler.get());
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
  _linCtrAlg = new Discrete_PID("linear_ctr_alg", _frequency);
  _angCtrAlg = new Discrete_PID("angular_ctr_alg", _frequency);

  // Declare all parameters
  addDoubleParam("maxAngSpeed", &_maxAngularSpeed, _maxAngularSpeed);
  addDoubleParam("maxLinSpeed", &_maxLinearSpeed, _maxLinearSpeed);
  addDoubleParam("maxAngAcceleration", &_maxAngularAcceleration, _maxAngularAcceleration);
  addDoubleParam("maxLinAcceleration", &_maxLinearAcceleration, _maxLinearAcceleration);
  addDoubleParam(_linCtrAlg->getParamTable());
  addDoubleParam(_angCtrAlg->getParamTable());
  _handler = this->add_on_set_parameters_callback(std::bind(&Navigation::paramCallback, this, std::placeholders::_1));
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
  // Wait for a valid destination
  if(_destination.isUnknown()) {
    return;
  }

  _mutex.lock();
  timespec start, stop;
  clock_gettime(CLOCK_REALTIME, &start);
  _navAlg->setDestination(_destination);
  _navAlg->setOrientation(_orientation);
  QLinkedList<Vector> path = _navAlg->path();
  if(path.size() > 0) {
    _pubPath->publish(generatePathMessage(path));
  } else {
    std::cout << "[Navigation] Empty path received\n";
    _mutex.unlock();
    return;
  }
  Vector currPos = path.takeFirst();
  Vector nextMovement = path.takeFirst() - currPos;
  nextMovement = nextMovement/nextMovement.norm();
  float myOrientation = _ib->myOrientation();
  nextMovement = Utils::rotateVectorAroundZ(nextMovement, M_PI_2-myOrientation);
  double linearError = calculateCurveLength(path);
  double desiredSpeed = _linCtrAlg->iterate(linearError);
  double desiredAngSpeed = _angCtrAlg->iterate(_orientation-myOrientation);
  clock_gettime(CLOCK_REALTIME, &stop);
  auto elapsed = ((stop.tv_sec*1E9+stop.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec))/1E3; // in s
  _mutex.unlock();
  // Apply constraints to get a feasible desired speed:
  // Linear speed:
  desiredSpeed = calculateConstrainedSpeed(desiredSpeed, _lastLinSpeed, 0.0, _maxLinearSpeed, _maxLinearAcceleration, elapsed);

  // Angular speed:
  desiredAngSpeed = calculateConstrainedSpeed(desiredAngSpeed, _lastAngSpeed, 0.0, _maxAngularSpeed, _maxAngularAcceleration, elapsed);

  // Send command:
  sendVelocity(nextMovement.x()*desiredSpeed, nextMovement.y()*desiredSpeed, desiredAngSpeed);
  _lastLinSpeed = desiredSpeed;
  _lastAngSpeed = desiredAngSpeed;
}

rcl_interfaces::msg::SetParametersResult Navigation::paramCallback(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for(const auto & parameter: parameters) {
    if(parameter.get_type() == rclcpp::PARAMETER_DOUBLE) {
      double *address = _paramAddressTable.value(parameter.get_name());
      if(address == nullptr) {
        result.successful = false;
        result.reason = "'" + parameter.get_name() + "' not defined before";
      } else {
        _mutex.lock();
        *address = parameter.as_double();
        _mutex.unlock();
        std::cout << "[Navigation] Parameter " + parameter.get_name() + " updated\n";
      }
    } else {
      result.successful = false;
      result.reason = "'" + parameter.get_name() + "' must be a double";
    }
  }
  return result;
}

bool Navigation::addDoubleParam(std::string paramName, double *paramAddress, double defaultValue) {
  if(paramAddress == nullptr) {
    return false;
  }

  this->declare_parameter<double>(paramName, defaultValue);
  _paramAddressTable.insert(paramName, paramAddress);
  return true;
}

bool Navigation::addDoubleParam(QMap<std::string, std::pair<double *, double> > paramTable) {
  QList<std::string> keys = paramTable.keys();
  if(keys.size() < 1) {
    return false;
  }

  for(const auto & key : keys) {
    this->declare_parameter<double>(key, paramTable.value(key).second);
    _paramAddressTable.insert(key, paramTable.value(key).first);
  }
  return true;
}

double Navigation::calculateCurveLength(QLinkedList<Vector> path) const {
  if(path.size() < 2) {
    return 0.0;
  }

  int size = path.size();
  double ret = 0.0;
  Vector currentPos = path.takeFirst();
  Vector nextPos = path.takeFirst();
  for(int i = 0; i < size-2; i++) {
    ret += Utils::distance(currentPos, nextPos);
    currentPos = nextPos;
    nextPos = path.takeFirst();
  }

  return ret;
}

double Navigation::calculateConstrainedSpeed(double speed, double lastSpeed, double minValue, double maxValue,
                                             double maxAcceleration, double elapsedTime) {
  // Limit speed value
  if(speed < minValue) {
    speed = minValue;
  } else if(speed > maxValue) {
    speed = maxValue;
  }

  // Apply acceleration constraints
  if(fabs(lastSpeed - speed)/elapsedTime > maxAcceleration) {
    if(lastSpeed - speed > 0) {
      speed = lastSpeed - maxAcceleration*elapsedTime;
    } else {
      speed = lastSpeed + maxAcceleration*elapsedTime;
    }
  }

  return speed;
}
