#include "navigation.hpp"
#include "../utils/utils.hpp"

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"

Navigation::Navigation(std::string team, int id, int frequency) : rclcpp::Node("navigation_"+team+"_"+std::to_string(id)), Entity(frequency) {
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
}

Navigation::~Navigation() {
  delete _ib;
  delete _navAlg;
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
}

void Navigation::callback(ctr_msgs::msg::Navigation::SharedPtr msg) {
  _orientation = msg->orientation;
  _destination.setX(msg->destination.x);
  _destination.setY(msg->destination.y);
  _destination.setZ(msg->destination.z);
  _destination.setIsUnknown(false);
}

ctr_msgs::msg::Path Navigation::generatePathMessage(QLinkedList<Vector> path) const {
  ctr_msgs::msg::Path msg;
  for(int i = 0; i < path.size(); i++) {
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
  _navAlg->setDestination(_destination);
  _navAlg->setOrientation(_orientation);
  QLinkedList<Vector> path = _navAlg->path();
  if(path.size() > 0) {
    _pubPath->publish(generatePathMessage(path));
  } else {
    std::cout << "[Navigation] Empty path received\n";
  }
}
