#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <QtCore>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "../utils/entity.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"
#include "ctr_msgs/msg/velocity.hpp"
#include "ctr_msgs/msg/navigation.hpp"
#include "ctr_msgs/msg/path.hpp"
#include "../controller/infobus.hpp"
#include "../utils/vector.hpp"
#include "navalg/pf.hpp"
#include "ctralg/discrete_pid.hpp"

class Navigation : public rclcpp::Node, public Entity {
private:
  rclcpp::CallbackGroup::SharedPtr _callback_group_map;
  rclcpp::CallbackGroup::SharedPtr _callback_group_server;
  rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
  rclcpp::CallbackGroup::SharedPtr _callback_group_nav_messages;
  rclcpp::CallbackGroup::SharedPtr _callback_group_parameters;

  rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr _clientElementRequest;
  rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr _clientInfoRequest;
  rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;
  rclcpp::Publisher<ctr_msgs::msg::Path>::SharedPtr _pubPath;
  rclcpp::Publisher<ctr_msgs::msg::Velocity>::SharedPtr _pubActuator;
  rclcpp::Subscription<ctr_msgs::msg::Navigation>::SharedPtr _subNavMessages;

  InfoBus *_ib;
  Vector _destination;
  float _orientation;
  Navigation_Algorithm *_navAlg;
  Control_Algorithm *_linCtrAlg;
  Control_Algorithm *_angCtrAlg;
  int _frequency;
  double _maxLinearSpeed;
  double _maxLinearAcceleration;
  double _maxAngularSpeed;
  double _maxAngularAcceleration;
  qint8 _id;
  std::string _team;
  OnSetParametersCallbackHandle::SharedPtr _handler;
  QMap<std::string, double *> _paramAddressTable;

  std::string name() {return "Navigation";}
  void configure();
  void run();
  void callback(ctr_msgs::msg::Navigation::SharedPtr msg);
  ctr_msgs::msg::Path generatePathMessage(QLinkedList<Vector> path) const;
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & parameters);
  void sendVelocity(float vx, float vy, float vang);
public:
  Navigation(std::string team, int id, int frequency = 60);
  bool addDoubleParam(std::string paramName, double *paramAddress, double defaultValue);
  bool addDoubleParam(QMap<std::string, std::pair<double *, double>> paramTable);
  ~Navigation();
};

#endif // NAVIGATION_HPP
