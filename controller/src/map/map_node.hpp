#ifndef MAPNODE_HPP
#define MAPNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <QtCore>
#include <ctime>

// Vision messages
#include "ctr_msgs/msg/ball.hpp"
#include "ctr_msgs/msg/robot.hpp"
#include "ctr_msgs/msg/visionpkg.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/positionrequest.hpp"

// My libs
#include "worldmap.hpp"
#include "gui/soccerview.hh"
#include "../utils/field.hpp"

class Map_Node : public rclcpp::Node {
private:
  qint8 _id;
  std::string _team;
  WorldMap *_wm;
  Field *_field;
  timespec _start, _stop;
  double _time_now;

  // Graphical client
  GLSoccerView *_view;

  // Callback groups to separate work into queues
  rclcpp::CallbackGroup::SharedPtr _callback_group_vision;
  rclcpp::CallbackGroup::SharedPtr _callback_worldmap_update;
  rclcpp::CallbackGroup::SharedPtr _callback_information_services;
  rclcpp::CallbackGroup::SharedPtr _callback_gui;

  // ROS interfaces
  rclcpp::Subscription<ctr_msgs::msg::Visionpkg>::SharedPtr _subVision;
  rclcpp::TimerBase::SharedPtr _timerUpdate;
  rclcpp::Service<ctr_msgs::srv::Inforequest>::SharedPtr _infoService;
  rclcpp::Service<ctr_msgs::srv::Positionrequest>::SharedPtr _posService;
  rclcpp::TimerBase::SharedPtr _timerGUI;

  void updateGUI();
  void visionCallback(const ctr_msgs::msg::Visionpkg::SharedPtr msg);
  void updateWorldMap();
  void getInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<ctr_msgs::srv::Inforequest::Request> request,
                      const std::shared_ptr<ctr_msgs::srv::Inforequest::Response> response);
  void getPosition(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<ctr_msgs::srv::Positionrequest::Request> request,
                      const std::shared_ptr<ctr_msgs::srv::Positionrequest::Response> response);
public:
  Map_Node(const std::string team, const int id, Field *f, int frequency = 60);
  ~Map_Node();
};

#endif // MAPNODE_HPP
