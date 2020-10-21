#ifndef MAPNODE_HPP
#define MAPNODE_HPP

#include <string>
#include <QtCore>
#include "rclcpp/rclcpp.hpp"

// Vision messages
#include "ctr_msgs/msg/robot.hpp"
#include "ctr_msgs/msg/visionpkg.hpp"
#include "ctr_msgs/msg/path.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"

// My libs
#include "worldmap.hpp"
#include "../utils/field.hpp"
#include "../utils/entity.hpp"

class Map_Node : public rclcpp::Node, public Entity {
private:
  qint8 _id;
  std::string _team;
  std::string _side;
  WorldMap *_wm;
  Field *_field;
  double _time_now;

  // Callback groups to separate work into queues
  rclcpp::CallbackGroup::SharedPtr _callback_group_vision;
  rclcpp::CallbackGroup::SharedPtr _callback_information_services;
  rclcpp::CallbackGroup::SharedPtr _callback_field_information;

  // ROS interfaces
  rclcpp::Subscription<ctr_msgs::msg::Visionpkg>::SharedPtr _subVision;
  rclcpp::Subscription<ctr_msgs::msg::Path>::SharedPtr _subPath;
  rclcpp::Service<ctr_msgs::srv::Inforequest>::SharedPtr _infoService;
  rclcpp::Service<ctr_msgs::srv::Elementrequest>::SharedPtr _posService;
  rclcpp::Service<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _fieldService;

  std::string name() {return "Map Node";}
  void configure();
  void run();
  void visionCallback(const ctr_msgs::msg::Visionpkg::SharedPtr msg);
  void pathUpdateCallback(const ctr_msgs::msg::Path::SharedPtr msg);
  void updateWorldMap();
  void getInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<ctr_msgs::srv::Inforequest::Request> request,
                      const std::shared_ptr<ctr_msgs::srv::Inforequest::Response> response);
  void getElement(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<ctr_msgs::srv::Elementrequest::Request> request,
                      const std::shared_ptr<ctr_msgs::srv::Elementrequest::Response> response);
  void getFieldInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<ctr_msgs::srv::Fieldinformationrequest::Request> request,
                           const std::shared_ptr<ctr_msgs::srv::Fieldinformationrequest::Response> response);
public:
  Map_Node(const std::string team, const int id, const std::string side, WorldMap *wm, Field *field, int frequency = 60);
  ~Map_Node();
};

#endif // MAPNODE_HPP
