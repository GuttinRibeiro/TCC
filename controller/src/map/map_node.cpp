#include "map_node.hpp"
#include <inttypes.h>
#include <memory>
#include <chrono>
#include "../utils/groups.hpp"

Map_Node::Map_Node(const std::string team, const int id, int frequency) : Node("map_"+team+std::to_string(id)) {
  _team = team;
  _id = (qint8)id;
  _wm = new WorldMap(5*1000/frequency);
  _time_now = 0.0;
  clock_gettime(CLOCK_REALTIME, &_start);

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_vision = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_worldmap_update = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_information_services = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize ROS 2 interaces
  // Vision
  auto vision_opt = rclcpp::SubscriptionOptions();
  vision_opt.callback_group = _callback_group_vision;
  _subVision = this->create_subscription<ctr_msgs::msg::Visionpkg>("vision/"+robotToken, rclcpp::QoS(10),
                                                                 std::bind(&Map_Node::visionCallback, this, std::placeholders::_1),
                                                                 vision_opt);
  // World Map timer
  _timerUpdate = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Map_Node::updateWorldMap, this), _callback_worldmap_update);

  // Information services
  _infoService = this->create_service<ctr_msgs::srv::Inforequest>("map_service/"+robotToken+"/info",
                                                                  std::bind(&Map_Node::getInformation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                  rmw_qos_profile_services_default,
                                                                  _callback_information_services);
  _posService = this->create_service<ctr_msgs::srv::Positionrequest>("map_service/"+robotToken+"/position",
                                                                 std::bind(&Map_Node::getPosition, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                 rmw_qos_profile_services_default,
                                                                 _callback_information_services);
}

Map_Node::~Map_Node() {
  delete _wm;
}

void Map_Node::visionCallback(const ctr_msgs::msg::Visionpkg::SharedPtr msg) {
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

void Map_Node::updateWorldMap() {
  // Stop timer
  clock_gettime(CLOCK_REALTIME, &_stop);
  // Count how many seconds have passed since last packet received
  _time_now += ((_stop.tv_sec*1E9+_stop.tv_nsec)-(_start.tv_sec*1E9+_start.tv_nsec))/1E9;
  // Check all timestamps and remove old information
  _wm->checkElements(_time_now);
}

void Map_Node::getInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<ctr_msgs::srv::Inforequest::Request> request,
                              const std::shared_ptr<ctr_msgs::srv::Inforequest::Response> response) {
  (void) request_header;
  QList<qint8> ids = _wm->getGroup(request->group).keys();
  for(int i = 0; i < ids.size(); i++) {
    response->ids.push_back(ids[i]);
  }
}

void Map_Node::getPosition(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<ctr_msgs::srv::Positionrequest::Request> request,
                           const std::shared_ptr<ctr_msgs::srv::Positionrequest::Response> response) {
  (void) request_header;
  response->pos.x = _wm->getElement(request->group, request->id).position().x();
  response->pos.y = _wm->getElement(request->group, request->id).position().y();
  response->pos.z = _wm->getElement(request->group, request->id).position().z();
  response->pos.isvalid = !_wm->getElement(request->group, request->id).position().isUnknown();
  response->orientation = _wm->getElement(request->group, request->id).orientation();
  if (response->orientation == 0.0f) {
    response->hasorientation = false;
  } else {
    response->hasorientation = true;
  }
}
