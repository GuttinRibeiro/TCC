#include "map_node.hpp"
#include <inttypes.h>
#include <memory>
#include <chrono>
#include "../utils/groups.hpp"

Map_Node::Map_Node(const std::string team, const int id, const std::string side, WorldMap *wm, Field *field, int frequency) : rclcpp::Node ("map_"+team+std::to_string(id)), Entity (frequency) {
  _team = team;
  _id = (qint8)id;
  _wm = wm;
  _side = side;
  _field = field;
  _time_now = this->get_clock()->now().seconds();

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_vision = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_information_services = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_field_information = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize ROS 2 interaces
  // Vision
  auto vision_opt = rclcpp::SubscriptionOptions();
  vision_opt.callback_group = _callback_group_vision;
  vision_opt.topic_stats_options.publish_period = std::chrono::minutes(2);
  vision_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  vision_opt.topic_stats_options.publish_topic = "/vision/"+robotToken+"/statistics";
  _subVision = this->create_subscription<ctr_msgs::msg::Visionpkg>("vision/"+robotToken,
                                                                 rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                                                                 std::bind(&Map_Node::visionCallback, this, std::placeholders::_1),
                                                                 vision_opt);


  auto path_opt = rclcpp::SubscriptionOptions();
  path_opt.callback_group = _callback_group_vision;
  path_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  path_opt.topic_stats_options.publish_topic = "/vision/path/"+robotToken+"/statistics";
  _subPath = this->create_subscription<ctr_msgs::msg::Path>("vision/path/"+robotToken, rclcpp::QoS(10),
                                                                 std::bind(&Map_Node::pathUpdateCallback, this, std::placeholders::_1),
                                                                 path_opt);

  // Information services
  _infoService = this->create_service<ctr_msgs::srv::Idrequest>("map_service/"+robotToken+"/id",
                                                                  std::bind(&Map_Node::getInformation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                  rmw_qos_profile_services_default,
                                                                  _callback_information_services);
  _posService = this->create_service<ctr_msgs::srv::Elementrequest>("map_service/"+robotToken+"/position",
                                                                 std::bind(&Map_Node::getElement, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                 rmw_qos_profile_services_default,
                                                                 _callback_information_services);
  _fieldService = this->create_service<ctr_msgs::srv::Fieldinformationrequest>("map_service/"+robotToken+"/field",
                                                                               std::bind(&Map_Node::getFieldInformation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                               rmw_qos_profile_services_default,
                                                                               _callback_field_information);
  // Start this entity's thread
  this->start();
}

Map_Node::~Map_Node() {
  delete _wm;
}

void Map_Node::visionCallback(const ctr_msgs::msg::Visionpkg::SharedPtr msg) {
  //Update ball position:
  if(msg->balls.size() > 0) {
    _wm->updateElement(Groups::BALL, 0, 0.01, 0.0, Vector(msg->balls.at(0).x, msg->balls.at(0).y, msg->header.stamp.sec, false, msg->balls.at(0).confidence));
  }

  //Update robots:
  while(msg->robots.size() > 0) {
    auto robot = msg->robots.at(msg->robots.size()-1);
    msg->robots.pop_back();
    // Check team color:
    if(robot.team == "blue") {
      _wm->updateElement(Groups::BLUE, robot.id, 0.09, robot.orientation, Vector(robot.pos.x, robot.pos.y, msg->header.stamp.sec, false, robot.pos.confidence));
    } else if(robot.team == "yellow") {
      _wm->updateElement(Groups::YELLOW, robot.id, 0.09, robot.orientation, Vector(robot.pos.x, robot.pos.y, msg->header.stamp.sec, false, robot.pos.confidence));
    }
  }
}

void Map_Node::pathUpdateCallback(const ctr_msgs::msg::Path::SharedPtr msg) {
  QLinkedList<Vector> path;
  for(int i = 0; i < msg->path.size(); i++) {
    Vector vet(msg->path.at(i).x, msg->path.at(i).y, msg->path.at(i).z, false);
    path.append(vet);
  }

  if(path.size() > 0) {
    if(_team == "blue") {
      _wm->addPath(Groups::BLUE, _id, path);
    } else if(_team == "yellow") {
      _wm->addPath(Groups::YELLOW, _id, path);
    } else {
      std::cout << "[Map] Incorrect attempt to update path due to an invalid group provided\n";
    }
  }
}

void Map_Node::configure() {

}

void Map_Node::run() {
  // Check all timestamps and remove old information
  _time_now = this->get_clock()->now().seconds();
  _wm->checkElements(_time_now);
}

void Map_Node::getInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<ctr_msgs::srv::Idrequest::Request> request,
                              const std::shared_ptr<ctr_msgs::srv::Idrequest::Response> response) {
  (void) request_header;
  QList<qint8> ids = _wm->getGroup(request->group).keys();
  for(int i = 0; i < ids.size(); i++) {
    response->ids.push_back(ids[i]);
  }
}

void Map_Node::getElement(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<ctr_msgs::srv::Elementrequest::Request> request,
                           const std::shared_ptr<ctr_msgs::srv::Elementrequest::Response> response) {
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
  response->radius = _wm->getElement(request->group, request->id).radius();
}

void Map_Node::getFieldInformation(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<ctr_msgs::srv::Fieldinformationrequest::Request> request,
                                   const std::shared_ptr<ctr_msgs::srv::Fieldinformationrequest::Response> response) {
  (void) request_header;
  std::string req = request->request;

  response->error = false;
  if(req == "our goal") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = 0.0;
      response->pos.x = _field->minX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = 0.0;
      response->pos.x = _field->maxX();
    } else {
      response->error = true;
    }
  } else if(req == "their goal") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = 0.0;
      response->pos.x = _field->maxX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = 0.0;
      response->pos.x = _field->minX();
    } else {
      response->error = true;
    }
  } else if(req == "field center") {
    response->pos.isvalid = true;
    response->pos.z = 0.0;
    response->pos.y = 0.0;
    response->pos.x = 0.0;
  } else if(req == "our goal left post") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = -_field->goalWidth()/2;
      response->pos.x = _field->minX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->goalWidth()/2;
      response->pos.x = _field->maxX();
    } else {
      response->error = true;
    }
  } else if(req == "our goal right post") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->goalWidth()/2;
      response->pos.x = _field->minX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = -_field->goalWidth()/2;
      response->pos.x = _field->maxX();
    } else {
      response->error = true;
    }
  } else if(req == "their goal left post") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = -_field->goalWidth()/2;
      response->pos.x = _field->maxX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->goalWidth()/2;
      response->pos.x = _field->minX();
    } else {
      response->error = true;
    }
  } else if(req == "their goal right post") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = -_field->goalWidth()/2;
      response->pos.x = _field->maxX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->goalWidth()/2;
      response->pos.x = _field->minX();
    } else {
      response->error = true;
    }
  } else if(req == "our field left corner") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->minY();
      response->pos.x = _field->minX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->maxY();
      response->pos.x = _field->maxX();
    } else {
      response->error = true;
    }
  } else if(req == "our field right corner") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->maxY();
      response->pos.x = _field->minX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->minY();
      response->pos.x = _field->maxX();
    } else {
      response->error = true;
    }
  } else if(req == "their field left corner") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->maxY();
      response->pos.x = _field->maxX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->minY();
      response->pos.x = _field->minX();
    } else {
      response->error = true;
    }
  } else if(req == "their field right corner") {
    if(_side == "left") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->minY();
      response->pos.x = _field->maxX();
    } else if (_side == "right") {
      response->pos.z = 0;
      response->pos.isvalid = true;
      response->pos.y = _field->maxY();
      response->pos.x = _field->minX();
    } else {
      response->error = true;
    }
  } else if(req == "center radius") {
    response->value = _field->centerRadius();
  } else if(req == "defense length") {
    response->value = _field->defenseLength();
  } else if(req == "defense width") {
    response->value = _field->defenseWidth();
  } else if(req == "field length") {
    response->value = _field->length();
  } else if(req == "field width") {
    response->value = _field->width();
  } else if(req == "goal width") {
    response->value = _field->goalWidth();
  } else if(req == "goal depth") {
    response->value = _field->goalDepth();
  } else if(req == "min x") {
    response->value = _field->minX();
  } else if(req == "max x") {
    response->value =_field->maxX();
  } else if(req == "min y") {
    response->value = _field->minY();
  } else if(req == "max y") {
    response->value = _field->maxY();
  } else {
    response->error = true;
  }
}
