#include "infobus.hpp"
#include "../utils/groups.hpp"

InfoBus::InfoBus(qint8 id, std::string team,
                 rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr *clientPosition,
                 rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr *clientInformation,
                 rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr *clientField) {
  _id = id;
  _clientPosition = clientPosition;
  _clientInformation = clientInformation;
  _clientField = clientField;

  // Check team color
  if(team == "blue") {
    _group = Groups::BLUE;
  } else if(team == "yellow") {
    _group = Groups::YELLOW;
  } else {
    _group = Groups::UNKNOWN;
  }
}

Vector InfoBus::myPosition() const {
  return robotPosition(_group, _id);
}

float InfoBus::myRadius() const {
  return robotRadius(_group, _id);
}

float InfoBus::myOrientation() const {
  return robotOrientation(_group, _id);
}

Vector InfoBus::ballPosition() const {
  return robotPosition(Groups::BALL, 0);
}

float InfoBus::ballRadius() const {
  return robotRadius(Groups::BALL, 0);
}

float InfoBus::ballOrientation() const {
  return robotOrientation(Groups::BALL, 0);
}

QList<qint8> InfoBus::ourPlayers() const {
  auto request = std::make_shared<ctr_msgs::srv::Inforequest::Request>();
  request->group = _group;

  // Wait for service:
  while (!(*_clientInformation)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Information service not available, waiting again...");
  }

  QList<qint8> ret;
  auto response = (*_clientInformation)->async_send_request(request);
  auto status = response.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready) {
    for(int i = 0; i < response.get()->ids.size(); i++) {
      ret.insert(i, response.get()->ids.at(i));
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request a list of ids");
  }

  return ret;
}

QList<qint8> InfoBus::theirPlayers() const {
  auto request = std::make_shared<ctr_msgs::srv::Inforequest::Request>();
  QList<qint8> ret;
  if(_group == Groups::BLUE) {
    request->group = Groups::YELLOW;
  } else if(_group == Groups::YELLOW) {
    request->group = Groups::BLUE;
  } else {
    std::cout << "[INFOBUS] Failed to request a list of ids due to an invalid group request\n";
    return ret;
  }

  auto response = (*_clientInformation)->async_send_request(request);
  auto status = response.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready) {
    for(int i = 0; i < response.get()->ids.size(); i++) {
      ret.insert(i, response.get()->ids.at(i));
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request a list of ids");
  }

  return ret;
}

Vector InfoBus::robotPosition(const int &group, const qint8 &id) const {
  auto request = std::make_shared<ctr_msgs::srv::Elementrequest::Request>();
  request->id = id;
  request->group = group;

  // Wait for service:
  while (!(*_clientPosition)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Position service not available, waiting again...");
  }

  Vector ret;
  auto response = (*_clientPosition)->async_send_request(request);
  auto status = response.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready) {
    ret.setX(response.get()->pos.x);
    ret.setY(response.get()->pos.y);
    ret.setZ(response.get()->pos.z);
    ret.setIsUnknown(!response.get()->pos.isvalid);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request position");
  }

  return ret;
}

float InfoBus::robotRadius(const int &group, const qint8 &id) const {
  auto request = std::make_shared<ctr_msgs::srv::Elementrequest::Request>();
  request->id = id;
  request->group = group;

  // Wait for service:
  while (!(*_clientPosition)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Position service not available, waiting again...");
  }

  float ret = -1;
  auto response = (*_clientPosition)->async_send_request(request);
  auto status = response.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready) {
    ret = response.get()->radius;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request radius");
  }

  return ret;
}

float InfoBus::robotOrientation(const int &group, const qint8 &id) const {
  auto request = std::make_shared<ctr_msgs::srv::Elementrequest::Request>();
  request->id = id;
  request->group = group;

  // Wait for service:
  while (!(*_clientPosition)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Position service not available, waiting again...");
  }

  float ret = -1;
  auto response = (*_clientPosition)->async_send_request(request);
  auto status = response.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready) {
    if(response.get()->hasorientation) {
      ret = response.get()->orientation;
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request orientation");
  }

  return ret;
}

float InfoBus::maxX() const {
  return requestFieldValue("max x");
}

float InfoBus::minY() const {
  return requestFieldValue("min y");
}

float InfoBus::minX() const {
  return requestFieldValue("min x");
}

float InfoBus::maxY() const {
  return requestFieldValue("max y");
}

float InfoBus::goalDepth() const {
  return requestFieldValue("goal depth");
}

float InfoBus::goalWidth() const {
  return requestFieldValue("goal width");
}

float InfoBus::fieldWidth() const {
  return requestFieldValue("field width");
}

float InfoBus::fieldLength() const {
  return requestFieldValue("field length");
}

float InfoBus::centerRadius() const {
  return requestFieldValue("center radius");
}

float InfoBus::defenseWidth() const {
  return requestFieldValue("defense width");
}

float InfoBus::defenseLength() const {
  return requestFieldValue("defense length");
}

float InfoBus::requestFieldValue(std::string info_requested) const {
  auto request = std::make_shared<ctr_msgs::srv::Fieldinformationrequest::Request>();
  request->request = info_requested;

  // Wait for service:
  while (!(*_clientField)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Field service not available, waiting again...");
  }

  auto respose = (*_clientField)->async_send_request(request);
  auto status = respose.wait_for(std::chrono::seconds(1));
  if(status == std::future_status::ready && respose.get()->error == false) {
    return respose.get()->value;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request a field value");
  return -1.0;
}

Vector InfoBus::ourGoal() const {
  return requestFieldPosition("our goal");
}

Vector InfoBus::theirGoal() const {
  return requestFieldPosition("their goal");
}

Vector InfoBus::ourGoalLeftPost() const {
  return requestFieldPosition("our goal left post");
}

Vector InfoBus::ourGoalRightPost() const {
  return requestFieldPosition("our goal right post");
}

Vector InfoBus::theirGoalLeftPost() const {
  return requestFieldPosition("their goal left post");
}

Vector InfoBus::theirGoalRightPost() const {
  return requestFieldPosition("their goal right post");
}

Vector InfoBus::fieldCenter() const {
  return requestFieldPosition("field center");
}

Vector InfoBus::ourFieldLeftCorner() const {
  return requestFieldPosition("our field left corner");
}

Vector InfoBus::ourFieldRightCorner() const {
  return requestFieldPosition("our field right corner");
}

Vector InfoBus::theirFieldLeftCorner() const {
  return requestFieldPosition("their field left corner");
}

Vector InfoBus::theirFieldRightCorner() const {
  return requestFieldPosition("their field right corner");
}

Vector InfoBus::requestFieldPosition(std::string info_requested) const {
  auto request = std::make_shared<ctr_msgs::srv::Fieldinformationrequest::Request>();
  request->request = info_requested;

  // Wait for service:
  while (!(*_clientField)->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Field service not available, waiting again...");
  }

  auto respose = (*_clientField)->async_send_request(request);
  auto status = respose.wait_for(std::chrono::seconds(1));
  Vector ret;
  if(status == std::future_status::ready && respose.get()->error == false) {
    ret.setX(respose.get()->pos.x);
    ret.setY(respose.get()->pos.y);
    ret.setZ(respose.get()->pos.z);
    ret.setIsUnknown(!respose.get()->pos.isvalid);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[INFOBUS] Failed to request a field value");
  }

  return ret;
}

int InfoBus::ourGroup() const {
  return _group;
}

int InfoBus::theirGroup() const {
  if(_group == Groups::BLUE) {
    return Groups::YELLOW;
  }

  if(_group == Groups::YELLOW) {
    return Groups::BLUE;
  }

  return Groups::UNKNOWN;
}
