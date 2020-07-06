#include "ssl_controller.hpp"
#include <string>
#include "../utils/field.hpp"
#include <cmath>
#include <iostream>
#include "../utils/utils.hpp"

SSL_Controller::SSL_Controller(std::string team, int id, int frequency) : Controller (team, id, frequency) {
  _kick = false;
  _spinner = false;
  _kickSpeed = 0.0;
  _kickAngle = 0.0;
}

SSL_Controller::~SSL_Controller() {

}

void SSL_Controller::updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                                 const std::shared_ptr<ctr_msgs::srv::State::Response> response) {
  (void) request_header;
  response->feedback = "State updated successfully";
}

void SSL_Controller::run() {
  spinner();
  kick(1.0f);
  sendVelocity(0.0f, 0.05f, 0.1f);
}

void SSL_Controller::kick(float kickPower) {
  _kick = true;
  _kickSpeed = kickPower;
}

void SSL_Controller::chipkick(float kickPower, float kickAngle) {
  _kick = true;
  _kickSpeed = kickPower;
  _kickAngle = kickAngle;
}

void SSL_Controller::spinner() {
  _spinner = true;
}

void SSL_Controller::sendVelocity(float vx, float vy, float vang) {
  auto message = ctr_msgs::msg::Command();
  message.id = _id;
  message.team = _team;
  message.vx = vx;
  message.vy = vy;
  message.vang = vang;

  if(_kick && _kickAngle == 0.0f) {
    message.kickspeedz = 0.0;
    message.kickspeedy = _kickSpeed;
    message.chipkick = false;
  } else if(_kick) {
    message.kickspeedy = _kickSpeed*cos(_kickAngle);
    message.kickspeedz = _kickSpeed*sin(_kickAngle);
    message.chipkick = true;
  }

  if(_spinner) {
    message.spinner = true;
    _spinner = false;
  }

  _kick = false;
  _kickAngle = 0.0;
  _kickSpeed = 0.0;

  this->sendCommand(&message);
}
