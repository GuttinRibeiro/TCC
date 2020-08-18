#include "ssl_controller.hpp"
#include <string>
#include "../utils/field.hpp"
#include <cmath>
#include <iostream>
#include "../utils/utils.hpp"
#include <ctime>
#include "ctr_msgs/msg/command.hpp"
#define KICK_PRECISION 0.01

SSL_Controller::SSL_Controller(std::string team, int id, int frequency) : Controller (team, id, frequency) {
  _holdBall = false;
  _kickSpeedY = 0.0;
  _kickSpeedZ = 0.0;
}

SSL_Controller::~SSL_Controller() {

}

void SSL_Controller::updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                                 const std::shared_ptr<ctr_msgs::srv::State::Response> response) {
  (void) request_header;
  response->feedback = "State updated successfully";
}

void SSL_Controller::configure() {
  std::cout << "SSL controller configured!\n";
}

void SSL_Controller::run() {
  std::cout << "SSL Controller running!\n";
  kick(1.0f);
  Vector pos = infoBus()->theirGoal();
  goTo(pos);
//  timespec start, stop;
//  clock_gettime(CLOCK_REALTIME, &start);
//  Vector mypos = infoBus()->myPosition();
//  clock_gettime(CLOCK_REALTIME, &stop);
//  std::cout << "Time to request information: " << ((stop.tv_sec*1E9+stop.tv_nsec)-(start.tv_sec*1E9+start.tv_nsec))/1E9 << " s\n";
//  std::cout << "My position: " << mypos.x() << ", " << mypos.y() << "\n";
//  Vector pos = infoBus()->ourGoalRightPost();
//  std::cout << "Our goal Right post: " << pos.x() << ", " << pos.y() << "\n";
}

void SSL_Controller::kick(float kickPower) {
  if(fabs(_kickSpeedY-kickPower) > KICK_PRECISION) {
    _kickSpeedY = kickPower;
    _kickSpeedZ = 0.0;
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = true;
    cmd.hasholdballinformation = false;
    cmd.kickspeedz = 0.0;
    cmd.kickspeedy = kickPower;
    send_command(cmd);
  }
}

void SSL_Controller::chipkick(float kickPower, float kickAngle) {
  if(fabs(_kickSpeedY-kickPower) > KICK_PRECISION || fabs(_kickSpeedZ-kickPower) > KICK_PRECISION) {
    _kickSpeedY = kickPower*cos(kickAngle);
    _kickSpeedZ = kickPower*sin(kickAngle);
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = true;
    cmd.hasholdballinformation = false;
    cmd.kickspeedz = _kickSpeedZ;
    cmd.kickspeedy = _kickSpeedY;
    send_command(cmd);
  }
}

void SSL_Controller::holdBall(bool turnOn) {
  if(turnOn != _holdBall) {
    _holdBall = turnOn;
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = false;
    cmd.hasholdballinformation = true;
    cmd.holdball = turnOn;
    send_command(cmd);
  }
}

ctr_msgs::msg::Navigation SSL_Controller::encodeNavMessage(Vector destination, float orientation, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
  ctr_msgs::msg::Position desiredPos;
  desiredPos.isvalid = true;
  desiredPos.x = destination.x();
  desiredPos.y = destination.y();
  desiredPos.z = destination.z();
  ctr_msgs::msg::Navigation ret;
  ret.destination = desiredPos;
  ret.orientation = orientation;
  ret.avoidball = avoidBall;
  ret.avoidallies = avoidAllies;
  ret.avoidenemies = avoidEnemies;
  return ret;
}
