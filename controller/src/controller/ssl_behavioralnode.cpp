#include "ssl_behavioralnode.hpp"
#include <string>
#include "../utils/field.hpp"
#include <cmath>
#include <iostream>
#include "../utils/utils.hpp"
#include <ctime>
#include "ctr_msgs/msg/command.hpp"

SSL_BehavioralNode::SSL_BehavioralNode(std::string team, int id, int frequency) : BehavioralNode (team, id, frequency) {
  _current = new State_Halt(this);
}

SSL_BehavioralNode::~SSL_BehavioralNode() {
  delete _current;
}

void SSL_BehavioralNode::updateState(ctr_msgs::msg::State::SharedPtr msg) {
  std::cout << "[SSL Controller] New state: " << msg->state << "\n";
  bool gameOn = (msg->state == "FORCESTART" || msg->state == "NORMALSTART");
  if(msg->isgk && gameOn) {
    nextState(States::GK);
    std::cout << "State gk\n";
    return;
  }

  if(gameOn) {
    nextState(States::ATK);
    std::cout << "State atk\n";
    return;
  }

  nextState(States::HALT);
  std::cout << "State halt\n";
}

void SSL_BehavioralNode::configure() {
  std::cout << "SSL controller was configured!\n";
}

void SSL_BehavioralNode::run() {
//  std::cout << "SSL Controller running!\n";
  _current->runState();
}

ctr_msgs::msg::Navigation SSL_BehavioralNode::encodeNavMessage(Vector destination, float orientation, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
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
  ret.header.stamp = this->get_clock()->now();
  return ret;
}

void SSL_BehavioralNode::nextState(int nextStateName) {
  State *oldState = _current;

  switch (nextStateName) {
  case States::HALT : {
    _current = new State_Halt(this);
    break;
  }

  case States::GK : {
    _current = new State_GK(this);
    break;
  }

  default:
    _current = new State_Halt(this);
  }


  delete oldState;
}
