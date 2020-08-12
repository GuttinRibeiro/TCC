#include "navigation_algorithm.hpp"
#include <iostream>
#define MAX_NUM_ROBOTS 6

Navigation_Algorithm::Navigation_Algorithm(InfoBus *ib, int frequency) : Entity(frequency) {
  if(ib == nullptr) {
    std::cout << "[Nav Alg] Invalid pointer to InfoBus received! Aborting...\n";
    return;
  }
  _ib = ib;
  _orientation = 0.0f;
  _ourGroup = _ib->ourGroup();
  _theirGroup = _ib->theirGroup();
  _path.clear();
  _mutex.unlock();
}

void Navigation_Algorithm::setOrientation(float orientation) {
  _mutex.lock();
  _orientation = orientation;
  _mutex.unlock();
}

void Navigation_Algorithm::setDestination(Vector destination) {
  if(destination.isUnknown()) {
    std::cout << "[Nav Alg] Destination is unknown!\n";
    return;
  }

  _mutex.lock();
  _destination = destination;
  _mutex.unlock();
}

QLinkedList<Vector> Navigation_Algorithm::path() {
  QLinkedList<Vector> ret;
  _mutex.lock();
  ret = _path;
  _mutex.unlock();
  return ret;
}

Navigation_Algorithm::~Navigation_Algorithm() {

}

void Navigation_Algorithm::ignoreObstacles() {
  _mutex.lock();
  _obstacles.clear();
  _mutex.unlock();
}

void Navigation_Algorithm::avoidAll() {
  // Get all visible players of our team
  QList<qint8> ids = _ib->ourPlayers();
  _mutex.lock();
  for(quint8 i = 0; i < ids.size(); i++) {
    _obstacles.insert(std::make_pair(_ourGroup, ids.at(i)), _ib->robotPosition(_ourGroup, ids.at(i)));
  }

  // Get all visible players of their team
  ids = _ib->theirPlayers();
  for(quint8 i = 0; i < ids.size(); i++) {
    _obstacles.insert(std::make_pair(_theirGroup, ids.at(i)), _ib->robotPosition(_theirGroup, ids.at(i)));
  }

  // Add ball
  Vector ballPos = _ib->ballPosition();
  if(ballPos.isUnknown() == false) {
    _obstacles.insert(std::make_pair(Groups::BALL, 0), _ib->robotPosition(Groups::BALL, 0));
  }
  _mutex.unlock();
}

void Navigation_Algorithm::avoidBall(bool turnOn) {
  if(turnOn) {
    Vector ballPos = _ib->ballPosition();
    if(ballPos.isUnknown() == false) {
      _mutex.lock();
      _obstacles.insert(std::make_pair(Groups::BALL, 0), ballPos);
      _mutex.unlock();
    }
  } else {
    if(_obstacles.contains(std::make_pair(Groups::BALL, 0))) {
      _mutex.lock();
      _obstacles.remove(std::make_pair(Groups::BALL, 0));
      _mutex.unlock();
    }
  }
}

void Navigation_Algorithm::removeRobotsFromObstacles(int group) {
  if(group == Groups::BLUE || group == Groups::YELLOW) {
    std::pair<int, qint8> key;
    _mutex.lock();
    for(quint8 i = 0; i < MAX_NUM_ROBOTS; i++) {
      key = std::make_pair(group, i);
      if(_obstacles.contains(key)) {
        _obstacles.remove(key);
      }
    }
    _mutex.unlock();
    return;
  }

  std::cout << "[Navigation algorithm] Invalid group request\n";
}

void Navigation_Algorithm::avoidRobots(bool turnOn) {
  // Remove robots from obstacle list to avoid consider
  // a robot that is not visible anymore
  removeRobotsFromObstacles(Groups::BLUE);
  removeRobotsFromObstacles(Groups::YELLOW);

  if(turnOn) {
    // Get all visible players of our team
    QList<qint8> ids = _ib->ourPlayers();
    _mutex.lock();
    for(quint8 i = 0; i < ids.size(); i++) {
      _obstacles.insert(std::make_pair(_ourGroup, ids.at(i)), _ib->robotPosition(_ourGroup, ids.at(i)));
    }

    // Get all visible players of their team
    ids = _ib->theirPlayers();
    for(quint8 i = 0; i < ids.size(); i++) {
      _obstacles.insert(std::make_pair(_theirGroup, ids.at(i)), _ib->robotPosition(_theirGroup, ids.at(i)));
    }
    _mutex.unlock();
  }
}

void Navigation_Algorithm::avoidAllies(bool turnOn) {
  removeRobotsFromObstacles(_ourGroup);

  if(turnOn) {
    // Get all visible players of our team
    QList<qint8> ids = _ib->ourPlayers();
    _mutex.lock();
    for(quint8 i = 0; i < ids.size(); i++) {
      _obstacles.insert(std::make_pair(_ourGroup, ids.at(i)), _ib->robotPosition(_ourGroup, ids.at(i)));
    }
    _mutex.unlock();
  }
}

void Navigation_Algorithm::avoidEnemies(bool turnOn) {
  removeRobotsFromObstacles(_theirGroup);

  if(turnOn) {
    // Get all visible players of their team
    QList<qint8> ids = _ib->ourPlayers();
    _mutex.lock();
    for(quint8 i = 0; i < ids.size(); i++) {
      _obstacles.insert(std::make_pair(_theirGroup, ids.at(i)), _ib->robotPosition(_theirGroup, ids.at(i)));
    }
    _mutex.unlock();
  }
}
