#include "navigation_algorithm.hpp"
#include <iostream>
#include "../../utils/utils.hpp"
#define MAX_NUM_ROBOTS 6
#define FLOAT_ERROR 0.01

Navigation_Algorithm::Navigation_Algorithm(InfoBus *ib, qint8 id, int frequency) : Entity(frequency) {
  if(ib == nullptr) {
    std::cout << "[Nav Alg] Invalid pointer to InfoBus received! Aborting...\n";
    return;
  }
  _id = id;
  _ib = ib;
  _orientation = 0.0f;
  _myPosition = Vector();
  _newDesiredState = false;
  _myOrientation = 0.0f;
  _ourGroup = _ib->ourGroup();
  _theirGroup = _ib->theirGroup();
  _path.clear();
  _mutex.unlock();
  // Start this entity's thread
  this->start();
}

void Navigation_Algorithm::setOrientation(float orientation) {
//  _mutex.lock();
  if(fabs(_orientation - orientation) > FLOAT_ERROR) {
    _orientation = orientation;
    _newDesiredState = true;
  }
//  _mutex.unlock();
}

void Navigation_Algorithm::setDestination(Vector destination) {
  if(destination.isUnknown()) {
    std::cout << "[Nav Alg] Destination is unknown!\n";
    return;
  }

//  _mutex.lock();
  if(Utils::distance(_destination, destination) > FLOAT_ERROR || _destination.isUnknown()) {
    _destination = destination;
    _newDesiredState = true;
  }
//  _mutex.unlock();
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
  if(ids.contains(_id)) {
    ids.removeAll(_id);
  }
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
    if(ids.contains(_id)) {
      ids.removeAll(_id);
    }
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
    if(ids.contains(_id)) {
      ids.removeAll(_id);
    }
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
    QList<qint8> ids = _ib->theirPlayers();
    _mutex.lock();
    for(quint8 i = 0; i < ids.size(); i++) {
      _obstacles.insert(std::make_pair(_theirGroup, ids.at(i)), _ib->robotPosition(_theirGroup, ids.at(i)));
    }
    _mutex.unlock();
  }
}

void Navigation_Algorithm::run() {
  // Update robot state
  _myPosition = _ib->myPosition();
  _myOrientation = _ib->myOrientation();

  _mutex.lock();
  // Check if a new path should be calculated
  if(_newDesiredState) {
//    std::cout << "[Navigation] New final state received. Recalculating path...\n";
    _path = calculatePath(_myPosition, _myOrientation, _path, _obstacles.values(), _destination, _orientation);
//    std::cout << "[Navigation] New path calculated\n";
    _newDesiredState = false;
  } else {
    // Update path tracking
    _path  = updatePathTracking(_myPosition, _path);

    if(checkCurrentPath(_myPosition, _myOrientation, _path, _obstacles.values())) {
//      std::cout << "[Navigation] Calculating new path\n";
      _path = calculatePath(_myPosition, _myOrientation, _path, _obstacles.values(), _destination, _orientation);
    }
  }
  _mutex.unlock();
}
