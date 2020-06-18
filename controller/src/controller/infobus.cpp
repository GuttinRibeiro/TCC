#include "infobus.hpp"
#include "../utils/groups.hpp"

InfoBus::InfoBus(WorldMap *wm, qint8 id, std::string team) {
  _wm = wm;
  _id = id;

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
  return _wm->getElement(_group, _id).position();
}

Vector InfoBus::myVelocity() const {
  return _wm->getElement(_group, _id).velocity();
}

float InfoBus::myRadius() const {
  return _wm->getElement(_group, _id).radius();
}

float InfoBus::myOrientation() const {
  return _wm->getElement(_group, _id).orientation();
}

Vector InfoBus::ballPosition() const {
  return _wm->getElement(Groups::BALL, 0).position();
}

Vector InfoBus::ballVelocity() const {
  return _wm->getElement(Groups::BALL, 0).velocity();
}

float InfoBus::ballRadius() const {
  return _wm->getElement(Groups::BALL, 0).radius();
}

float InfoBus::ballOrientation() const {
  return _wm->getElement(Groups::BALL, 0).orientation();
}

QList<qint8> InfoBus::ourPlayers() const {
  return  _wm->getGroup(_group).keys();
}

QList<qint8> InfoBus::theirPlayers() const {
  if(_group == Groups::BLUE) {
    return _wm->getGroup(_group).keys();
  }

  return _wm->getGroup(_group).keys();
}

Vector InfoBus::robotPosition(const int &group, const qint8 &id) const {
  return _wm->getElement(group, id).position();
}

Vector InfoBus::robotVelocity(const int &group, const qint8 &id) const {
  return _wm->getElement(group, id).velocity();
}

float InfoBus::robotRadius(const int &group, const qint8 &id) const {
  return _wm->getElement(group, id).radius();
}

float InfoBus::robotOrientation(const int &group, const qint8 &id) const {
  return _wm->getElement(group, id).orientation();
}
