#ifndef WORLDMAP_HPP
#define WORLDMAP_HPP

#include <QtCore>
#include "../utils/element.hpp"
#include <iostream>

class WorldMap {
private:
  QMutex _mutex;
  QHash<int, QHash<qint8, Element>> _elements;
  float _threshold;
public:
  WorldMap(float persistense_time) {
    _mutex.unlock();
    _threshold = persistense_time/1000;
  }
  void checkElements(const double &now) {
    _mutex.lock();
    QList<int> groups = _elements.keys();
    // For each group:
    for(int i = 0; i < groups.size(); i++) {
      QList<qint8> ids = _elements.value(groups.at(i)).keys();
      // For each element:
      QHash<qint8, Element> group = _elements.value(groups.at(i));
      for(int j = 0; j < ids.size(); j++) {
        Element elem = group.value(ids[j]);
        // If data are too old:
        if(now - elem.position().timestamp() > _threshold) {
          group.remove(elem.id());
        }
      }
      _elements.insert(groups.at(i), group);
    }
    _mutex.unlock();
  }
  void updateElement(const int &group, const qint8 &id, const float &radius, const float &orientation, Vector position) {
    _mutex.lock();
    std::string groupName;
    Element elem;
    elem.setPosition(position);
    elem.setOrientation(orientation);
    elem.setId(id);
    elem.setGroup(group);
    elem.setRadius(radius);
    // TODO: como calcular a velocidade?

    QHash<qint8, Element> aux = _elements.value(group);
    aux.insert(id, elem);
    _elements.insert(group, aux);
    _mutex.unlock();
  }
  void addPath(const int &group, const qint8 &id, const QLinkedList<Vector> &path) {
    if(_elements.contains(group)) {
      _mutex.lock();
      QHash<qint8, Element> aux = _elements.value(group);
      if(aux.contains(id)) {
        Element elem = aux.value(id);
        elem.addPath(path);
        aux.insert(id, elem);
      } else {
        std::cout << "[WorldMap] The required group does not contain the specified element\n";
      }
      _elements.insert(group, aux);
      _mutex.unlock();
    } else {
       std::cout << "[WorldMap] The required group does not exist\n";
    }
  }
  Element getElement(const int &group, const qint8 &id) {
    _mutex.lock();
    Element ret;
    if(_elements.contains(group) && _elements.value(group).contains(id)) {
      ret = _elements.value(group).value(id);
    }
    _mutex.unlock();
    return ret;
  }
  QHash<qint8, Element> getGroup(const int &group) {
    _mutex.lock();
    QHash<qint8, Element> ret = _elements.value(group);
    _mutex.unlock();
    return ret;
  }
};

#endif // WORLDMAP_HPP
