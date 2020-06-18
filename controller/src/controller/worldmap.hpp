#ifndef WORLDMAP_HPP
#define WORLDMAP_HPP

#include <QtCore>
#include "../utils/element.hpp"

class WorldMap {
private:
  QMutex _mutex;
  QHash<int, QHash<qint8, Element>> _elements;
public:
  WorldMap() {_mutex.unlock();}
  void updateElement(const int &group, const qint8 &id, const float &radius, const float &orientation, Vector position) {
    _mutex.lock();
    Element elem;
    elem.setPosition(position);
    elem.setOrientation(orientation);
    elem.setId(id);
    elem.setGroup(group);
    elem.setRadius(radius);
    // TODO: como calcular a velocidade?

    QHash<qint8, Element> aux;
    aux.insert(id, elem);
    _elements.insert(group, aux);
    _mutex.unlock();
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
