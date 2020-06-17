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
  void updateElement(int group, qint8 id, float radius, float orientation, Vector position) {
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
  Element getElement(int group, qint8 id) {
    _mutex.lock();
    Element ret = _elements.value(group).value(id);
    _mutex.unlock();
    return ret;
  }
};

#endif // WORLDMAP_HPP
