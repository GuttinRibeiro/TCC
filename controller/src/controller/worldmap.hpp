#ifndef WORLDMAP_HPP
#define WORLDMAP_HPP

#include <QHash>
#include "../utils/element.hpp"

class WorldMap {
  private:
    QHash<int, QHash<qint8, Element>> _elements;
  public:
    WorldMap();
};

#endif // WORLDMAP_HPP
