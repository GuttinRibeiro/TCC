#ifndef INFO_HPP
#define INFO_HPP

#include "worldmap.hpp"
#include <string>
#include <QList>
#include "../utils/vector.hpp"

class InfoBus {
private:
  WorldMap *_wm;
  int _group;
  qint8 _id;

public:
  InfoBus(WorldMap *wm, qint8 id, std::string team);

  // Myself
  Vector myPosition() const;
  Vector myVelocity() const;
  float myRadius() const;
  float myOrientation() const;

  // Ball
  Vector ballPosition() const;
  Vector ballVelocity() const;
  float ballRadius() const;
  float ballOrientation() const;

  // Generic element
  QList<qint8> ourPlayers() const;
  QList<qint8> theirPlayers() const;
  Vector robotPosition(const int &group, const qint8 &id) const;
  Vector robotVelocity(const int &group, const qint8 &id) const;
  float robotRadius(const int &group, const qint8 &id) const;
  float robotOrientation(const int &group, const qint8 &id) const;
};

#endif // INFO_HPP
