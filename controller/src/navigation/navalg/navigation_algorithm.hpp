#ifndef NAVIGATION_ALGORITHM_HPP
#define NAVIGATION_ALGORITHM_HPP

#include "../../controller/infobus.hpp"
#include "../../utils/entity.hpp"
#include "../../utils/vector.hpp"
#include "../../utils/groups.hpp"
//#include "../../utils/element.hpp"
#include <QLinkedList>
#include <QMutex>
#include <QMap>
#include <utility>

class Navigation_Algorithm : public Entity {
private:
  InfoBus *_ib;
  QMutex _mutex;

protected:
  float _orientation;
  Vector _destination;
  int _ourGroup;
  int _theirGroup;
  QLinkedList<Vector> _path;
  QMap<std::pair<int, qint8>, Vector> _obstacles;

  InfoBus* ib() const {return _ib;}
  virtual void run() {return;}
  virtual void configure() {return;}
  virtual std::string name() {return "Navigation algorithm";}
  void removeRobotsFromObstacles(int group);
public:
  Navigation_Algorithm(InfoBus *ib, int frequency = 60);
  virtual ~Navigation_Algorithm();
  QLinkedList<Vector> path();
  void setDestination(Vector destination);
  void setOrientation(float orientation);
  void avoidBall(bool turnOn);
  void avoidRobots(bool turnOn);
  void avoidEnemies(bool turnOn);
  void avoidAllies(bool turnOn);
  void ignoreObstacles();
  void avoidAll();
};

#endif // NAVIGATION_ALGORITHM_HPP
