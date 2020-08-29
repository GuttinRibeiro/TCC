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
  float _orientation;
  Vector _destination;
  bool _newDesiredState;
  Vector _myPosition;
  float _myOrientation;
  int _ourGroup;
  int _theirGroup;
  qint8 _id;
  QLinkedList<Vector> _path;
  QMap<std::pair<int, qint8>, Vector> _obstacles;
protected:
  void run();
  virtual void configure() {return;}
  virtual std::string name() {return "Navigation algorithm";}
  void removeRobotsFromObstacles(int group);

  virtual QLinkedList<Vector> updatePathTracking(Vector currentPosition, QLinkedList<Vector> currentPath) = 0;
  virtual bool checkCurrentPath(Vector currentPosition, float currentOrientation, QLinkedList<Vector> path,
                                QList<Vector> obstacles) = 0;
  virtual QLinkedList<Vector> calculatePath(Vector currentPosition, float currentOrientation,
                                            QLinkedList<Vector> oldPath, QList<Vector> obstacles,
                                            Vector destination, float orientation) = 0;

public:
  Navigation_Algorithm(InfoBus *ib, qint8 id, int frequency = 60);
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
