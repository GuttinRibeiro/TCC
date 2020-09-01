#ifndef PF_HPP
#define PF_HPP

#include "navigation_algorithm.hpp"
#include <string>
#include <QList>

class PF : public Navigation_Algorithm {
private:
  float _x_shift;
  float _y_shift;
  float _factor;
  float _k;

  QLinkedList<Vector> updatePathTracking(Vector currentPosition, QLinkedList<Vector> currentPath);
  bool checkCurrentPath(Vector currentPosition, float currentOrientation,
                        QLinkedList<Vector> path, QList<Vector> obstacles);
  QLinkedList<Vector> calculatePath(Vector currentPosition, float currentOrientation,
                                    QLinkedList<Vector> oldPath, QList<Vector> obstacles,
                                    Vector destination, float orientation);
  std::string name() {return "Potential Fields";}
  Vector calculateForce(Vector robot, Vector element);
  bool isOnCollisionRoute(Vector currentPosition, Vector destination, Vector resultantForce, Vector obstacle, float radius);
public:
  PF(InfoBus *ib, qint8 id, int frequency = 60);
  ~PF();
};

#endif // PF_HPP
