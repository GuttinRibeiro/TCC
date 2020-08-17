#ifndef PF_HPP
#define PF_HPP

#include "navigation_algorithm.hpp"
#include <string>
#include <QList>
#include <utility>

class PF : public Navigation_Algorithm {
private:
  std::pair<float, float> _repulsionConstants;
  std::pair<float, float> _attractiveConstants;

  QLinkedList<Vector> updatePathTracking(Vector currentPosition, QLinkedList<Vector> currentPath);
  bool checkCurrentPath(Vector currentPosition, float currentOrientation,
                        QLinkedList<Vector> path, QList<Vector> obstacles);
  QLinkedList<Vector> calculatePath(Vector currentPosition, float currentOrientation,
                                    QLinkedList<Vector> oldPath, QList<Vector> obstacles,
                                    Vector destination, float orientation);
  std::string name() {return "Potential Fields";}
  Vector calculateForce(Vector robot, Vector element, bool isRepulsive);
public:
  PF(InfoBus *ib, qint8 id, int frequency = 60);
  ~PF();
};

#endif // PF_HPP
