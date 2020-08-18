#include "pf.hpp"
#include "../../utils/utils.hpp"
#include <cmath>
#define RESOLUTION 0.1
#define MAX_DISTANCE 5

PF::PF(InfoBus *ib, qint8 id, int frequency) : Navigation_Algorithm (ib, id, frequency) {
  _repulsionConstants.first = -0.1;
  _repulsionConstants.second = 1.5;
  _attractiveConstants.first = 5;
  _attractiveConstants.second = 2;
}

PF::~PF() {

}

QLinkedList<Vector> PF::calculatePath(Vector currentPosition, float currentOrientation,
                                      QLinkedList<Vector> oldPath, QList<Vector> obstacles,
                                      Vector destination, float orientation) {

  QLinkedList<Vector> path;
  path.append(currentPosition);
  while(path.contains(destination) == false) {
    // Sum all repulsive components
    Vector resultantForce(0.0, 0.0, 0.0, false);
//    std::cout << "Current possition: " << currentPosition.x() << ", " << currentPosition.y() << "\n";
    for (int j = 0; j < obstacles.size(); j++) {
      if(Utils::distance(currentPosition, obstacles.at(j)) < MAX_DISTANCE) {
        resultantForce = resultantForce + calculateForce(currentPosition, obstacles.at(j), true);
      }
    }

    // Calculate the attractive component
    resultantForce = resultantForce + calculateForce(currentPosition, destination, false);

    // Normalize the resultant force and calculate a new position using the current position and the resolution selected
    resultantForce = resultantForce/resultantForce.norm();
    Vector nextPosition = currentPosition+(resultantForce*RESOLUTION);
    if(Utils::distance(nextPosition, destination) < RESOLUTION) {
      path.append(destination);
    } else {
      path.append(nextPosition);
      currentPosition = nextPosition;
    }
  }

  return path;
}

Vector PF::calculateForce(Vector robot, Vector element, bool isRepulsive) {
  float distance = Utils::distance(robot, element);
  Vector ret = element - robot;
  float norm = ret.norm();
  ret = ret/norm;
  if(isRepulsive) {
    ret = ret * (_repulsionConstants.first/pow(distance, _repulsionConstants.second));
  } else {
    ret = ret * (_attractiveConstants.first/pow(distance, _attractiveConstants.second));
  }

  return ret;
}

bool PF::checkCurrentPath(Vector currentPosition, float currentOrientation, QLinkedList<Vector> path,
                          QList<Vector> obstacles) {
  return true;
}

QLinkedList<Vector> PF::updatePathTracking(Vector currentPosition, QLinkedList<Vector> currentPath) {
  return currentPath;
}
