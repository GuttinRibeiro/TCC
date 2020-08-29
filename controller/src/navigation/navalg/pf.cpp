#include "pf.hpp"
#include "../../utils/utils.hpp"
#include <cmath>
#define RESOLUTION 0.1f
#define MAX_DISTANCE 5

PF::PF(InfoBus *ib, qint8 id, int frequency) : Navigation_Algorithm (ib, id, frequency) {
  _x_shift = -0.5;
  _y_shift = 0.0;
  _factor = 2.0;
  _k = 0.15;
}

PF::~PF() {

}

QLinkedList<Vector> PF::calculatePath(Vector currentPosition, float currentOrientation,
                                      QLinkedList<Vector> oldPath, QList<Vector> obstacles,
                                      Vector destination, float orientation) {

  QLinkedList<Vector> path;
  path.append(currentPosition);
  // Trivial case
  if(destination == currentPosition) {
    path.append(destination);
//    std::cout << "[PF] Robot is already clone enough to its destination\n";
  }

  while(path.contains(destination) == false) {
    // Calculate the attractive component
    Vector resultantForce = (destination-currentPosition);

    // Sum all repulsive components
    for (int j = 0; j < obstacles.size(); j++) {
      if(Utils::distance(currentPosition, obstacles.at(j)) < MAX_DISTANCE) {
        resultantForce = resultantForce + calculateForce(currentPosition, obstacles.at(j));
      }
    }

    // Normalize the resultant force and calculate a new position using the current position and the resolution selected
    resultantForce = resultantForce/resultantForce.norm();

    // Check if the resultant force may lead the robot to a collision with an obstacle
    for (int j = 0; j < obstacles.size(); j++) {
      if(isOnCollisionRoute(currentPosition, destination, resultantForce, obstacles.at(j), 0.2f)) {
        Vector obstacle = obstacles.at(j);
        Vector reference = obstacle - currentPosition;
        float alpha = acos(Utils::scalarProduct(resultantForce, reference)/(resultantForce.norm()*reference.norm()));
        Vector tangentPosition;
        if(alpha > 0) {
          tangentPosition = Utils::threePoints(obstacle, currentPosition, 0.4f, M_PI_2f32);
        } else {
          tangentPosition = Utils::threePoints(obstacle, currentPosition, 0.4f, -M_PI_2f32);
        }

        // Iterate through all obstacles again considering the new resultant force
        resultantForce = tangentPosition/tangentPosition.norm();
        j = 0;
      }
    }

    Vector nextPosition = currentPosition+(resultantForce*RESOLUTION);

    if(Utils::distance(nextPosition, destination) < RESOLUTION) {
//      std::cout << "[PF] Operation completed!\n";
      path.append(destination);
    } else {
//      std::cout << "[PF] Appending position " << nextPosition.x() << ", " << nextPosition.y() << "\n";
      path.append(nextPosition);
      currentPosition = nextPosition;
    }
  }

  return path;
}

Vector PF::calculateForce(Vector robot, Vector element) {
  float distance = Utils::distance(robot, element);
  Vector ret = element - robot;
  float norm = ret.norm();
  ret = ret/norm;
  ret = ret * (_k/pow(distance-_y_shift, _factor) + _y_shift);
  return ret;
}

bool PF::checkCurrentPath(Vector currentPosition, float currentOrientation, QLinkedList<Vector> path,
                          QList<Vector> obstacles) {
  return true;
}

QLinkedList<Vector> PF::updatePathTracking(Vector currentPosition, QLinkedList<Vector> currentPath) {
  return currentPath;
}

bool PF::isOnCollisionRoute(Vector currentPosition, Vector destination, Vector resultantForce, Vector obstacle, float radius) {
  if(Utils::distance(obstacle, currentPosition) < Utils::distance(destination, currentPosition)) {
    Vector reference = obstacle - currentPosition;
    float alpha = acos(Utils::scalarProduct(resultantForce, reference)/(resultantForce.norm()*reference.norm()));
    Vector tangentPosition;
    if(alpha > 0) {
      tangentPosition = Utils::threePoints(obstacle, currentPosition, radius, M_PI_2f32);
    } else {
      tangentPosition = Utils::threePoints(obstacle, currentPosition, radius, -M_PI_2f32);
    }
    Vector limitVector = tangentPosition-currentPosition;
    float beta = acos(Utils::scalarProduct(limitVector, reference)/(limitVector.norm()*reference.norm()));
    if(Utils::isWithinInterval(0.0f, beta, alpha)) {
      return  true;
    }
  }

  return false;
}
