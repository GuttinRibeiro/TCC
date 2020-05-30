#include "utils.hpp"
#include <cmath>

float Utils::distance(Position p1, Position p2) {
    return sqrt((pow(p1.y()-p2.y(), 2))+(pow(p1.x()-p2.x(), 2)));
}

bool Utils::isWithinInterval(float min, float max, float value) {
    if(value > min && value < max) {
        return true;
    }

    return false;
}

float Utils::getAngle(Position pos) {
    return atan2(pos.y(), pos.x());
}

float Utils::getAngle(Position pos1, Position pos2) {
    return atan2(pos1.y()-pos2.y(), pos1.x()-pos2.x());
}

float Utils::scalarProduct(Position A, Position B) {
    return A.x()*B.x() + A.y()*B.y();
}

Position Utils::projectPointAtLine(Position p1, Position p2, Position point) {
    Position a(point.x()-p1.x(), point.y()-p1.y(), 0.0, false);
    Position b(p2.x()-p1.x(), p2.y()-p1.y(), 0.0, false);
    float bModule = sqrt(pow(b.x(),2)+pow(b.y(),2));
    Position bUnitary(b.x()/bModule, b.y()/bModule, 0.0, false);
    float scalar = Utils::scalarProduct(a, bUnitary);
    return Position(p1.x()+scalar*bUnitary.x(), p1.y()+scalar*bUnitary.y(), 0.0, false);
}

float Utils::distanceToLine(Position p1, Position p2, Position point) {
    Position projectedPoint = Utils::projectPointAtLine(p1, p2, point);
    float distance = Utils::distance(point, projectedPoint);
    return (distance<=0.001f)? 0 : distance;
}

float Utils::wrapToTwoPi(float angle) {
    angle += 10*M_PI; //Sum a bunch of PIs to make sure this angle is positive
    return fmod(angle, 2*M_PI);
}

float Utils::angleDiff(float a, float b) {
    float diff = a-b;
    if(diff > M_PI) {
        diff = 2*M_PI-diff;
    } else if(diff < -M_PI) {
        diff = 2*M_PI+diff;
    }

    return diff;
}