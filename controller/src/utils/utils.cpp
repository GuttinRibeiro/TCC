#include "utils.hpp"
#include <cmath>

double Utils::distance(Vector p1, Vector p2) {
    return sqrt((pow(p1.y()-p2.y(), 2))+(pow(p1.x()-p2.x(), 2)));
}

bool Utils::isWithinInterval(float min, float max, float value) {
    if(value > min && value < max) {
        return true;
    }

    return false;
}

float Utils::getAngle(Vector pos) {
    return atan2(pos.y(), pos.x());
}

float Utils::getAngle(Vector pos1, Vector pos2) {
    return atan2(pos1.y()-pos2.y(), pos1.x()-pos2.x());
}

float Utils::scalarProduct(Vector A, Vector B) {
    return A.x()*B.x() + A.y()*B.y();
}

Vector Utils::projectPointAtLine(Vector p1, Vector p2, Vector point) {
    Vector a(point.x()-p1.x(), point.y()-p1.y(), 0.0, false);
    Vector b(p2.x()-p1.x(), p2.y()-p1.y(), 0.0, false);
    float bModule = sqrt(pow(b.x(),2)+pow(b.y(),2));
    Vector bUnitary(b.x()/bModule, b.y()/bModule, 0.0, false);
    float scalar = Utils::scalarProduct(a, bUnitary);
    return Vector(p1.x()+scalar*bUnitary.x(), p1.y()+scalar*bUnitary.y(), 0.0, false);
}

float Utils::distanceToLine(Vector p1, Vector p2, Vector point) {
    Vector projectedPoint = Utils::projectPointAtLine(p1, p2, point);
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
