#include "vector.hpp"

Vector::Vector() {
  _x = 0.0;
  _y = 0.0;
  _z = 0.0;
  _isUnknown = true;
  _timestamp = 0.0;
  _hasTimestamp = false;
}


Vector::Vector(float x, float y, float z, bool isUnknown) {
  _x = x;
  _y = y;
  _z = z;
  _isUnknown = isUnknown;
  _timestamp = 0.0;
  _hasTimestamp = false;
}

Vector::Vector(float x, float y, bool isUnknown) {
  _x = x;
  _y = y;
  _z = 0.0;
  _isUnknown = isUnknown;
  _timestamp = 0.0;
  _hasTimestamp = false;
}

Vector::Vector(float x, float y, float z, double timestamp, bool isUnknown) {
  _x = x;
  _y = y;
  _z = z;
  _isUnknown = isUnknown;
  _timestamp = timestamp;
  _hasTimestamp = true;
}

Vector::Vector(float x, float y, double timestamp, bool isUnknown) {
  _x = x;
  _y = y;
  _z = 0.0;
  _isUnknown = isUnknown;
  _timestamp = timestamp;
  _hasTimestamp = true;
}

void Vector::setTimestamp(double timestamp) {
  _timestamp = timestamp;
  _hasTimestamp = true;
}
