#include "vector.hpp"

Vector::Vector() {
  _x = 0.0;
  _y = 0.0;
  _z = 0.0;
  _isUnknown = true;
  _confidence = 0.0;
  _timestamp = 0.0;
  _hasTimestamp = false;
}


Vector::Vector(float x, float y, float z, bool isUnknown, float confidence) {
  _x = x;
  _y = y;
  _z = z;
  if(isUnknown) {
    _confidence = 0.0;
  } else {
    _confidence = confidence;
  }
  _isUnknown = isUnknown;
  _timestamp = 0.0;
  _hasTimestamp = false;
}

Vector::Vector(float x, float y, bool isUnknown, float confidence) {
  _x = x;
  _y = y;
  _z = 0.0;
  if(isUnknown) {
    _confidence = 0.0;
  } else {
    _confidence = confidence;
  }
  _isUnknown = isUnknown;
  _timestamp = 0.0;
  _hasTimestamp = false;
}

Vector::Vector(float x, float y, float z, int32_t timestamp, bool isUnknown, float confidence) {
  _x = x;
  _y = y;
  _z = z;
  if(isUnknown) {
    _confidence = 0.0;
  } else {
    _confidence = confidence;
  }
  _isUnknown = isUnknown;
  _timestamp = timestamp;
  _hasTimestamp = true;
}

Vector::Vector(float x, float y, int32_t timestamp, bool isUnknown, float confidence) {
  _x = x;
  _y = y;
  _z = 0.0;
  if(isUnknown) {
    _confidence = 0.0;
  } else {
    _confidence = confidence;
  }
  _isUnknown = isUnknown;
  _timestamp = timestamp;
  _hasTimestamp = true;
}

void Vector::setTimestamp(double timestamp) {
  _timestamp = timestamp;
  _hasTimestamp = true;
}
