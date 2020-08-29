#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <cmath>

class Vector {
  private:
    float _x;
    float _y;
    float _z;
    double _timestamp;
    bool  _isUnknown;
    bool _hasTimestamp;

  public:
    Vector();
    Vector(float x, float y, float z, bool isUnknown);
    Vector(float x, float y, bool isUnknown);
    Vector(float x, float y, float z, double timestamp, bool isUnknown);
    Vector(float x, float y, double timestamp, bool isUnknown);
    float x() const {return _x;}
    float y() const {return _y;}
    float z() const {return _z;}
    float norm() const {return sqrt(pow(_x, 2)+pow(_y, 2)+pow(_z, 2));}
    double timestamp() const {return _timestamp;}
    bool hasTimestamp() const {return _hasTimestamp;}
    bool isUnknown() const {return _isUnknown;}

    void setIsUnknown(bool isUnknown) {_isUnknown = isUnknown;}
    void setX(float x) {_x = x;}
    void setY(float y) {_y = y;}
    void setZ(float z) {_z = z;}
    void setTimestamp(double timestamp);

    // Operators:
    bool operator==(Vector v) {
        if(_x == v.x() && _y == v.y() && _z == v.z()) {
            return true;
        }

        return false;
    }

    Vector operator+(Vector v) {
        return Vector(this->x()+v.x(), this->y()+v.y(), this->z()+v.z(), false);
    }

    Vector operator-(Vector v) {
        return Vector(this->x()-v.x(), this->y()-v.y(), this->z()-v.z(), false);
    }

    Vector operator/(float k) {
      return Vector(this->x()/k, this->y()/k, this->z()/k, false);
    }

    Vector operator*(float k) {
      return Vector(this->x()*k, this->y()*k, this->z()*k, false);
    }
};

#endif // POSITION_HPP
