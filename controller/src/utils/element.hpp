#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include "vector.hpp"
#include "groups.hpp"
#include <QtCore>

class Element {
  private:
    Vector _pos;
    Vector _vel;
    float _ori;
    qint8 _id;
    int _group;
    float _radius;
    QLinkedList<Vector> _path;

  public:
    Element() {
      _pos = Vector();
      _vel = Vector();
      _ori = 0.0;
      _id = 0;
      _group = Groups::UNKNOWN;
      _radius = 0.0;
    }

    Element(Vector pos, Vector vel, float ori, qint8 id, int group, float radius) {
      _pos = pos;
      _vel = vel;
      _ori = ori;
      _id = id;
      _group = group;
      _radius = radius;
    }

    void setPosition(Vector pos) {_pos = pos;}
    void setVelocity(Vector vel) {_vel = vel;}
    void setOrientation(float ori) {_ori = ori;}
    void setId(qint8 id) {_id = id;}
    void setGroup(int group) {_group = group;}
    void setRadius(float radius) {_radius = radius;}
    void addPath(QLinkedList<Vector> path) {_path = path;}

    Vector position() const {return _pos;}
    Vector velocity() const {return _vel;}
    float orientation() const {return  _ori;}
    qint8 id() const {return  _id;}
    int group() const {return  _group;}
    float radius() const {return _radius;}
    QLinkedList<Vector> path() const {return _path;}
};

#endif // ELEMENT_HPP
