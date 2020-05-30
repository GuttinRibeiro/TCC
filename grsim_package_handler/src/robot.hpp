#ifndef ROBOT_HH
#define ROBOT_HH
#include "position.hpp"
#include "sensor.hpp"
#include <QList>

class Robot {
    private:
        Position _pos;
        float _ori;
        qint8 _id;
        int _team;
        bool _hasOri;
        Sensor _sensorSet;
        float _radius;

    public:
        Robot() {
            _pos = Position();
            _ori = 0.0;
            _hasOri = false;
            _team = -1;
            _id = -1;
            _radius = 0.09;
        }
        Robot(Position pos, int team, qint8 id) {
            _pos = pos;
            _team = team;
            _id = id;
            _ori = 0.0;
            _hasOri = false;
            _radius = 0.09;
        }
        Robot(Position pos, float orientation, int team, quint8 id) {
            _pos = pos;
            _ori = orientation;
            _hasOri = true;
            _team = team;
            _id = id;
            _radius = 0.09;
        }

        bool hasOrientarion() {return _hasOri;}
        Position position() {return _pos;}
        float orientation() {return _ori;}
        quint8 id() {return _id;}
        int team() {return _team;}
        Sensor getSensor() {return _sensorSet;}
        float radius() {return _radius;}

        void setPosition(Position pos) {_pos = pos;}
        void setOrientation(float ori) {_ori = ori; _hasOri = true;}
        void clearOrientation() {_hasOri = false;}
        void setId(quint8 id) {_id = id;}
        void setTeam(int team) {_team = team;}
        void addSensor(Sensor sensor) {_sensorSet = sensor;}
};

#endif