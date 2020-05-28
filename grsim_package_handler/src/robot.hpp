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
        QList<Sensor> _sensorSet;

    public:
        Robot() {
            _pos = Position();
            _ori = -1;
            _hasOri = false;
            _team = -1;
            _id = -1;
        }
        Robot(Position pos, int team, qint8 id) {
            _pos = pos;
            _team = team;
            _id = id;
            _ori = -1;
            _hasOri = false;
        }
        Robot(Position pos, float orientation, int team, qint8 id) {
            _pos = pos;
            _ori = orientation;
            _hasOri = true;
            _team = team;
            _id = id;
        }

        bool hasOrientarion() {return _hasOri;}
        Position position() {return _pos;}
        float orientation() {return _ori;}
        qint8 id() {return _id;}
        int team() {return _team;}
        QList<Sensor> sensorSet() {return _sensorSet;}

        void setPosition(Position pos) {_pos = pos;}
        void setOrientation(float ori) {_ori = ori; _hasOri = true;}
        void clearOrientation() {_hasOri = false;}
        void setId(qint8 id) {_id = id;}
        void setTeam(int team) {_team = team;}
        void addSensor(Sensor sensor) {_sensorSet.append(sensor);}
};

#endif