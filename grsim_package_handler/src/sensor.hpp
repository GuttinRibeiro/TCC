#ifndef SENSOR_HH
#define SENSOR_HH

#include <QString>

class Sensor {
    private:
        float _minRange;
        float _maxRange;
        float _minAngle;
        float _maxAngle;
        QString _name;

    public:
        Sensor() {
            _name = "NULL";
            _minRange = 0.0;
            _maxRange = 0.0;
            _maxAngle = 0.0;
            _minAngle = 0.0;            
        }

        Sensor(QString name) {
            _name = name;
            _minRange = 0.0;
            _maxRange = 0.0;
            _maxAngle = 0.0;
            _minAngle = 0.0;
        }

        Sensor(QString name, float maxRange) {
            _name = name;
            _maxRange = maxRange;
            _minRange = 0.0;
            _minAngle = 0.0;
            _maxAngle = 0.0;
        }

        Sensor(QString name, float maxRange, float minAngle, float maxAngle) {
            _name = name;
            _maxRange = maxRange;
            _minRange = 0.0;
            _minAngle = minAngle;
            _maxAngle = maxAngle;
        }

        Sensor(QString name, float minRange, float maxRange, float minAngle, float maxAngle) {
            _name = name;
            _maxRange = maxRange;
            _minRange = minRange;
            _minAngle = minAngle;
            _maxAngle = maxAngle;
        }

        QString name() {return _name;}
        float minRange() {return _minRange;}
        float minAngle() {return _minAngle;}
        float maxRange() {return _maxRange;}
        float maxAngle() {return _maxAngle;}

        void setMaxRange(float value) {_maxRange = value;}
        void setMinRange(float value) {_minRange = value;} 
        void setMaxAngle(float value) {_maxAngle = value;}
        void setMinAngle(float value) {_minAngle = value;}
};

#endif