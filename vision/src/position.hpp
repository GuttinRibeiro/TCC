#ifndef POSITION_HH
#define POSITION_HH

class Position {
    private:
        float _x;
        float _y;
        float _z;
        bool  _isUnknown;
        float _confidence;

    public:
        Position(float x = 0.0, float y = 0.0, float z = 0.0, bool isUnknown = true, float confidence = 1.0) {
            _x = x;
            _y = y;
            _z = z;
            if(isUnknown) {
              _confidence = 0.0;
            } else {
              _confidence = confidence;
            }
            _isUnknown = isUnknown;
        }

        const float x() {return _x;}
        const float y() {return _y;}
        const float z() {return _z;}
        const bool isUnknown() {return _isUnknown;}
        const float confidence() {return _confidence;}

        void setBool(bool isUnknown) {_isUnknown = isUnknown;}
        void setX(float x) {_x = x;}
        void setY(float y) {_y = y;}
        void setZ(float z) {_z = z;}
        void updateConfidence(float confidence) {_confidence = confidence;}

        // Operators:
        void operator=(Position pos) {
            this->_x = pos.x();
            this->_y = pos.y();
            this->_z = pos.z();
            this->_isUnknown = pos.isUnknown();
            this->_confidence = pos.confidence();
        }

        bool operator==(Position pos) {
            if(this->x() == pos.x() && this->y() == pos.y() && this->z() == pos.z()) {
                return true;
            }

            return false;
        }

        Position operator+(Position pos) {
            return Position(this->x()+pos.x(), this->y()+pos.y(), this->z()+pos.z());
        }

        Position operator-(Position pos) {
            return Position(this->x()-pos.x(), this->y()-pos.y(), this->z()-pos.z());
        }
};

#endif
