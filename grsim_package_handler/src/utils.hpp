#ifndef UTILS_HH
#define UTILS_HH

#include "position.hpp"

class Utils {
    public:
        static float distance(Position p1, Position p2);
        static bool isWithinInterval(float min, float max, float value);
        static float getAngle(Position pos);
        static float getAngle(Position p1, Position p2);
        static float scalarProduct(Position A, Position B);
        static Position projectPointAtLine(Position p1, Position p2, Position point);
        static float distanceToLine(Position p1, Position p2, Position point);
};

#endif