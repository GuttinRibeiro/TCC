#ifndef UTILS_HH
#define UTILS_HH

#include "vector.hpp"

class Utils {
    public:
        static double distance(Vector p1, Vector p2);
        static bool isWithinInterval(float min, float max, float value);
        static float getAngle(Vector pos);
        static float getAngle(Vector p1, Vector p2);
        static float scalarProduct(Vector A, Vector B);
        static Vector projectPointAtLine(Vector p1, Vector p2, Vector point);
        static float distanceToLine(Vector p1, Vector p2, Vector point);
        static float wrapToTwoPi(float angle);
        static float angleDiff(float a, float b);
        static Vector rotateVectorAroundZ(Vector v, float angle);
};

#endif
