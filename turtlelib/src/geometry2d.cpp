#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    double normalize_angle(double rad)
    {
        // Normalize to range (-PI, PI].
        if (almost_equal(rad, -PI))
        {
            return PI;
        }
        return atan2(sin(rad), cos(rad));
    }
}
