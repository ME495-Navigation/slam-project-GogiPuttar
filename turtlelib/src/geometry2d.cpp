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

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {   
        return Vector2D{head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {   
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p)
    {
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        const auto c = is.peek();        // examine the next character without extracting it

        if (c == '[') {
            is.get();         // remove the '[' character from the stream
            is >> p.x;
            is >> p.y;
            is.get();         // remove the ']' character from the stream
        } else {
            is >> p.x >> p.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        const auto c = is.peek();        // examine the next character without extracting it

        if (c == '[') {
            is.get();         // remove the '[' character from the stream
            is >> v.x;
            is >> v.y;
            is.get();         // remove the ']' character from the stream
        } else {
            is >> v.x >> v.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    Vector2D normalizeVector(const Vector2D & v)
    {
        Vector2D v_hat{};
        double v_norm = sqrt(v.x * v.x + v.y * v.y);

        if(v_norm == 0.0)
        {
            std::cout << "INVALID VECTOR." << std::endl;
        }
        else
        {
            v_hat.x = v.x / v_norm;
            v_hat.y = v.y / v_norm;
        }

        return v_hat;
    }

    Vector2D operator-(const vector2D & va, const vector2D & vb)
    {
        return Vector2D{va.x - vb.x, va.y - vb.y};
    }

    Vector2D operator+(const vector2D & va, const vector2D & vb)
    {
        return Vector2D{va.x + vb.x, va.y + vb.y};
    }

    Vector2D operator*(const double & scale, const vector2D & v)
    {
        return Vector2D{scale * v.x, scale * v.y};
    }

    Vector2D operator*(const vector2D & v, const double & scale);
    {
        return Vector2D{scale * v.x, scale * v.y};
    }

}
