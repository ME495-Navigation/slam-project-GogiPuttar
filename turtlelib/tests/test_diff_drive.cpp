#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using turtlelib::normalize_angle;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::wheelAngles;
using turtlelib::pose2D;
using turtlelib::DiffDrive;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Initialization works for new DiffDrive", "[DiffDrive()]") 
{
    DiffDrive turtle;

    REQUIRE_THAT( turtle.wheels().left, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().right, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().theta, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().y, WithinAbs(0.0,1.0e-6));
}

TEST_CASE( "Initialization works for general DiffDrive", "[DiffDrive()]") 
{
    DiffDrive turtle{wheelAngles{6.9, 4.20}, pose2D{69, 6.9, 0.69}};

    REQUIRE_THAT( turtle.wheels().left, WithinAbs(normalize_angle(6.9),1.0e-6));
    REQUIRE_THAT( turtle.wheels().right, WithinAbs(normalize_angle(4.20),1.0e-6));
    REQUIRE_THAT( turtle.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle.pose().x, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( turtle.pose().y, WithinAbs(0.69,1.0e-6));
}

