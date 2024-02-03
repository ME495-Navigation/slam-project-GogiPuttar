#include <sstream>
#include <cmath>
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

    REQUIRE_THAT( turtle.radius(), WithinAbs(1.0,1.0e-6));
    REQUIRE_THAT( turtle.separation(), WithinAbs(1.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().left, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().right, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().theta, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().y, WithinAbs(0.0,1.0e-6));
}

TEST_CASE( "Initialization works for general DiffDrive", "[DiffDrive(double radius, double sep, wheelAngles wheels, pose2D pose)]") 
{
    DiffDrive turtle{69, 420, wheelAngles{6.9, 4.20}, pose2D{69, 6.9, 0.69}};

    REQUIRE_THAT( turtle.radius(), WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( turtle.separation(), WithinAbs(420.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().left, WithinAbs(normalize_angle(6.9),1.0e-6));
    REQUIRE_THAT( turtle.wheels().right, WithinAbs(normalize_angle(4.20),1.0e-6));
    REQUIRE_THAT( turtle.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle.pose().x, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( turtle.pose().y, WithinAbs(0.69,1.0e-6));
}

TEST_CASE( "Initialization works for a DiffDrive at the origin", "[DiffDrive(double radius, double sep)]") 
{
    DiffDrive turtle{69, 420};

    REQUIRE_THAT( turtle.radius(), WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( turtle.separation(), WithinAbs(420.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().left, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.wheels().right, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().theta, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( turtle.pose().y, WithinAbs(0.0,1.0e-6));
}

TEST_CASE( "Driving through wheels works", "[driveWheels()]") 
{
    // No motion
    DiffDrive turtle0{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    wheelAngles delta_phi0 = {0.0, 0.0};

    turtle0.driveWheels(delta_phi0);

    REQUIRE_THAT( turtle0.wheels().left, WithinAbs(normalize_angle(6.9),1.0e-6));
    REQUIRE_THAT( turtle0.wheels().right, WithinAbs(normalize_angle(4.20),1.0e-6));
    REQUIRE_THAT( turtle0.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle0.pose().x, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( turtle0.pose().y, WithinAbs(420.0,1.0e-6));
    
    // Linear motion
    DiffDrive turtle1{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    wheelAngles delta_phi1 = {6.9, 6.9};

    turtle1.driveWheels(delta_phi1);

    REQUIRE_THAT( turtle1.wheels().left, WithinAbs(normalize_angle(6.9 + 6.9),1.0e-6));
    REQUIRE_THAT( turtle1.wheels().right, WithinAbs(normalize_angle(4.20 + 6.9),1.0e-6));
    REQUIRE_THAT( turtle1.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle1.pose().x, WithinAbs(69.0 + 0.69 * 6.9 * cos(69.0),1.0e-6));
    REQUIRE_THAT( turtle1.pose().y, WithinAbs(420.0 + 0.69 * 6.9 * sin(69.0),1.0e-6));
    
    // Spinning in place
    DiffDrive turtle2{69, 420, wheelAngles{6.9, 4.20}, pose2D{69, 6.9, 0.69}};
    wheelAngles delta_phi2 = {0.69, -0.69};

    turtle2.driveWheels(delta_phi2);

    REQUIRE_THAT( turtle2.wheels().left, WithinAbs(normalize_angle(6.9 + 0.69),1.0e-6));
    REQUIRE_THAT( turtle2.wheels().right, WithinAbs(normalize_angle(4.20 - 0.69),1.0e-6));
    REQUIRE_THAT( turtle2.pose().theta, WithinAbs(normalize_angle(69 - 2 * 0.69 * 69.0 / 420.0 ),1.0e-6));
    REQUIRE_THAT( turtle2.pose().x, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( turtle2.pose().y, WithinAbs(0.69,1.0e-6));
    
    // Quarter circular arc
    DiffDrive turtle3{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    wheelAngles delta_phi3 = {(0.420 / 0.69) * (PI / 2), 2 * (0.420 / 0.69) * (PI / 2)};

    turtle3.driveWheels(delta_phi3);

    REQUIRE_THAT( turtle3.wheels().left, WithinAbs(normalize_angle(6.9 + (0.420 / 0.69) * (PI / 2)),1.0e-6));
    REQUIRE_THAT( turtle3.wheels().right, WithinAbs(normalize_angle(4.20 + 2 * (0.420 / 0.69) * (PI / 2)),1.0e-6));
    REQUIRE_THAT( turtle3.pose().theta, WithinAbs(normalize_angle(69 + PI/2),1.0e-6));
    REQUIRE_THAT( turtle3.pose().x, WithinAbs(69 + 0.63 * sqrt(2) * cos(69 + PI/4),1.0e-6));
    REQUIRE_THAT( turtle3.pose().y, WithinAbs(420 + 0.63 * sqrt(2) * sin(69 + PI/4),1.0e-6));

    // Random arc
    DiffDrive turtle4{69, 420, wheelAngles{6.9, 4.20}, pose2D{69, 6.9, 0.69}};
    wheelAngles delta_phi4 = {0.69, -0.420};

    turtle4.driveWheels(delta_phi4);

    REQUIRE_THAT( turtle4.wheels().left, WithinAbs(normalize_angle(6.9 + 0.69),1.0e-6));
    REQUIRE_THAT( turtle4.wheels().right, WithinAbs(normalize_angle(4.20 - 0.420),1.0e-6));
    REQUIRE_THAT( turtle4.pose().theta, WithinAbs(normalize_angle(69 - (0.69 + 0.420) * 69.0/420.0 ),1.0e-6));
    REQUIRE_THAT( turtle4.pose().x, WithinAbs(16.0050106563,1.0e-6));
    REQUIRE_THAT( turtle4.pose().y, WithinAbs(-1.2146835484,1.0e-6));
}

TEST_CASE( "Driving directly through twist works", "[driveTwist()]") 
{
    // No motion
    DiffDrive turtle0{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    Twist2D V_b0;

    turtle0.driveTwist(V_b0);

    REQUIRE_THAT( turtle0.wheels().left, WithinAbs(normalize_angle(6.9),1.0e-6));
    REQUIRE_THAT( turtle0.wheels().right, WithinAbs(normalize_angle(4.20),1.0e-6));
    REQUIRE_THAT( turtle0.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle0.pose().x, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( turtle0.pose().y, WithinAbs(420.0,1.0e-6));
    
    // Linear motion
    DiffDrive turtle1{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    Twist2D V_b1{0.0, 0.69 * 6.9, 0.0};

    turtle1.driveTwist(V_b1);

    REQUIRE_THAT( turtle1.wheels().left, WithinAbs(normalize_angle(6.9 + 6.9),1.0e-6));
    REQUIRE_THAT( turtle1.wheels().right, WithinAbs(normalize_angle(4.20 + 6.9),1.0e-6));
    REQUIRE_THAT( turtle1.pose().theta, WithinAbs(normalize_angle(69),1.0e-6));
    REQUIRE_THAT( turtle1.pose().x, WithinAbs(69.0 + 0.69 * 6.9 * cos(69.0),1.0e-6));
    REQUIRE_THAT( turtle1.pose().y, WithinAbs(420.0 + 0.69 * 6.9 * sin(69.0),1.0e-6));
    
    // Spinning in place
    DiffDrive turtle2{69, 420, wheelAngles{6.9, 4.20}, pose2D{69, 6.9, 0.69}};
    Twist2D V_b2{-0.69 * 69 / 210, 0.0, 0.0};

    turtle2.driveTwist(V_b2);

    REQUIRE_THAT( turtle2.wheels().left, WithinAbs(normalize_angle(6.9 + 0.69),1.0e-6));
    REQUIRE_THAT( turtle2.wheels().right, WithinAbs(normalize_angle(4.20 - 0.69),1.0e-6));
    REQUIRE_THAT( turtle2.pose().theta, WithinAbs(normalize_angle(69 - 2 * 0.69 * 69.0 / 420.0 ),1.0e-6));
    REQUIRE_THAT( turtle2.pose().x, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( turtle2.pose().y, WithinAbs(0.69,1.0e-6));
    
    // Quarter circular arc
    DiffDrive turtle3{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    Twist2D V_b3{PI/2, 0.63 * PI/2, 0.0};

    turtle3.driveTwist(V_b3);

    REQUIRE_THAT( turtle3.wheels().left, WithinAbs(normalize_angle(6.9 + (0.420 / 0.69) * (PI / 2)),1.0e-6));
    REQUIRE_THAT( turtle3.wheels().right, WithinAbs(normalize_angle(4.20 + 2 * (0.420 / 0.69) * (PI / 2)),1.0e-6));
    REQUIRE_THAT( turtle3.pose().theta, WithinAbs(normalize_angle(69 + PI/2),1.0e-6));
    REQUIRE_THAT( turtle3.pose().x, WithinAbs(69 + 0.63 * sqrt(2) * cos(69 + PI/4),1.0e-6));
    REQUIRE_THAT( turtle3.pose().y, WithinAbs(420 + 0.63 * sqrt(2) * sin(69 + PI/4),1.0e-6));

    //  Invalid Twist
    DiffDrive turtle5{0.69, 0.420, wheelAngles{6.9, 4.20}, pose2D{69, 69, 420}};
    Twist2D V_b5{PI/2, 0.63 * PI/2, 0.0004};

    REQUIRE_THROWS(turtle5.driveTwist(V_b5));
}

