#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using turtlelib::normalize_angle;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Stream insertion operator << works for 2d twist", "[operator<<]") 
{
    Twist2D tw{0.69, 6.9, 69.6};
    std::string str = "[0.69 6.9 69.6]"; //typecast to int for integers ??

    std::stringstream sstr;
    sstr << tw;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d twist", "[operator>>]") // Adapted from Abhishek Sankar
{
    Twist2D tw_1{}, tw_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.69 4.20 69.0";
    sstr_1 >> tw_1;

    sstr_2 << "[6969.0 420.0 69.0]";
    sstr_2 >> tw_2;

    REQUIRE_THAT( tw_1.omega, WithinAbs(0.69,1.0e-6));
    REQUIRE_THAT( tw_1.x, WithinAbs(4.20,1.0e-6));
    REQUIRE_THAT( tw_1.y, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( tw_2.omega, WithinAbs(6969.0,1.0e-6));
    REQUIRE_THAT( tw_2.x, WithinAbs(420.0,1.0e-6));
    REQUIRE_THAT( tw_2.y, WithinAbs(69.0,1.0e-6));
}

TEST_CASE( "Initialization works for SE(2) identity transform", "[Transform2d()]") 
{
    Transform2D tf;

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "Initialization works for SE(2) pure translation transform", "[Transform2d(Vector2D)]") 
{
    Vector2D displacement{69.0, 69.0};
    Transform2D tf{displacement};

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(69.0,1.0e-6));    
}

TEST_CASE( "Initialization works for SE(2) pure rotation transform", "[Transform2d(double)]") 
{
    double angle{-6.9*PI};
    Transform2D tf{angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(-0.9*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "Initialization works for general SE(2) transform", "[Transform2d(Vector2D, double)]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{-6.9*PI};
    Transform2D tf{displacement, angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(-0.9*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(4.20,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(6.9,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D point works", "[Transform2d() Point2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Point2D p{6.9, 4.20};

    Point2D newp = tf(p);

    REQUIRE_THAT( newp.x, WithinAbs(7.399056180434521,1.0e-6));
    REQUIRE_THAT( newp.y, WithinAbs(14.317279794805081,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D vector works", "[Transform2d() Vector2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Vector2D v{6.9, 4.20};

    Vector2D newv = tf(v);

    REQUIRE_THAT( newv.x, WithinAbs(3.199056180434521,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(7.417279794805081,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D twist works", "[Transform2d() Twist2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Twist2D v{6.9, 6.9, 4.20};

    Twist2D newv = tf(v);

    REQUIRE_THAT( newv.omega, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( newv.x, WithinAbs(50.809056180434524,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(-21.562720205194921,1.0e-6));    
}

TEST_CASE( "Inverse of SE(2) transformation works", "[inv()]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{-6.9*PI};
    Transform2D tf{displacement, angle};
    
    Transform2D tf_inv = tf.inv();

    REQUIRE_THAT( tf_inv.rotation(), WithinAbs(0.9*PI,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().x, WithinAbs(6.126654629626783,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().y, WithinAbs(5.264418586061781,1.0e-6));  
}

TEST_CASE( "SE(2) composition operator works", "[oprator *=]") 
{
    Vector2D displacement_1{4.20, 6.9};
    double angle_1{-6.9*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{6.9, 4.20};
    double angle_2{4.2*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    tf_1 *= tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(-0.7*PI,1.0e-6));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
}

TEST_CASE( "Stream insertion operator << works for SE(2) transform", "[operator<<]") 
{
    Vector2D displacement{6.9, 69.6};
    double angle{0.69};
    Transform2D tf{displacement, angle};

    std::string str = "deg: 0.69 x: 6.9 y: 69.6";

    std::stringstream sstr;
    sstr << tf;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works SE(2) transform", "[operator>>]") // Adapted from Abhishek Sankar
{
    Transform2D tf_1{}, tf_2{};
    std::stringstream sstr_a, sstr_b;

    sstr_a << "0.69 4.20 69.0";
    sstr_a >> tf_1;

    sstr_b << "deg: 6969.0 x: 420.0 y: 69.0";
    sstr_b >> tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(turtlelib::normalize_angle(turtlelib::deg2rad(0.69)), 1.0e-6 ));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(4.20, 1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(69.0, 1.0e-6));
    REQUIRE_THAT( tf_2.rotation(), WithinAbs(turtlelib::normalize_angle(turtlelib::deg2rad(6969.0)), 1.0e-6));
    REQUIRE_THAT( tf_2.translation().x, WithinAbs(420.0, 1.0e-6) );
    REQUIRE_THAT( tf_2.translation().y, WithinAbs(69.0, 1.0e-6) );
}

TEST_CASE( "SE(2) multiplication operator works", "[oprator *]") 
{
    Vector2D displacement_1{4.20, 6.9};
    double angle_1{-6.9*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{6.9, 4.20};
    double angle_2{4.2*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    Transform2D tf_3{(tf_1 * tf_2).translation(), (tf_1 * tf_2).rotation()};

    REQUIRE_THAT( tf_3.rotation(), WithinAbs(-0.7*PI,1.0e-6));
    REQUIRE_THAT( tf_3.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
    REQUIRE_THAT( tf_3.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
}
