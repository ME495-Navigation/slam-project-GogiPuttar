#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"

using turtlelib::normalize_angle;
using turtlelib::normalizeVector;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Angles are normalized", "[normalize_angle]" ) 
{
    // Typecast case
    REQUIRE_THAT( normalize_angle(1), WithinAbs(1.0,1.0e-6));
    // Overflow case
    REQUIRE_THAT(normalize_angle(PI+1), WithinAbs(1.0-PI,1.0e-6));
    // Large Overflow case
    REQUIRE_THAT(normalize_angle(400*PI+1), WithinAbs(1.0,1.0e-6));
    // Upper limit case (included)
    REQUIRE_THAT(normalize_angle(PI), WithinAbs(PI,1.0e-6));
    // Lower limit case (not included)
    REQUIRE_THAT(normalize_angle(-PI), WithinAbs(PI,1.0e-6));
    // Zero case
    REQUIRE_THAT(normalize_angle(0), WithinAbs(0,1.0e-6));
    // Simple case with pi
    REQUIRE_THAT(normalize_angle(-PI/4.0), WithinAbs(-PI/4.0,1.0e-6));
    // Overflow case with pi
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
    // Underflow case with pi
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
}

TEST_CASE( "Relative vector construction through head and tail points works", "[operator-]") 
{
    // Check x.
    REQUIRE_THAT(  (Point2D{-2.6, 0.0} - Point2D{4.0, 0.0}).x, WithinAbs(-6.6,1.0e-6));
    // Check y.
    REQUIRE_THAT(  (Point2D{0.0, 3.0} - Point2D{0.0, 5.3}).y, WithinAbs(-2.3,1.0e-6));
}

TEST_CASE( "Point displacement through relative vector works", "[operator+]") 
{
    // Check x.
    REQUIRE_THAT(  (Point2D{-2.6, 0.0} + Vector2D{1.0, -1.0}).x, WithinAbs(-1.6,1.0e-6));
    // Check y.
    REQUIRE_THAT(  (Point2D{0.0, 3.0} + Vector2D{1.0, -1.0}).y, WithinAbs(2.0,1.0e-6));
}

TEST_CASE( "Stream insertion works for 2d point", "[operator<<]") 
{
    Point2D point{1.2, 3.4};
    std::string str = "[1.2 3.4]";

    std::stringstream sstr;
    sstr << point;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d point", "[operator>>]") // Adapted from Abhishek Sankar
{
    Point2D point_1{}, point_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.7 4.3";
    sstr_1 >> point_1;

    sstr_2 << "[9.5 1.32]";
    sstr_2 >> point_2;

    REQUIRE( point_1.x == 0.7 );
    REQUIRE( point_1.y == 4.3 );
    REQUIRE( point_2.x == 9.5 );
    REQUIRE( point_2.y == 1.32 );
}

TEST_CASE( "Stream insertion operator << works for 2d vector", "[operator<<]") 
{
    Vector2D vec{1.2, 3.4};
    std::string str = "[1.2 3.4]";

    std::stringstream sstr;
    sstr << vec;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d vector", "[operator>>]") // Adapted from Abhishek Sankar
{
    Vector2D vec_1{}, vec_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.7 4.3";
    sstr_1 >> vec_1;

    sstr_2 << "[9.5 1.32]";
    sstr_2 >> vec_2;

    REQUIRE( vec_1.x == 0.7 );
    REQUIRE( vec_1.y == 4.3 );
    REQUIRE( vec_2.x == 9.5 );
    REQUIRE( vec_2.y == 1.32 );
}

TEST_CASE( "Vector normalization works", "[normalizeVector]") 
{
    Vector2D v{-69, 4.20};

    Vector2D v_hat = normalizeVector(v);

    // Check x.
    REQUIRE_THAT(v_hat.x, WithinAbs(-0.998153,1.0e-6));
    // Check y.
    REQUIRE_THAT(v_hat.y, WithinAbs(0.0607571,1.0e-6));
}

TEST_CASE( "Vector subtraction works", "[operator-]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9}, vab{0,0};

    vab = va - vb;

    // Check x.
    REQUIRE_THAT(vab.x, WithinAbs(-64.8,1.0e-6));
    // Check y.
    REQUIRE_THAT(vab.y, WithinAbs(-2.7,1.0e-6));
}

TEST_CASE( "Vector addition works", "[operator+]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9}, vab{0,0};

    vab = va + vb;

    // Check x.
    REQUIRE_THAT(vab.x, WithinAbs(-73.2,1.0e-6));
    // Check y.
    REQUIRE_THAT(vab.y, WithinAbs(11.1,1.0e-6));
}

TEST_CASE( "Vector scaling works", "[operator*]") 
{
    double scale = -69.0;
    Vector2D v{-69, 4.20}, vs1{0,0}, vs2{0,0};

    vs1 = scale * v;
    vs2 = v * scale;

    // Check x.
    REQUIRE_THAT(vs1.x, WithinAbs(4761,1.0e-6));
    // Check y.
    REQUIRE_THAT(vs1.y, WithinAbs(-289.8,1.0e-6));

    // Check x.
    REQUIRE_THAT(vs2.x, WithinAbs(4761,1.0e-6));
    // Check y.
    REQUIRE_THAT(vs2.y, WithinAbs(-289.8,1.0e-6));
}
