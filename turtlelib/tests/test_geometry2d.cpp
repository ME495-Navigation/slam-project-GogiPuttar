#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"

using turtlelib::normalize_angle;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Angles are normalized", "[normalize_angle]" ) {
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

