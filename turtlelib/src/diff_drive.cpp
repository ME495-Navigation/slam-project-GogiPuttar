#include <cstdio>
#include <cmath>
#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
    // CONSTRUCTORS.

    // Create a new Diff Drive.
    DiffDrive::DiffDrive() : 
    phi{0.0, 0.0}, q{0.0, 0.0, 0.0} 
    {}

    // Create a general Diff Drive.
    DiffDrive::DiffDrive(wheelAngles wheels, pose2D pose) :
    phi{normalize_angle(wheels.left), normalize_angle(wheels.right)}, q{normalize_angle(pose.theta), pose.x, pose.y} 
    // phi{0.0, }, q{pose.theta, pose.x, pose.y} 
    {}

    // GETTERS.

    // Get translation vector.
    wheelAngles DiffDrive::DiffDrive::wheels() const
    {
        return phi;
    }

    // Get rotation angle.
    pose2D DiffDrive::DiffDrive::pose() const
    {
        return q;
    }

    // // Create a pure translation transform.
    // DiffDrive::DiffDrive(double left_wheel_angle, double right_wheel_angle, pose2D pose) :
    // phi_l{left_wheel_angle}, phi_r{right_wheel_angle}, q{pose}
    // {}

    // // Create a pure rotation transform.
    // DiffDrive::DiffDrive(double angle) :
    // rotationAngle{normalize_angle(angle)}
    // {}

    // // Create a transform with translation and rotation.
    // // First rotate, then translate (in intermediate frame); which is equivalent to first translate, then rotate (in global frame).
    // DiffDrive::DiffDrive(Vector2D displacement, double angle) :
    // translationVector{displacement}, rotationAngle{normalize_angle(angle)}
    // {}

}