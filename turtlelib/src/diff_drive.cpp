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
    wheel_radius{1.0}, wheel_sep{1.0}, phi{0.0, 0.0}, q{0.0, 0.0, 0.0} 
    {}

    // Create a general Diff Drive.
    DiffDrive::DiffDrive(double radius, double sep, wheelAngles wheels, pose2D pose) :
    wheel_radius{radius}, wheel_sep{sep}, phi{normalize_angle(wheels.left), normalize_angle(wheels.right)}, q{normalize_angle(pose.theta), pose.x, pose.y} 
    {}

    // Drive the robot forward through the wheels (compute forward velocity kinematics)
    void DiffDrive::driveWheels(wheelAngles delta_phi)
    {
        // Transform from world {w} to body {b}
        Transform2D T_wb{Vector2D{q.x, q.y}, q.theta};

        // Get body twist from wheel rotations
        Twist2D V_b;
        V_b.omega = (wheel_radius / wheel_sep) * (delta_phi.right - delta_phi.left);
        V_b.x = (wheel_radius / 2) * cos(q.theta) * (delta_phi.right + delta_phi.left);
        V_b.y = (wheel_radius / 2) * sin(q.theta) * (delta_phi.right + delta_phi.left);
        
        // Transform from body {b} to new body {B}
        Transform2D TbB = integrate_twist(V_b);

        // Transform from world {w} to new body {B}
        Transform2D T_wB = T_wb * TbB;

        // Update wheel angles
        phi.left = normalize_angle(phi.left + delta_phi.left);
        phi.right = normalize_angle(phi.right + delta_phi.right);

        // Update pose
        q.theta = T_wB.rotation();
        q.x = T_wB.translation().x;
        q.y = T_wB.translation().y;
    }

    // GETTERS.

    // Get wheel radius
    double DiffDrive::DiffDrive::radius() const
    {
        return wheel_radius;
    }

    // Get wheel separation
    double DiffDrive::DiffDrive::separation() const
    {
        return wheel_sep;
    }

    // Get wheel angles
    wheelAngles DiffDrive::DiffDrive::wheels() const
    {
        return phi;
    }

    // Get pose
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