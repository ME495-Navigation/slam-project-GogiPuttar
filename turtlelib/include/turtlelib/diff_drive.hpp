#ifndef TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{

    /// \brief represent a mobile robot's pose
    struct pose2D
    {
        /// \brief angle with the world frame
        double theta = 0.0;

        /// \brief x position in the world frame
        double x = 0.0;

        /// \brief y position in world frame
        double y = 0.0;
    };

    /// \brief represent a mobile robot's pose
    struct wheelAngles
    {
        /// \brief left wheel angle
        double left = 0.0;

        /// \brief right wheel angle
        double right = 0.0;
    };

    /// \brief models the kinematics of a differential drive robot with a given wheel track and wheel radius
    class DiffDrive
    {

    private:

        /// \brief angle of rotation of wheels, in radians
        wheelAngles phi;
    
        /// \brief pose of the mobile robot in the world frame
        pose2D q;

    public:

        /// \brief Initialize the kinematics for a new Diff Drive robot
        DiffDrive();

        /// \brief Initialize the kinematics for a general Diff Drive robot
        explicit DiffDrive(wheelAngles wheels, pose2D pose);

        // Get translation vector.
        wheelAngles wheels() const;

        // Get rotation angle.
        pose2D pose() const;

        // Drive the robot forward through the wheels (compute forward velocity kinematics)
        void driveWheels(wheelAngles delta_phi) const;

        // Drive the robot forward by defining its twist (compute inverse velocity kinematics)
        void driveTwist(Twist2D delta_q) const;

        // /// \brief create a transformation that is a pure translation
        // /// \param trans - the vector by which to translate
        // explicit DiffDrive(Vector2D trans);

        // /// \brief create a pure rotation
        // /// \param radians - angle of the rotation, in radians
        // explicit DiffDrive(double radians);

        // /// \brief Create a transformation with a translational and rotational
        // /// component
        // /// \param trans - the translation
        // /// \param radians - the rotation, in radians
        // DiffDrive(Vector2D trans, double radians);

    };
}

#endif
