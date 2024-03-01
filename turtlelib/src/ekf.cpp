#include <iostream>
#include <armadillo>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
    // State variable includes configuration of the robot, and X and y positions of the landmarks
    EKFSlam::EKFSlam(): 
    q{num_dof, arma::fill::zeros}, m{2 * num_landmarks, arma::fill::zeros}, Xi{num_dof + 2 * num_landmarks, arma::fill::zeros}, sigma{num_dof + 2 * num_landmarks, num_dof + 2 * num_landmarks, arma::fill::zeros}
    {
        initialize_covariance();
    }

    EKFSlam::EKFSlam(Pose2D turtle_pose_0): 
    q{num_dof, arma::fill::zeros}, m{2 * num_landmarks, arma::fill::zeros}, Xi{num_dof + 2 * num_landmarks, arma::fill::zeros}, sigma{num_dof + 2 * num_landmarks, num_dof + 2 * num_landmarks, arma::fill::zeros}
    {
        initialize_covariance();
        initialize_pose(turtle_pose_0);
    }

    void EKFSlam::initialize_covariance()
    {
        arma::mat sigma_0_q = arma::zeros<arma::mat>(num_dof, num_dof); // We are absolutely sure about the initial pose
        arma::mat sigma_0_m = arma::eye(2 * num_landmarks, 2 * num_landmarks) * 1e6; // Uncertainity in sensing, very high with no knowledge of obstacles
        arma::mat zeros_12 = arma::zeros<arma::mat>(num_dof, 2*num_landmarks); // Zeros due to sensing and localization noise being independent
        arma::mat zeros_21 = arma::zeros<arma::mat>(2*num_landmarks, num_dof); // Zeros due to sensing and localization noise being independent
        sigma =
            arma::join_vert(
            arma::join_horiz(sigma_0_q, zeros_12), 
            arma::join_horiz(zeros_21, sigma_0_m));
    }

    void EKFSlam::initialize_pose(Pose2D turtle_pose_0)
    {
        q(0) = turtle_pose_0.theta;
        q(1) = turtle_pose_0.x;
        q(2) = turtle_pose_0.y;
        
        update_state_vector();
    }

    void EKFSlam::update_state_vector()
    {
        // Populate with pose vector
        for (int pose_index = 0; pose_index < num_dof; pose_index++)
        {
            Xi(pose_index) = q(pose_index);
        }
        // Populate with map vector
        for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++)
        {
            Xi(num_dof + 2*landmark_index) = m(2*landmark_index); // X coordinate of landmark
            Xi(num_dof + 2*landmark_index + 1) = m(2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    void EKFSlam::update_pose_and_map()
    {
        // Populate pose vector
        for (int pose_index = 0; pose_index < num_dof; pose_index++)
        {
            q(pose_index) = Xi(pose_index);
        }
        // Populate map vector
        for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++)
        {
            m(2*landmark_index) = Xi(num_dof + 2*landmark_index); // X coordinate of landmark
            m(2*landmark_index + 1) = Xi(num_dof + 2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    /// \brief - Predict the state and covariance, every time a twist input is given
    void EKFSlam::predict(Twist2D twist)
    {
        // Check for proper twist
        if(!almost_equal(twist.y , 0.0))
        {
            throw std::runtime_error("Improper twist for estimation!");
        }

        // Update twist
        u(0) = twist.omega;
        u(1) = twist.x;
        u(2) = 0.0;

        // First we predict the covariance ˆΣ-_t using current A_t which is calculated using the previous state ξ_{t−1}, and current input u_t.
        // ˆΣ¯_t = A_t ˆΣ_{t−1} A_t^{T} + Q-,

        // Calculate A matrix.
        arma::mat zeros_12{num_dof, 2 * num_landmarks, arma::fill::zeros};
        arma::mat zeros_21{2 * num_landmarks, num_dof, arma::fill::zeros};
        arma::mat zeros_22{2 * num_landmarks, 2 * num_landmarks, arma::fill::zeros};
        arma::mat pose_state_matrix(num_dof, num_dof, arma::fill::zeros);

        // Zero rotational velocity
        if (almost_equal(u(0), 0.0)) 
        {     
            pose_state_matrix(1, 0) = -u(1) * sin(q(0));
            pose_state_matrix(2, 0) = u(1) * cos(q(0));
        } 
        // Non-zero rotational velocity
        else 
        {   
            pose_state_matrix(1, 0) = -(u(1) / u(0)) * cos(q(0)) + (u(1) / u(0)) * cos(normalize_angle(q(0) + u(0)));
            pose_state_matrix(2, 0) = -(u(1) / u(0)) * sin(q(0)) + (u(1) / u(0)) * sin(normalize_angle(q(0) + u(0)));
        }

        A = I +
            arma::join_vert(
            arma::join_horiz(pose_state_matrix, zeros_12),
            arma::join_horiz(zeros_21, zeros_22));

        // Represent process noise Q as Q_bar
        arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));
        
        // Update covariance matrix
        sigma = A * sigma * A.t() + Q_bar;
        
        // Now we predict the mean pose ˆξ-_t using the current pose ξ_{t−1}
        // ˆξ¯_t = g(ˆξ_{t−1}, u_t, 0)
        
        // Predict mean pose nonlinearly
        Transform2D pose_as_tf_Twb{Vector2D{q(1), q(2)}, q(0)}; // Original mean pose
        Transform2D twist_as_tf_TbB = integrate_twist(twist);   // Change in mean pose
        Transform2D newpose_as_tf_TwB = pose_as_tf_Twb * twist_as_tf_TbB;  // Final mean pose

        // Update pose and state
        q(0) = newpose_as_tf_TwB.rotation();
        q(1) = newpose_as_tf_TwB.translation().x;
        q(2) = newpose_as_tf_TwB.translation().y;
        update_state_vector();

        // // Check covariance matrix (symmetric and positive semi-definite)
        // // Check if symmetric
        // if (!(sigma.is_symmetric(1e-8)))
        // {
        //     throw std::runtime_error("Covariance is Assymetric!!!");
        // }
        // // Check if positive semi-definite
        // arma::vec eigvals = arma::eig_sym(sigma);
        // if (!(arma::all(eigvals >= 0)))
        // {
        //     throw std::runtime_error("Covariance is not Positive Semi-Definite!!!");
        // }

    }

    /// \brief Correct the state and covariance using the Kalman gain, every time a landmark is sensed
    void EKFSlam::correct(double x, double y, size_t j)
    {
        // Convert relative measurements to range-bearing
        // r_j = (x^2 + y^2)^0.5
        // ϕ_j = atan2(y, x)
        double r_j = magnitude(Vector2D{x, y});
        double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

        // If landmark has not been seen before, add it to the map
        // Since our pose variables are predictions (distributions), our map is also a prediction. 
        if (seen_landmarks.find(j) == seen_landmarks.end()) 
        {
            // Initialize the landmark prediction as x and y predictions in map frame
            m(2 * (j-1)) = q(1) + r_j * cos(phi_j + q(0)); // ˆm_{x,j}
            m(2 * (j-1) + 1) = q(2) + r_j * sin(phi_j + q(0)); // ˆm_{y,j}
            // Insert the new landmark index in the unordered_set
            seen_landmarks.insert(j);
            update_state_vector();
        }
        // Actual Measurement of that landmark. Not a distribution.
        z_i(0) = r_j;
        z_i(1) = phi_j;

        // Relative predictions of landmark position, as cartesian coordinates
        // δ_{x,j} = ˆm_{x,j} − ˆx_t
        // δ_{y,j} = ˆm_{y,j} − ˆy_t
        // d_j = δ_{x,j}^2 + δ_{y,j}^2
        Vector2D delta_j{m(2*(j-1)) - q(1), m(2*(j-1)+1) - q(2)};
        double d_j = std::pow(magnitude(delta_j), 2);

        // Relative predictions of landmark position, as range-bearing
        double r_j_hat = std::sqrt(d_j);
        double phi_j_hat = normalize_angle(atan2(delta_j.y, delta_j.x) - q(0));
        z_i_hat(0) = r_j_hat;
        z_i_hat(1) = phi_j_hat;

        // Calculate H matrix
        arma::mat small_H_first{2, num_dof, arma::fill::zeros}; // Dependence on pose
        arma::mat zeros_2_first{2, 2 * (j-1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
        arma::mat small_H_second{2, 2, arma::fill::zeros}; // Dependence on sensed landmark
        arma::mat zeros_2_second{2, 2 * num_landmarks - 2 * j, arma::fill::zeros}; // Dependence on landmarks having larger indices

        small_H_first(0, 0) = 0.0;
        small_H_first(0, 1) = -delta_j.x / std::sqrt(d_j);
        small_H_first(0, 2) = -delta_j.y / std::sqrt(d_j);
        small_H_first(1, 0) = -1;
        small_H_first(1, 1) = delta_j.y / d_j;
        small_H_first(1, 2) = -delta_j.x / d_j;

        small_H_second(0, 0) = delta_j.x / std::sqrt(d_j);
        small_H_second(0, 1) = delta_j.y / std::sqrt(d_j);
        small_H_second(1, 0) = -delta_j.y / d_j;
        small_H_second(1, 1) = delta_j.x / d_j;

        H_i = arma::join_horiz(arma::join_horiz(small_H_first, zeros_2_first), arma::join_horiz(small_H_second, zeros_2_second));

        // Sensor noise matrix
        R = arma::mat{2, 2, arma::fill::eye} * R_noise;
        // Rj = R.submat(j, j, j + 1, j + 1);

        // Kalman gain
        // K_i = Σ¯_t H_{i}^{T} (H_i Σ¯_t H_{i}^{T} + R)^{-1}
        K_i = sigma * H_i.t() * (H_i * sigma * H_i.t() + R).i();

        // Update state to corrected prediction
        
        // Subtract z_i and z_i_hat correcctly
        arma::colvec z_i_diff{2, arma::fill::zeros};
        z_i_diff(0) = z_i(0) - z_i_hat(0);
        z_i_diff(1) = normalize_angle(z_i(1) - z_i_hat(1));

        // ξ_t = ˆξ¯_t + K_i (z^i_t − ˆz^i_t)
        Xi = Xi + K_i * (z_i_diff);    
        update_pose_and_map();

        // Update covariance
        // Σ_t = (I − K_i H_i) Σ¯_t
        sigma = (I - K_i * H_i) * sigma;

        // Srikanth is super cool
    }

    /// GETTERS

    /// \brief get current pose prediction/correction of the robot
    Pose2D EKFSlam::pose() const
    {
        return Pose2D{q(0), q(1), q(2)};
    }

    /// \brief get current map vector prediction/correction of the robot
    arma::colvec EKFSlam::map() const
    {
        return m;
    }

    /// \brief get current state vector prediction/correction of the robot
    arma::colvec EKFSlam::state_vector() const
    {
        return Xi;
    }

    /// \brief get current covariance matrix prediction/correction of the robot
    arma::mat EKFSlam::covariance_matrix() const
    {
        return sigma;
    }

    /// \brief get current twist input to the robot
    Twist2D EKFSlam::twist() const
    {
        return Twist2D{u(0), u(1), u(2)};
    }

    /// \brief get current state matrix prediction of the robot
    arma::mat EKFSlam::state_matrix() const
    {
        return A;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::actual_measurement() const
    {
        return z_i;
    }

    /// \brief get the current actual sensor measurement 
    arma::mat EKFSlam::predicted_measurement() const
    {
        return z_i_hat;
    }

    /// \brief get the current predicted sensor matrix 
    arma::mat EKFSlam::sensor_matrix() const
    {
        return H_i;
    }
    
}