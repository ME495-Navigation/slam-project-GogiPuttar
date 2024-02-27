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
        // ˆΣ-_t = A_t ˆΣ_{t−1} A_t^{T} + Q-,

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

        // Calculate Q_bar matrix
        arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));
        
        // Update covariance matrix
        sigma = A * sigma * A.t() + Q_bar;
        
        // Now we predict the mean pose ˆξ-_t using the current pose ξ_{t−1}
        // ˆξ-_t = g(ˆξ_{t−1}, u_t, 0)
        
        // Predict mean pose nonlinearly
        Transform2D pose_as_tf_Twb{Vector2D{q(1), q(2)}, q(0)}; // Original mean pose
        Transform2D twist_as_tf_TbB = integrate_twist(twist);   // Change in mean pose
        Transform2D newpose_as_tf_TwB = pose_as_tf_Twb * twist_as_tf_TbB;  // Final mean pose

        // Update pose and state
        q(0) = newpose_as_tf_TwB.rotation();
        q(1) = newpose_as_tf_TwB.translation().x;
        q(2) = newpose_as_tf_TwB.translation().y;
        update_state_vector();
    }

    // void EKFSlam::EKFSlam_Correct(double x, double y, size_t j)
    // {
    // // Convert relative measurements to range bearing
    // double r_j = std::sqrt(x * x + y * y);
    // double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

    // // Check if landmark has been seen before
    // if (seen_landmarks.find(j) == seen_landmarks.end()) {
    //     // Initialize the landmark estimate x and y coordinates in Xi
    //     Xi(m + 2 * j) = Xi(1) + r_j * cos(phi_j + Xi(0));
    //     Xi(m + 2 * j + 1) = Xi(2) + r_j * sin(phi_j + Xi(0));
    //     // Insert the new landmark index in the unordered_set
    //     seen_landmarks.insert(j);
    // }

    // // Actual measurements
    // zj(0) = r_j;
    // zj(1) = phi_j;

    // // Estimate measurements
    // Vector2D estimate_rel_dist_j;
    // estimate_rel_dist_j.x = Xi(m + 2 * j) - Xi(1);
    // estimate_rel_dist_j.y = Xi(m + 2 * j + 1) - Xi(2);
    // double d_j = estimate_rel_dist_j.x * estimate_rel_dist_j.x + estimate_rel_dist_j.y *
    //     estimate_rel_dist_j.y;
    // double r_j_hat = std::sqrt(d_j);
    // double phi_j_hat = normalize_angle(atan2(estimate_rel_dist_j.y, estimate_rel_dist_j.x) - Xi(0));
    // zj_hat(0) = r_j_hat;
    // zj_hat(1) = phi_j_hat;

    // // Calculate H matrix
    // arma::mat zeros_1j(2, 2 * j);
    // arma::mat zeros_1nj(2, 2 * n - 2 * (j + 1));
    // arma::mat temp1(2, 3);
    // arma::mat temp2(2, 2);

    // temp1(1, 0) = -1;
    // temp1(0, 1) = -estimate_rel_dist_j.x / std::sqrt(d_j);
    // temp1(1, 1) = estimate_rel_dist_j.y / d_j;
    // temp1(0, 2) = -estimate_rel_dist_j.y / std::sqrt(d_j);
    // temp1(1, 2) = -estimate_rel_dist_j.x / d_j;

    // temp2(0, 0) = estimate_rel_dist_j.x / std::sqrt(d_j);
    // temp2(1, 0) = -estimate_rel_dist_j.y / d_j;
    // temp2(0, 1) = estimate_rel_dist_j.y / std::sqrt(d_j);
    // temp2(1, 1) = estimate_rel_dist_j.x / d_j;

    // Hj = arma::join_rows(arma::join_rows(temp1, zeros_1j), arma::join_rows(temp2, zeros_1nj));

    // // Noise
    // R = arma::mat{2 * n, 2 * n, arma::fill::eye} *R_noise;
    // Rj = R.submat(j, j, j + 1, j + 1);

    // // Kalman gain
    // Ki = covariance * Hj.t() * (Hj * covariance * Hj.t() + Rj).i();

    // // Update state to corrected prediction
    // Xi = Xi + Ki * (zj - zj_hat);

    // // Update covariance
    // covariance = (I - Ki * Hj) * covariance;
    // }

    // size_t EKFSlam::Data_association(double x, double y)
    // {
    // // Convert relative measurements to range bearing
    // double r_j = std::sqrt(x * x + y * y);
    // double phi_j = std::atan2(y, x);      // Normalize ?? TODO ??

    // // Create a temp Xi with new temp landmark
    // Xi_temp = Xi;
    // // Initialize the landmark estimate x and y coordinates in Xi
    // Xi_temp(m + 2*N + 1) = Xi_temp(1) + r_j * cos(phi_j + Xi_temp(0));
    // Xi_temp(m + 2*N + 1 + 1) = Xi_temp(2) + r_j * sin(phi_j + Xi_temp(0));


    // // Actual measurements
    // zj(0) = r_j;
    // zj(1) = phi_j;

    // std::vector<arma::mat> distances{}; // Mahalanobis distance for each landmark

    // for (int k = 0; k < N+1; k++)
    // {
    //     /*
    //         Step 4.1.1
    //     */
    //     // Estimate measurements
    //     Vector2D estimate_rel_dist_j;
    //     estimate_rel_dist_j.x = Xi_temp(m + 2 * k) - Xi_temp(1);
    //     estimate_rel_dist_j.y = Xi_temp(m + 2 * k + 1) - Xi_temp(2);
    //     double d_j = estimate_rel_dist_j.x * estimate_rel_dist_j.x + estimate_rel_dist_j.y *
    //         estimate_rel_dist_j.y;

    //     // Calculate H matrix
    //     arma::mat zeros_1j(2, 2 * k);
    //     arma::mat zeros_1nj(2, 2 * n - 2 * (k + 1));
    //     arma::mat temp1(2, 3);
    //     arma::mat temp2(2, 2);

    //     temp1(1, 0) = -1;
    //     temp1(0, 1) = -estimate_rel_dist_j.x / std::sqrt(d_j);
    //     temp1(1, 1) = estimate_rel_dist_j.y / d_j;
    //     temp1(0, 2) = -estimate_rel_dist_j.y / std::sqrt(d_j);
    //     temp1(1, 2) = -estimate_rel_dist_j.x / d_j;

    //     temp2(0, 0) = estimate_rel_dist_j.x / std::sqrt(d_j);
    //     temp2(1, 0) = -estimate_rel_dist_j.y / d_j;
    //     temp2(0, 1) = estimate_rel_dist_j.y / std::sqrt(d_j);
    //     temp2(1, 1) = estimate_rel_dist_j.x / d_j;

    //     arma::mat Hk = arma::join_rows(arma::join_rows(temp1, zeros_1j), arma::join_rows(temp2, zeros_1nj));


    //     /*
    //         Step 4.1.2
    //     */
    //     // Noise
    //     R = arma::mat{2 * n, 2 * n, arma::fill::eye} *R_noise;
    //     arma::mat Rk = R.submat(k, k, k + 1, k + 1);

    //     arma::mat covariance_k = Hk*covariance*Hk.t() + Rk;


    //     /*
    //         Step 4.1.3
    //     */
    //     double r_j_hat = std::sqrt(d_j);
    //     double phi_j_hat = normalize_angle(atan2(estimate_rel_dist_j.y, estimate_rel_dist_j.x) - Xi_temp(0));
    //     zj_hat(0) = r_j_hat;
    //     zj_hat(1) = phi_j_hat;


    //     /*
    //         Step 4.1.4 -> Calc Mahalanobis distance
    //     */
    //     arma::mat dist_k = ((zj - zj_hat).t())*(covariance_k.i())*(zj - zj_hat);

    //     distances.push_back(dist_k);
    // }

    // /*
    //     Step 4.2 & 4.3
    // */
    // // Set distance threshold to distance N+1
    // arma::mat distance_threshold = distances.at(distances.size()-1);
    // size_t index = N+1;
    // bool new_landmark = true;

    // for (size_t i = 0; i<distances.size(); i++)
    // {
    //     if (distances.at(i)(0) < distance_threshold(0))
    //     {
    //         distance_threshold = distances.at(i);
    //         index = i;
    //         new_landmark = false; // Not a new landmark
    //     }
    // }

    // /*
    //     Step 4.4
    // */
    // if (new_landmark == true) // If it is a new landmark increase N
    // {
    //     N++;
    // }

    // return index;
    // }

    // Robot_configuration EKFSlam::EKFSlam_config()
    // {
    // return {Xi(1), Xi(2), Xi(0)};
    // }

    // arma::colvec EKFSlam::EKFSlam_Xi()
    // {
    // return Xi;
    // }

    /// \brief set the initial state of the robot
    Pose2D EKFSlam::pose() const
    {
        return Pose2D{q(0), q(1), q(2)};
    }

    /// \brief set the initial state of the robot
    arma::colvec EKFSlam::map() const
    {
        return m;
    }

    /// \brief set the initial state of the robot
    arma::colvec EKFSlam::state_vector() const
    {
        return Xi;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::covariance_matrix() const
    {
        return sigma;
    }

    /// \brief set the initial state of the robot
    Twist2D EKFSlam::twist() const
    {
        return Twist2D{u(0), u(1), u(2)};
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::state_matrix() const
    {
        return A;
    }
    
}