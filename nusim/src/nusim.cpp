/// \file
/// \brief The nusim node is a simulation and visualization tool for the turtlebot3 robots.
///        It uses rviz2 for visualization and provides a simulated environment. The package
///        creates stationary walls and obstacles and track the position of a red robot.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0 (double): Initial x coordinate of the robot [m]
///     \param y0 (double): Initial y coordinate of the robot [m]
///     \param theta0 (double): Initial theta angle of the robot [radians]
///     \param obstacles/x (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles/y (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles/r (double): Radius of cylindrical obstacles [m]
///     \param arena_x_length (double): Inner length of arena in x direction [m]
///     \param arena_y_length (double): Inner length of arena in y direction [m]
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Current simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles that are
///                                                                displayed in Rviz
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker walls that are
///                                                            displayed in Rviz
///
/// SUBSCRIBES:
///     \param /red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel command velocity in
///                                                                   ticks
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleport robot to a specific pose
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts red turtle position

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;

/// \brief Generate random number
std::mt19937 & get_random()
{
  // static variables inside a function are created once and persist for the remainder of the program
  static std::random_device rd{};
  static std::mt19937 mt{rd()};
  // we return a reference to the pseudo-random number genrator object. This is always the
  // same object every time get_random is called
  return mt;
}

/// \brief This class publishes the current timestep of the simulation, obstacles and walls that
///        appear in Rviz as markers. The class has a timer_callback to continually update the
///        simulation at each timestep. The reset service resets the simulation to the initial
///        state thus restarting the simulation. A teleport service is available to teleport a
///        turtlebot to any pose. A broadcaster broadcasts the robots TF frames to a topic for
///        visualization in Rviz. The simulation operates in a loop, updating the state of the
///        world, publishing messages that provides state information simulating a real robot,
///        and processing service/subscriber callbacks for commands for the next time step. The
///        loop runs at a fixed frequency until termination.
///
///  \param rate (int): Timer callback frequency [Hz]
///  \param x0_ (double): Initial x coordinate of the robot [m]
///  \param y0_ (double): Initial y coordinate of the robot [m]
///  \param theta0_ (double): Initial theta angle of the robot [radians]
///  \param x_ (double): Current x coordinate of the robot [m]
///  \param y_ (double): Current y coordinate of the robot [m]
///  \param theta_ (double): Current theta angle of the robot [radians]
///  \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///  \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///  \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///  \param arena_x_length_ (double): Inner length of arena in x direction [m]
///  \param arena_y_length_ (double): Inner length of arena in y direction [m]

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    // Parameter descirption
    auto rate_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto x0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto y0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto theta0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_x_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_y_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto arena_x_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto arena_y_des = rcl_interfaces::msg::ParameterDescriptor{};
    // auto wheelradius_des = rcl_interfaces::msg::ParameterDescriptor{};
    // auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};

    rate_des.description = "Timer callback frequency [Hz]";
    x0_des.description = "Initial x coordinate of the robot [m]";
    y0_des.description = "Initial y coordinate of the robot [m]";
    theta0_des.description = "Initial theta angle of the robot [radians]";
    obstacles_x_des.description = "Vector of x coordinates for each obstacle [m]";
    obstacles_y_des.description = "Vector of y coordinates for each obstacle [m]";
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    arena_x_des.description = "Length of arena along x [m]";
    arena_y_des.description = "Length of arena along y [m]";
    // wheelradius_des.description = "The radius of the wheels [m]";
    // track_width_des.description = "The distance between the wheels [m]";
    
    // TODO !! CHECK DESCRIPTIONS

    // Declare default parameters values
    declare_parameter("rate", 200, rate_des);     // Hz for timer_callback
    declare_parameter("x0", 0.0, x0_des);
    declare_parameter("y0", 0.0, y0_des);
    declare_parameter("theta0", 0.0, theta0_des);
    declare_parameter("obstacles/x", std::vector<double>{}, obstacles_x_des);
    declare_parameter("obstacles/y", std::vector<double>{}, obstacles_y_des);
    declare_parameter("obstacles/r", 0.0, obstacles_r_des);
    declare_parameter("arena_x_length", 0.0, arena_x_des);
    declare_parameter("arena_y_length", 0.0, arena_y_des);
    // declare_parameter("wheelradius", -1.0, wheelradius_des);
    // declare_parameter("track_width", -1.0, track_width_des);

    // Get params - Read params from yaml file that is passed in the launch file
    int rate = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles/x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles/y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles/r").get_parameter_value().get<double>();
    arena_x_ = get_parameter("arena_x_length").get_parameter_value().get<double>();
    arena_y_ = get_parameter("arena_y_length").get_parameter_value().get<double>();

    // Set current robot pose equal to initial pose
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    // Timer timestep [seconds]
    dt_ = 1.0 / static_cast<double>(rate);

    // Create obstacles
    create_obstacles_array();
    
    // Create arena
    create_arena_walls();

    // Get transform from robot to world
    // T_world_red_ =
    //   turtlelib::Transform2D{{turtle_.configuration().x, turtle_.configuration().y},
    //   turtle_.configuration().theta};
    // T_red_world_ = T_world_red_.inv();

    // Publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    // red_turtle_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);

    //Subscribers
    // red_wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    //   "red/wheel_cmd", 10, std::bind(
    //     &Nusim::red_wheel_cmd_callback, this,
    //     std::placeholders::_1));

    // Reset service
    reset_server_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Teleport service
    teleport_server_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&Nusim::timer_callback, this));
    // timer2_ = create_wall_timer(
    //   std::chrono::milliseconds(1000 / 5),
    //   std::bind(&Nusim::timer_callback_2, this));
  }

private:
  // Variables
  size_t timestep_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_, y_, theta_;     // Theta in radians, x & y in meters.
  double x0_ = 0.0;
  double y0_ = 0.0;
  double theta0_ = 0;
  double dt_ = 0.0; // Nusim Timer
  double obstacles_r_ = 0.01;    // Size of obstacles
  double obstacles_h_ = 0.25;
  double wall_height_ = 0.25;   // Height of walls [m]
  double wall_thickness_ = 0.156;  // Thickness of walls [m]
  double arena_x_ = 0.0;    // Length of the arena along x [m]
  double arena_y_ = 0.0;  // Length of the arena along y [m]
//   double wheelradius_;
//   double track_width_;
  std::vector<double> obstacles_x_;    // Location of obstacles
  std::vector<double> obstacles_y_;
  visualization_msgs::msg::MarkerArray obstacles_;
  visualization_msgs::msg::MarkerArray walls_;
//   geometry_msgs::msg::PoseStamped red_pose_stamped_;
//   turtlelib::Wheel delta_wheel_pos_{0.0, 0.0};
//   turtlelib::Wheel new_wheel_pos_;
//   turtlelib::Wheel old_wheel_pos_{0.0, 0.0};
//   turtlelib::DiffDrive turtle_;
  turtlelib::Transform2D T_world_red_{};
  turtlelib::Transform2D T_red_world_{};

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_turtle_publisher_;
//   rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;

  /// \brief Subscription callback function for wheel_cmd topic
//   void red_wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
//   {
//     // To generate a gaussian variable:
//     double left_noise = 0.0;
//     double right_noise = 0.0;

//     // Convert wheel cmd ticks to rad/sec and add noise if the wheel is commanded to move
//     if (msg.left_velocity != 0) {
//       left_noise = noise_(get_random());
//     }

//     if (msg.right_velocity != 0) {
//       right_noise = noise_(get_random());
//     }

//     new_wheel_vel_.left = static_cast<double>(msg.left_velocity) * motor_cmd_per_rad_sec_ +
//       left_noise;
//     new_wheel_vel_.right = static_cast<double>(msg.right_velocity) * motor_cmd_per_rad_sec_ +
//       right_noise;
//   }

  /// \brief Updates the red turtle's configuration
//   void update_red_turtle_config()
//   {
//     double left_slip = slip_(get_random());  // Add slip to wheel position
//     double right_slip = slip_(get_random());
//     delta_wheel_pos_.left = new_wheel_vel_.left * (1 + left_slip) * dt_;  // Change in position
//     delta_wheel_pos_.right = new_wheel_vel_.right * (1 + right_slip) * dt_;
//     turtle_.ForwardKinematics(delta_wheel_pos_);  // Update robot position
//     // Check collision with obstacles
//     check_collision();
//     x_ = turtle_.configuration().x;
//     y_ = turtle_.configuration().y;
//     theta_ = turtle_.configuration().theta;
//     update_sensor_data();
//   }

  /// \brief Check collision with obstacles
//   void check_collision()
//   {
//     for (size_t i = 0; i < obstacles_x_.size(); i++) {
//       double dx = turtle_.configuration().x - obstacles_x_.at(i);
//       double dy = turtle_.configuration().y - obstacles_y_.at(i);
//       double eucl_distance = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));

//       // Check if collision occured
//       if (eucl_distance < collision_radius_ + obstacles_r_) {
//         // // Vector between robot and obstacle
//         turtlelib::Vector2D V{dx, dy};
//         turtlelib::Vector2D V_normal = turtlelib::normalize(V);
//         // Distance to move back
//         double collision_dis = collision_radius_ + obstacles_r_ - eucl_distance;
//         // New robot configuration
//         turtlelib::Robot_configuration after_collision{};
//         after_collision.x = turtle_.configuration().x + collision_dis * V_normal.x;
//         after_collision.y = turtle_.configuration().y + collision_dis * V_normal.y;
//         after_collision.theta = turtle_.configuration().theta;
//         turtle_.set_configuration(after_collision);
//       }
//     }
//   }

  /// \brief Reset the simulation
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  /// \brief Teleport the robot to a specified pose
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  /// \brief Broadcast the TF frames of the robot
  void broadcast_red_turtle()
  {
    geometry_msgs::msg::TransformStamped t_;

    t_.header.stamp = get_clock()->now();
    // t_.header.stamp.nanosec += 50000000; // TODO - Fixes the fake obstacle in Rviz
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.transform.translation.x = x_;
    t_.transform.translation.y = y_;
    t_.transform.translation.z = 0.0;     // Turtle only exists in 2D

    tf2::Quaternion q_;
    q_.setRPY(0, 0, theta_);     // Rotation around z-axis
    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t_);

    // if (timestep_ % 100 == 1) {
    //   red_turtle_NavPath();
    // }
  }

  /// \brief Create the red turtle's nav_msgs/Path
//   void red_turtle_NavPath()
//   {
//     // Update ground truth red turtle path
//     red_path_.header.stamp = get_clock()->now();
//     red_path_.header.frame_id = "nusim/world";
//     // Create new pose stamped
//     red_pose_stamped_.header.stamp = get_clock()->now();
//     red_pose_stamped_.header.frame_id = "nusim/world";
//     red_pose_stamped_.pose.position.x = x_;
//     red_pose_stamped_.pose.position.y = y_;
//     red_pose_stamped_.pose.position.z = 0.0;
//     tf2::Quaternion q_;
//     q_.setRPY(0, 0, theta_);     // Rotation around z-axis
//     red_pose_stamped_.pose.orientation.x = q_.x();
//     red_pose_stamped_.pose.orientation.y = q_.y();
//     red_pose_stamped_.pose.orientation.z = q_.z();
//     red_pose_stamped_.pose.orientation.w = q_.w();
//     // Append pose stamped
//     red_path_.poses.push_back(red_pose_stamped_);
//   }

  /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
  void create_obstacles_array()
  {
    if (obstacles_x_.size() != obstacles_y_.size()) {
      throw std::runtime_error("x and y coordinate lists are not the same length!");
    }

    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      visualization_msgs::msg::Marker obstacle_;
      obstacle_.header.frame_id = "nusim/world";
      obstacle_.header.stamp = get_clock()->now();
      obstacle_.id = i;
      obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_.action = visualization_msgs::msg::Marker::ADD;
      obstacle_.pose.position.x = obstacles_x_.at(i);
      obstacle_.pose.position.y = obstacles_y_.at(i);
      obstacle_.pose.position.z = obstacles_h_ / 2.0;
      obstacle_.pose.orientation.x = 0.0;
      obstacle_.pose.orientation.y = 0.0;
      obstacle_.pose.orientation.z = 0.0;
      obstacle_.pose.orientation.w = 1.0;
      obstacle_.scale.x = obstacles_r_ * 2.0;   // Diameter in x
      obstacle_.scale.y = obstacles_r_ * 2.0;   // Diameter in y
      obstacle_.scale.z = obstacles_h_;         // Height
      obstacle_.color.r = 1.0f;
      obstacle_.color.g = 0.0f;
      obstacle_.color.b = 0.0f;
      obstacle_.color.a = 1.0;
      obstacles_.markers.push_back(obstacle_);
    }
  }

  /// \brief Create walls as a MarkerArray and publish them to a topic to display them in Rviz
  void create_arena_walls()
  {
    for (int i = 0; i <= 3; i++) {
      visualization_msgs::msg::Marker wall_;
      wall_.header.frame_id = "nusim/world";
      wall_.header.stamp = get_clock()->now();
      wall_.id = i;
      wall_.type = visualization_msgs::msg::Marker::CUBE;
      wall_.action = visualization_msgs::msg::Marker::ADD;

      if (i == 0) {
        wall_.pose.position.x = (arena_x_ + wall_thickness_)/2.0;
        wall_.pose.position.y = 0.0;
        
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.7071068;
        wall_.pose.orientation.w = 0.7071068;
      }

      else if (i == 1) {
        wall_.pose.position.x = 0.0;
        wall_.pose.position.y = (arena_y_ + wall_thickness_)/2.0;
        
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.0;
        wall_.pose.orientation.w = 1.0;
      }

      else if (i == 2) {
        wall_.pose.position.x = -(arena_x_ + wall_thickness_)/2.0;
        wall_.pose.position.y = 0.0;
        
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.7071068;
        wall_.pose.orientation.w = 0.7071068;
      }

      else if (i == 3) {
        wall_.pose.position.x = 0.0;
        wall_.pose.position.y = -(arena_y_ + wall_thickness_)/2.0;
        
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.0;
        wall_.pose.orientation.w = 1.0;
      }

      // Z - Position
      wall_.pose.position.z = wall_height_ / 2.0;

      if (i == 0 || i == 2) {
        wall_.scale.x = arena_y_ + 2 * wall_thickness_;
      } else {
        wall_.scale.x = arena_x_ + 2 * wall_thickness_;
      }
      wall_.scale.y = wall_thickness_;
      wall_.scale.z = wall_height_;

      wall_.color.r = 1.0f;
      wall_.color.g = 0.0f;
      wall_.color.b = 0.0f;
      wall_.color.a = 1.0;

      walls_.markers.push_back(wall_);
    }
  }

  /// \brief Main simulation timer loop
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    timestep_publisher_->publish(message);
    obstacles_publisher_->publish(obstacles_);
    walls_publisher_->publish(walls_);
    // if (draw_only_ == false) {
    //   update_red_turtle_config();
    //   sensor_data_publisher_->publish(sensor_data_);
    broadcast_red_turtle();
    //   red_turtle_publisher_->publish(red_path_);
    // }
  }

  /// \brief Calculate the euclidean distance
  /// \param x1 point 1 x-coordinate (double)
  /// \param y1 point 1 y-coordinate (double)
  /// \param x2 point 2 x-coordinate (double)
  /// \param y2 point 2 y-coordinate (double)
  /// \return euclidean distance (double)
  double euclidean_distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  /// \brief Secondary timer loop (5Hz)
//   void timer_callback_2()
//   {
//     if (draw_only_ == false) {
//       basic_laser_sensor();
//       lidar();
//     }
//   }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
