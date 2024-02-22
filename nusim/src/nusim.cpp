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
///     \param obstacles.x (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles.y (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
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
///     None
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
#include "nav_msgs/msg/path.hpp"
#include "nusim/srv/teleport.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

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
    auto wheel_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto encoder_ticks_per_rad_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_per_rad_sec_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto input_noise_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto slip_fraction_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto basic_sensor_variance_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto max_range_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto collision_radius_des = rcl_interfaces::msg::ParameterDescriptor{};

    rate_des.description = "Timer callback frequency [Hz]";
    x0_des.description = "Initial x coordinate of the robot [m]";
    y0_des.description = "Initial y coordinate of the robot [m]";
    theta0_des.description = "Initial theta angle of the robot [radians]";
    obstacles_x_des.description = "Vector of x coordinates for each obstacle [m]";
    obstacles_y_des.description = "Vector of y coordinates for each obstacle [m]";
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    arena_x_des.description = "Length of arena along x [m]";
    arena_y_des.description = "Length of arena along y [m]";
    wheel_radius_des.description = "Radius of the wheels [m]";
    track_width_des.description = "Separation between the wheels [m]";
    encoder_ticks_per_rad_des.description = "The number of encoder 'ticks' per radian [ticks/rad]";
    motor_cmd_per_rad_sec_des.description = "Radius per second in every motor command unit [(rad/s) / mcu]. Stupid name, I know!";
    input_noise_des.description = "Variance of noise due to non-ideal motor behaviour [(rad/s)^2]";
    slip_fraction_des.description = "Fractional range in which wheel can slip [-slip_fraction, +slip_fraction] [dimensionless]";
    basic_sensor_variance_des.description = "Variance in lidar [m^2]";
    max_range_des.description = "Range of the lidar [m]";
    collision_radius_des.description = "Collision radius of the robot [m]";

    // Declare default parameters values
    declare_parameter("rate", 200, rate_des);     // Hz for timer_callback
    declare_parameter("x0", 0.0, x0_des);         // Meters
    declare_parameter("y0", 0.0, y0_des);         // Meters
    declare_parameter("theta0", 0.0, theta0_des); // Radians
    declare_parameter("obstacles.x", std::vector<double>{}, obstacles_x_des); // Meters
    declare_parameter("obstacles.y", std::vector<double>{}, obstacles_y_des); // Meters
    declare_parameter("obstacles.r", 0.0, obstacles_r_des); // Meters
    declare_parameter("arena_x_length", 0.0, arena_x_des);  // Meters
    declare_parameter("arena_y_length", 0.0, arena_y_des);  // Meters
    declare_parameter("wheel_radius", -1.0, wheel_radius_des); // Meters
    declare_parameter("track_width", -1.0, track_width_des);  // Meters
    declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_des); // Ticks per radian
    declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_des); // MCU per radian/s
    declare_parameter("input_noise", -1.0, input_noise_des); // (radian/s)^2
    declare_parameter("slip_fraction", -1.0, slip_fraction_des); // Dimensionless (radian/radian)
    declare_parameter("basic_sensor_variance", -1.0, basic_sensor_variance_des); // 
    declare_parameter("max_range", -1.0, basic_sensor_variance_des); // Meters
    declare_parameter("collision_radius", -1.0, collision_radius_des); // Meters
    
    // Get params - Read params from yaml file that is passed in the launch file
    rate = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    arena_x_ = get_parameter("arena_x_length").get_parameter_value().get<double>();
    arena_y_ = get_parameter("arena_y_length").get_parameter_value().get<double>();
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<double>();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    

    check_yaml_params();

    // Initialize the differential drive kinematic state
    turtle_ = turtlelib::DiffDrive{wheel_radius_, track_width_, turtlelib::wheelAngles{}, turtlelib::pose2D{theta0_, x0_, y0_}};

    // Initialize the noise generators
    motor_control_noise_ = std::normal_distribution<>{0.0, std::sqrt(input_noise_)}; // Uncertainity in motor control
    wheel_slip_ = std::uniform_real_distribution<>{-slip_fraction_, slip_fraction_}; // Wheel slipping
    laser_noise_ = std::normal_distribution<>{0.0, std::sqrt(basic_sensor_variance_)}; // Uncertainity in laser triangulation

    // Timer timestep [seconds]
    dt_ = 1.0 / static_cast<double>(rate);

    // Create obstacles
    create_obstacles_array();

    // Create arena
    create_arena_walls();

    // Create ~/timestep publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    // Create ~/obstacles publisher
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    // Create ~/walls publisher
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    // Create red/sensor_data publisher
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10);
    // Create red/path publisher
    red_path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    // Create /fake_sensor
    fake_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10);

    // Create ~/reset service
    reset_server_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Create ~/teleport service
    teleport_server_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&Nusim::timer_callback, this));

    // Create red/wheel_cmd
    wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(
        &Nusim::wheel_cmd_callback, this,
        std::placeholders::_1));
  }

private:
  // Variables related to environment
  size_t timestep_;
  int rate;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x0_ = 0.0;          // Meters
  double y0_ = 0.0;          // Meters
  double theta0_ = 0;        // Radians
  double dt_ = 0.0; // Nusim Timer in seconds
  double obstacles_r_ = 0.01;    // Size of obstacles [m]
  double obstacles_h_ = 0.25;    // Height of obstacles [m]
  double wall_height_ = 0.25;   // Height of walls [m]
  double wall_thickness_ = 0.156;  // Thickness of walls [m]
  double arena_x_ = 0.0;    // Length of the arena along x [m]
  double arena_y_ = 0.0;  // Length of the arena along y [m]
  std::vector<double> obstacles_x_;    // Location of obstacles [m]
  std::vector<double> obstacles_y_;
  visualization_msgs::msg::MarkerArray obstacles_;
  visualization_msgs::msg::MarkerArray walls_;

  // Variables related to diff drive
  double wheel_radius_ = -1.0;
  double track_width_ = -1.0;
  nuturtlebot_msgs::msg::SensorData current_sensor_data_; // Encoder ticks
  nuturtlebot_msgs::msg::SensorData prev_sensor_data_; // Encoder ticks
  double encoder_ticks_per_rad_;
  double motor_cmd_per_rad_sec_;
  turtlelib::DiffDrive turtle_;

  // Variables related to visualization
  geometry_msgs::msg::PoseStamped red_path_pose_stamped_;
  nav_msgs::msg::Path red_path_;
  int path_frequency_ = 100; // per timer callback

  // Variables related to noise and sensing
  double input_noise_;
  std::normal_distribution<> motor_control_noise_{0.0, 0.0};
  double slip_fraction_;
  std::uniform_real_distribution<> wheel_slip_{0.0, 0.0};
  double basic_sensor_variance_;
  std::normal_distribution<> laser_noise_{0.0, 0.0};
  double max_range_;
  double fake_sensor_frequency_ = 5.0; //Hz
  visualization_msgs::msg::MarkerArray sensed_obstacles_;
  double collision_radius_;
  bool lie_group_collision_ = true;
  bool colliding_ = false;

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;

  /// \brief Reset the simulation
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timestep_ = 0;
    turtle_.q.x = x0_;
    turtle_.q.y = y0_;
    turtle_.q.theta = theta0_;
  }

  /// \brief Teleport the robot to a specified pose
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    turtle_.q.x = request->x;
    turtle_.q.y = request->y;
    turtle_.q.theta = request->theta;
  }

  /// \brief Broadcast the TF frames of the robot
  void broadcast_red_turtle()
  {
    geometry_msgs::msg::TransformStamped t_;

    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.transform.translation.x = turtle_.pose().x;
    t_.transform.translation.y = turtle_.pose().y;
    t_.transform.translation.z = 0.0;     

    tf2::Quaternion q_;
    q_.setRPY(0, 0, turtle_.pose().theta);     
    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t_);

    if (timestep_ % path_frequency_ == 1) {
      update_red_NavPath();
    }
  }

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

      // Wall on positive x-axis
      if (i == 0) {
        wall_.pose.position.x = (arena_x_ + wall_thickness_) / 2.0;
        wall_.pose.position.y = 0.0;

        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.7071068;
        wall_.pose.orientation.w = 0.7071068;
      }
      // Wall on positive y-axis
      else if (i == 1) {
        wall_.pose.position.x = 0.0;
        wall_.pose.position.y = (arena_y_ + wall_thickness_) / 2.0;

        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.0;
        wall_.pose.orientation.w = 1.0;
      }
      // Wall on negative x-axis
      else if (i == 2) {
        wall_.pose.position.x = -(arena_x_ + wall_thickness_) / 2.0;
        wall_.pose.position.y = 0.0;

        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.7071068;
        wall_.pose.orientation.w = 0.7071068;
      }
      // Wall on negative y-axis
      else if (i == 3) {
        wall_.pose.position.x = 0.0;
        wall_.pose.position.y = -(arena_y_ + wall_thickness_) / 2.0;

        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.0;
        wall_.pose.orientation.w = 1.0;
      }
      // Z - Position
      wall_.pose.position.z = wall_height_ / 2.0;

      // Wall dimensions
      if (i == 0 || i == 2) {
        wall_.scale.x = arena_y_ + 2 * wall_thickness_;
      } else {
        wall_.scale.x = arena_x_ + 2 * wall_thickness_;
      }
      wall_.scale.y = wall_thickness_;
      wall_.scale.z = wall_height_;

      // Red Walls
      wall_.color.r = 1.0f;
      wall_.color.g = 0.0f;
      wall_.color.b = 0.0f;
      wall_.color.a = 1.0;

      // Add wall to array
      walls_.markers.push_back(wall_);
    }
  }

  /// \brief wheel_cmd_callback subscription
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {    

    nuturtlebot_msgs::msg::WheelCommands noisy_wheel_cmd_;
    
    if(msg.left_velocity != 0.0)
    {
      noisy_wheel_cmd_.left_velocity = msg.left_velocity + motor_control_noise_(get_random());
    }
    else
    {
      noisy_wheel_cmd_.left_velocity = msg.left_velocity;
    }

    if(msg.right_velocity != 0.0)
    {
      noisy_wheel_cmd_.right_velocity = msg.right_velocity + motor_control_noise_(get_random());
    }
    else
    {
      noisy_wheel_cmd_.right_velocity = msg.right_velocity;
    }

    // Get current sensor data timestamp
    current_sensor_data_.stamp = get_clock()->now();

    // Update current sensor data as integer values
    current_sensor_data_.left_encoder = round((static_cast<double>(noisy_wheel_cmd_.left_velocity) * motor_cmd_per_rad_sec_) * encoder_ticks_per_rad_ * dt_ + prev_sensor_data_.left_encoder);
    current_sensor_data_.right_encoder = round((static_cast<double>(noisy_wheel_cmd_.right_velocity) * motor_cmd_per_rad_sec_) * encoder_ticks_per_rad_ * dt_ + prev_sensor_data_.right_encoder);

    // Set previous as current    
    prev_sensor_data_ = current_sensor_data_;

    // Simulate slipping
    double left_slip_ = wheel_slip_(get_random());  // Add slip to wheel position
    double right_slip_ = wheel_slip_(get_random());

    // Change in wheel angles with slip
    turtlelib::wheelAngles delta_wheels_{(static_cast<double>(noisy_wheel_cmd_.left_velocity) * (1 + left_slip_) * motor_cmd_per_rad_sec_) * dt_, (static_cast<double>(noisy_wheel_cmd_.right_velocity) * (1 + right_slip_) * motor_cmd_per_rad_sec_) * dt_};

    // Detect and perform required update to transform if colliding, otherwise update trasnform normally
    if(!detect_and_simulate_collison(delta_wheels_))
    {
      // Update Transform if no collision
      turtle_.driveWheels(delta_wheels_);
    }
  }

  /// \brief Publish sensor data
  void sensor_data_pub()
  {    
    sensor_data_publisher_->publish(current_sensor_data_);
  }

  /// \brief Update Simulated turtle's nav path.
  void update_red_NavPath()
  {
    // Update ground truth red turtle path
    red_path_.header.stamp = get_clock()->now();
    red_path_.header.frame_id = "nusim/world";
    // Create new pose stamped
    red_path_pose_stamped_.header.stamp = get_clock()->now();
    red_path_pose_stamped_.header.frame_id = "nusim/world";
    red_path_pose_stamped_.pose.position.x = turtle_.pose().x;
    red_path_pose_stamped_.pose.position.y = turtle_.pose().y;
    red_path_pose_stamped_.pose.position.z = 0.0;
    tf2::Quaternion q_;
    q_.setRPY(0, 0, turtle_.pose().theta);     // Rotation around z-axis
    red_path_pose_stamped_.pose.orientation.x = q_.x();
    red_path_pose_stamped_.pose.orientation.y = q_.y();
    red_path_pose_stamped_.pose.orientation.z = q_.z();
    red_path_pose_stamped_.pose.orientation.w = q_.w();
    // Append pose stamped
    red_path_.poses.push_back(red_path_pose_stamped_);
  }

  void scan_obstacles()
  {
    // Get transform from robot to world
    turtlelib::Transform2D T_world_red_{{turtle_.pose().x, turtle_.pose().y}, turtle_.pose().theta};
    turtlelib::Transform2D T_red_world_ = T_world_red_.inv();

    visualization_msgs::msg::MarkerArray sensed_obstacles_temp_;

    for (size_t i = 0; i < obstacles_x_.size(); i++)
    {

      visualization_msgs::msg::Marker sensed_obstacle_;

      sensed_obstacle_.header.frame_id = "red/base_footprint";
      sensed_obstacle_.header.stamp = get_clock()->now();
      sensed_obstacle_.id = i;
      sensed_obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;

      // Find local coordinates of obstacle
      turtlelib::Point2D obstacle_pos_world_{obstacles_x_.at(i), obstacles_y_.at(i)};
      turtlelib::Point2D obstacle_pos_robot_ = T_red_world_(obstacle_pos_world_);
      
      // Add noise only in radially outward direcction
      double laser_noise_now = laser_noise_(get_random());
      turtlelib::Point2D noisy_obstacle_pos_robot_= obstacle_pos_robot_ 
                                                    + turtlelib::Vector2D{
                                                      laser_noise_now * cos(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x)),
                                                      laser_noise_now * sin(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x))};

      sensed_obstacle_.pose.position.x = noisy_obstacle_pos_robot_.x;
      sensed_obstacle_.pose.position.y = noisy_obstacle_pos_robot_.y;

      // Set the marker's action depending on how far it is
      if (std::sqrt(std::pow(noisy_obstacle_pos_robot_.x, 2) + std::pow(noisy_obstacle_pos_robot_.y, 2)) > max_range_) 
      {
        sensed_obstacle_.action = visualization_msgs::msg::Marker::DELETE; // Delete if farther away than max range
      } 
      else 
      {
        sensed_obstacle_.action = visualization_msgs::msg::Marker::ADD; // Add if within the max range
      }

      sensed_obstacle_.pose.position.z = obstacles_h_ / 2.0;
      sensed_obstacle_.pose.orientation.x = 0.0;
      sensed_obstacle_.pose.orientation.y = 0.0;
      sensed_obstacle_.pose.orientation.z = 0.0;
      sensed_obstacle_.pose.orientation.w = 1.0;
      sensed_obstacle_.scale.x = obstacles_r_ * 2.0;   // Diameter in x
      sensed_obstacle_.scale.y = obstacles_r_ * 2.0;   // Diameter in y
      sensed_obstacle_.scale.z = obstacles_h_;         // Height
      sensed_obstacle_.color.r = 1.0f;
      sensed_obstacle_.color.g = 1.0f;
      sensed_obstacle_.color.b = 0.0f;
      sensed_obstacle_.color.a = 1.0;
      sensed_obstacles_temp_.markers.push_back(sensed_obstacle_);

      sensed_obstacles_ = sensed_obstacles_temp_;
    }
  }

  bool detect_and_simulate_collison(turtlelib::wheelAngles predicted_delta_wheels_)
  {    
    // Predicted robot motion
    turtlelib::DiffDrive predicted_turtle_ = turtle_;
    predicted_turtle_.driveWheels(predicted_delta_wheels_);   

    turtlelib::Transform2D T_world_robot_{{predicted_turtle_.pose().x, predicted_turtle_.pose().y}, predicted_turtle_.pose().theta};
    turtlelib::Transform2D T_robot_world_ = T_world_robot_.inv(); 

    for (size_t i = 0; i < obstacles_x_.size(); i++)
    {
      // Find local coordinates of obstacle
      turtlelib::Point2D obstacle_pos_world_{obstacles_x_.at(i), obstacles_y_.at(i)};
      turtlelib::Point2D obstacle_pos_robot_ = T_robot_world_(obstacle_pos_world_);

      // Detect collision
      if (std::sqrt(std::pow(obstacle_pos_robot_.x, 2) + std::pow(obstacle_pos_robot_.y, 2)) < (collision_radius_ + obstacles_r_)) 
      {
        // If colliding, calculate shift of robot frame, in robot frame
        turtlelib::Vector2D robotshift_robot_{
        -((collision_radius_ + obstacles_r_) * cos(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x)) - obstacle_pos_robot_.x),
        -((collision_radius_ + obstacles_r_) * sin(atan2(obstacle_pos_robot_.y, obstacle_pos_robot_.x)) - obstacle_pos_robot_.y)
        };

        // Calculate transform corresponding to this shift
        turtlelib::Transform2D T_robot_newrobot_{robotshift_robot_};

        // Define this transformation in the world frame
        turtlelib::Transform2D T_world_newrobot_ = T_world_robot_ * T_robot_newrobot_;

        if(lie_group_collision_)
        {
          turtle_.q.x = T_world_newrobot_.translation().x;
          turtle_.q.y = T_world_newrobot_.translation().y;
          turtle_.q.theta = T_world_newrobot_.rotation();
        }
        turtle_.phi.left = turtlelib::normalize_angle(turtle_.phi.left + predicted_delta_wheels_.left); // TODO: wheel rotation not working properly
        turtle_.phi.right = turtlelib::normalize_angle(turtle_.phi.right + predicted_delta_wheels_.right);

        RCLCPP_ERROR(this->get_logger(), "turtle: %f B: %f", obstacle_pos_robot_.x, obstacle_pos_robot_.y);

        return true; // Colliding with one obstacle, therefore, ignore other obstacles
      } 
    }
    return false; // Not colliding

    throw std::runtime_error("Invalid collision! Check collision simulation!");
  }

  /// \brief Main simulation time loop
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = ++timestep_;
    timestep_publisher_->publish(message);
    obstacles_publisher_->publish(obstacles_);
    walls_publisher_->publish(walls_);

    scan_obstacles();
    
    sensor_data_pub();

    broadcast_red_turtle();

    red_path_publisher_->publish(red_path_);

    // Publish at fake_sensor_frequency_ despite the timer frequency
    if (timestep_ % static_cast<int>(rate / fake_sensor_frequency_) == 1)
    {
      fake_sensor_publisher_->publish(sensed_obstacles_);
    }
  }

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (  wheel_radius_ == -1.0 ||
          track_width_ == -1.0 ||
          encoder_ticks_per_rad_ == -1.0 ||
          motor_cmd_per_rad_sec_ == -1.0 ||
          input_noise_ == -1.0 ||
          slip_fraction_ == -1.0 ||
          basic_sensor_variance_ == -1.0 ||
          max_range_ == -1.0 ||
          collision_radius_ == -1.0
          )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param rate: %d", rate);
      RCLCPP_DEBUG(this->get_logger(), "Param wheel_radius: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param track_width: %f", track_width_);
      RCLCPP_DEBUG(this->get_logger(), "Param encoder_ticks_per_rad: %f", encoder_ticks_per_rad_);
      RCLCPP_DEBUG(this->get_logger(), "Param motor_cmd_per_rad_sec: %f", motor_cmd_per_rad_sec_);
      RCLCPP_DEBUG(this->get_logger(), "Param input_noise: %f", input_noise_);
      RCLCPP_DEBUG(this->get_logger(), "Param slip_fraction: %f", slip_fraction_);
      RCLCPP_DEBUG(this->get_logger(), "Param basic_sensor_variance: %f", basic_sensor_variance_);
      RCLCPP_DEBUG(this->get_logger(), "Param max_range: %f", max_range_);
      RCLCPP_DEBUG(this->get_logger(), "Param collision_radius: %f", collision_radius_);
      
      throw std::runtime_error("Missing necessary parameters in diff_params.yaml!");
    }

    if (  wheel_radius_ <= 0.0 ||
          track_width_ <= 0.0 ||
          encoder_ticks_per_rad_ <= 0.0 ||
          motor_cmd_per_rad_sec_ <= 0.0 ||
          input_noise_ < 0.0 ||
          slip_fraction_ < 0.0 ||
          basic_sensor_variance_ < 0.0 ||
          max_range_ <= 0.0 ||
          collision_radius_ < 0.0
          )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param rate: %d", rate);
      RCLCPP_DEBUG(this->get_logger(), "Param wheel_radius: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param track_width: %f", track_width_);
      RCLCPP_DEBUG(this->get_logger(), "Param encoder_ticks_per_rad: %f", encoder_ticks_per_rad_);
      RCLCPP_DEBUG(this->get_logger(), "Param motor_cmd_per_rad_sec: %f", motor_cmd_per_rad_sec_);
      RCLCPP_DEBUG(this->get_logger(), "Param input_noise: %f", input_noise_);
      RCLCPP_DEBUG(this->get_logger(), "Param slip_fraction: %f", slip_fraction_);
      RCLCPP_DEBUG(this->get_logger(), "Param basic_sensor_variance: %f", basic_sensor_variance_);
      RCLCPP_DEBUG(this->get_logger(), "Param max_range: %f", max_range_);
      RCLCPP_DEBUG(this->get_logger(), "Param collision_radius: %f", collision_radius_);
      
      throw std::runtime_error("Incorrect params in diff_params.yaml!");
    }
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
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
