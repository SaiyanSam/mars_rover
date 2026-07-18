#define BOOST_BIND_NO_PLACEHOLDERS

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motor_controller/vel_parser_node.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;
using namespace motor_controller;

VelParserNode::VelParserNode() : rclcpp::Node("vel_parser_node") {

  // declaring params
  this->declare_parameter<std::vector<double>>(
      "hardware_distances", std::vector<double>({23.0, 25.5, 28.5, 26.0}));

  this->declare_parameter<int>("enc_min", 250);
  this->declare_parameter<int>("enc_max", 750);

  // Speed [-100, +100] * 6 = [-600, +600]
  this->declare_parameter<int>("speed_factor", 10);

  this->declare_parameter<float>("linear_limit", 1.0);
  this->declare_parameter<float>("angular_limit", 1.0);
  this->declare_parameter<float>("angular_factor", 0.0);

  // getting params
  std::vector<double> hardware_distances;
  this->get_parameter("hardware_distances", hardware_distances);

  this->get_parameter("enc_min", this->enc_min);
  this->get_parameter("enc_max", this->enc_max);
  this->get_parameter("speed_factor", this->speed_factor);

  this->get_parameter("linear_limit", this->linear_limit);
  this->get_parameter("angular_limit", this->angular_limit);
  this->get_parameter("angular_factor", this->angular_factor);

  this->d1 = hardware_distances[0];
  this->d2 = hardware_distances[1];
  this->d3 = hardware_distances[2];
  this->d4 = hardware_distances[3];

  // pubs and subs
  this->publisher = this->create_publisher<rover_msgs::msg::MotorsCommand>(
      "motors_command", 10);

  this->subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&VelParserNode::callback, this, _1));
}

void VelParserNode::callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

  auto motors_command = rover_msgs::msg::MotorsCommand();

  // ==========================================================
  // TODO 1 — Process the Incoming Velocity Command
  //
  // Objective:
  // Convert the incoming cmd_vel command into normalized linear
  // and angular values that can safely be used by the rover.
  //
  // What to implement:
  // • Read the requested linear velocity from msg->linear.x.
  // • Read the requested steering command from msg->angular.z.
  // • Clamp both values so they do not exceed the configured
  //   linear_limit and angular_limit.
  // • Compute the rover's overall speed by combining the linear
  //   and angular components.
  // • If the rover is commanded to move backwards, preserve the
  //   negative sign of the computed speed.
  //
  // Why?
  // The rover cannot execute commands larger than its physical
  // limits. Normalizing the incoming command ensures all later
  // calculations operate within a safe and predictable range.
  //
  // Hint:
  // Useful functions:
  //   • std::min()
  //   • sqrt()
  //   • pow()
  //
  // Variables available:
  //   • msg
  //   • linear_limit
  //   • angular_limit
  //   • angular_factor
  //
  // Expected outputs:
  //   • linear
  //   • angular
  //   • speed
  // ==========================================================

  // YOUR CODE HERE
  float linear = std::clamp(
      static_cast<float>(msg->linear.x),
      -this->linear_limit,
      this->linear_limit);
    
  float angular = std::clamp(
      static_cast<float>(msg->angular.z),
      -this->angular_limit,
      this->angular_limit);
    
  float speed = std::sqrt(
      std::pow(linear, 2) +
      std::pow(angular * this->angular_factor, 2));
    
  if (msg->linear.x < 0.0) {
      speed *= -1.0;
  }

  float norm_speed = this->normalize(speed, -this->linear_limit,
                                     this->linear_limit, -100, 100);
  float norm_steering = this->normalize(angular, -this->angular_limit,
                                        this->angular_limit, -100, 100) *
                        -1;

  // calculate new speeds and steerings
  std::vector<float> new_speeds =
      this->calculate_velocity(norm_speed, norm_steering);
  std::vector<float> new_ticks =
      this->calculate_target_tick(this->calculate_target_deg(norm_steering));

  // convert to int
  for (unsigned i = 0; i < new_speeds.size(); i++) {
    motors_command.drive_motor.push_back(int(new_speeds.at(i)) *
                                         this->speed_factor);
  }

  for (unsigned i = 0; i < new_ticks.size(); i++) {
    motors_command.corner_motor.push_back(int(new_ticks.at(i)));
  }

  // publish
  this->publisher->publish(motors_command);
}

float VelParserNode::normalize(float value, float old_min, float old_max,
                               float new_min, float new_max) {
  return (new_max - new_min) * ((value - old_min) / (old_max - old_min)) +
         new_min;
}

float VelParserNode::deg_to_tick(float deg, float e_min, float e_max) {
  float temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg;

  if (temp < e_min)
    temp = e_min;
  else if (temp > e_max)
    temp = e_max;

  return temp;
}

float VelParserNode::radians_to_deg(float radians) {
  float pi = atan(1) * 4;
  return radians * 180.0 / pi;
}

std::vector<float> VelParserNode::calculate_velocity(float velocity,
                                                     float radius) {

  std::vector<float> new_velocity = {0, 0, 0, 0, 0, 0};
  float new_radius = 0;

  if (velocity == 0)
    return new_velocity;

if (abs(radius) <= 5) {

    // ==========================================================
    // TODO 2 — Handle Straight-Line Driving
    //
    // Objective:
    // Compute the wheel velocities when the rover is driving
    // approximately straight.
    //
    // What to implement:
    // • Check whether the steering command is close to zero.
    // • Assign the same speed magnitude to all six wheels.
    // • Remember that the wheels on one side of the rover rotate
    //   in the opposite direction because of their mounting
    //   orientation.
    //
    // Why?
    // When driving straight, every wheel should rotate at the
    // same speed. The only difference is the direction of
    // rotation required by the wheel configuration.
    //
    // Hint:
    // • Store the computed values inside new_velocity.
    // • Return six wheel speeds in the following order:
    //
    //   Front Left
    //   Middle Left
    //   Back Left
    //   Front Right
    //   Middle Right
    //   Back Right
    //
    // Example:
    //
    //   velocity = 40
    //
    //   Left wheels  ->  40
    //   Right wheels -> -40
    //
    // (The sign difference is due to the wheel orientation,
    // not because the rover is turning.)
    // ==========================================================

    // YOUR CODE HERE
    new_velocity = {velocity, velocity, velocity,
                    -velocity, -velocity, -velocity};

  } else {
    // Get radius in centimeters(MAX_RADIUS(255) to MIN_RADIUS(55))
    new_radius =
        MAX_RADIUS - (((MAX_RADIUS - MIN_RADIUS) * abs(radius)) / 100.0);

    float a = pow(this->d2, 2); // Back - D2
    float b = pow(this->d3, 2); // Front - D3

    float c = pow(new_radius + this->d1, 2); // Front / Back - Farthest
    float d = pow(new_radius - this->d1, 2); // Front / Back - Closest

    float e = new_radius - this->d4; // Center - Closest
    float f = new_radius + this->d4; // Center - Farthest

    float rx = 1;

    if (new_radius < 111) {
      // Front - Farthest wheel is the Farthest
      rx = sqrt(b + c);
    } else {
      // Center - Farthest wheel is the Farthest
      rx = f;
    }

    //  Get speed of each wheel
    float abs_v1 = abs(velocity) * sqrt(b + c) / rx;
    float abs_v2 = abs(velocity) * (f / rx);
    float abs_v3 = abs(velocity) * sqrt(a + c) / rx;
    float abs_v4 = abs(velocity) * sqrt(b + d) / rx;
    float abs_v5 = abs(velocity) * (e / rx);
    float abs_v6 = abs(velocity) * sqrt(a + d) / rx;

    if (velocity < 0) { //#Go back

      if (radius < 0) { // Turn Left
        new_velocity = {-abs_v4, -abs_v5, -abs_v6, abs_v1, abs_v2, abs_v3};

      } else { // Turn Right
        new_velocity = {-abs_v1, -abs_v2, -abs_v3, abs_v4, abs_v5, abs_v6};
      }

    } else { // Go ahead

      if (radius < 0) { // Turn Left
        new_velocity = {abs_v4, abs_v5, abs_v6, -abs_v1, -abs_v2, -abs_v3};

      } else { // Turn Right
        new_velocity = {abs_v1, abs_v2, abs_v3, -abs_v4, -abs_v5, -abs_v6};
      }
    }
  }

  // Set the speeds between the range[-max_speed, +max_speed]
  return new_velocity;
}

std::vector<float> VelParserNode::calculate_target_deg(float radius) {

  float new_radius = 0;
  std::vector<float> angles = {0, 0, 0, 0};

  // Scaled from MAX_RADIUS (255) to MIN_RADIUS (55) centimeters
  if (radius == 0) {
    new_radius = MAX_RADIUS;
  } else if (-100 <= radius && radius <= 100) {
    new_radius = MAX_RADIUS - abs(radius) * int(MAX_RADIUS / 100);
  } else {
    new_radius = MAX_RADIUS;
  }

  if (new_radius == MAX_RADIUS) {
    return angles;
  }

  // Turn Right - Turn Left
  // Front Left - Front Right
  float ang7 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) + this->d1)));

  // Front Right - Front Left
  float ang8 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) - this->d1)));

  // Back Left - Back Right
  float ang9 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) + this->d1)));

  // Back Right - Back Left
  float ang10 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) - this->d1)));

  // ==========================================================
  // TODO 3 — Assign Steering Angles
  //
  // Objective:
  // Assign the steering angle for each steerable wheel using
  // the angles that were already computed above.
  //
  // What to implement:
  // • If the rover is turning left (radius < 0):
  //     - The left wheels become the inner wheels.
  //     - The right wheels become the outer wheels.
  // • If the rover is turning right:
  //     - The right wheels become the inner wheels.
  //     - The left wheels become the outer wheels.
  //
  // Why?
  // During a turn, the inner wheels must steer more sharply than
  // the outer wheels so that every wheel follows the same
  // instantaneous turning circle. This reduces wheel slip and
  // results in smoother steering.
  //
  // Hint:
  // The steering angles have already been calculated:
  //
  //   ang7
  //   ang8
  //   ang9
  //   ang10
  //
  // Your task is NOT to compute these values again.
  // Simply assign them to the `angles` vector in the correct
  // order and with the appropriate sign.
  //
  // Return the steering angles in this order:
  //
  //   [ Front Left,
  //     Front Right,
  //     Back Left,
  //     Back Right ]
  // ==========================================================

  // YOUR CODE HERE
  if (radius < 0) {
    // Turn Left
    angles = {-ang8, -ang7, ang10, ang9};
  } else {
    // Turn Right
    angles = {ang7, ang8, -ang9, -ang10};
  }

  return angles;
}

std::vector<float>
VelParserNode::calculate_target_tick(std::vector<float> target_angles) {
  std::vector<float> tick;

  for (int i = 0; i < 4; i++) {
    tick.push_back(
        this->deg_to_tick(target_angles[i], this->enc_min, this->enc_max));
  }

  return tick;
}
