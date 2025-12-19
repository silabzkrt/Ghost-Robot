#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @class GhostPublisher
 * @brief ROS 2 node that publishes Twist messages in a figure-8 pattern
 * 
 * This node generates velocity commands that create a figure-8 (lemniscate) 
 * trajectory for a robot. The pattern is parametric and time-based, publishing
 * geometry_msgs/Twist messages that can be consumed by a robot in Isaac Sim
 * or any other ROS 2-compatible simulator/robot.
 */
class GhostPublisher : public rclcpp::Node
{
public:
  GhostPublisher()
  : Node("ghost_publisher"), time_(0.0)
  {
    // Declare parameters with default values
    this->declare_parameter<double>("linear_scale", 0.5);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<double>("frequency", 0.5);
    this->declare_parameter<int>("publish_rate_hz", 50);
    
    // Get parameters
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    frequency_ = this->get_parameter("frequency").as_double();
    int publish_rate = this->get_parameter("publish_rate_hz").as_int();
    
    // Cache time increment to avoid parameter lookup in timer callback
    time_increment_ = 1.0 / publish_rate;
    
    // Create publisher for cmd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create timer for publishing at specified rate
    auto timer_period = std::chrono::milliseconds(1000 / publish_rate);
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&GhostPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Ghost Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Publishing to topic: cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Parameters - linear_scale: %.2f, angular_scale: %.2f, frequency: %.2f, rate: %d Hz",
                linear_scale_, angular_scale_, frequency_, publish_rate);
  }

private:
  /**
   * @brief Timer callback that computes and publishes velocity commands
   * 
   * Implements a figure-8 (lemniscate of Gerono) pattern using parametric equations:
   * x(t) = a * sin(2πft)
   * y(t) = a * sin(4πft)
   * 
   * The velocities are computed by taking the derivative of position with respect to time.
   */
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    
    // Compute the phase for the figure-8 pattern
    double omega = 2.0 * M_PI * frequency_;
    double t = time_;
    
    // Figure-8 parametric equations (lemniscate of Gerono)
    // x(t) = sin(omega * t)
    // y(t) = sin(2 * omega * t)
    
    // Compute velocities (derivatives of position)
    // dx/dt = omega * cos(omega * t)
    // dy/dt = 2 * omega * cos(2 * omega * t)
    
    double dx_dt = omega * std::cos(omega * t);
    double dy_dt = 2.0 * omega * std::cos(2.0 * omega * t);
    
    // Compute linear velocity magnitude
    double linear_velocity = linear_scale_ * std::sqrt(dx_dt * dx_dt + dy_dt * dy_dt);
    
    // Compute angular velocity from the change in heading
    // Angular velocity = d(atan2(dy, dx))/dt
    // Using the cross product method: omega = (dx * d²y - dy * d²x) / (dx² + dy²)
    
    double d2x_dt2 = -omega * omega * std::sin(omega * t);
    double d2y_dt2 = -4.0 * omega * omega * std::sin(2.0 * omega * t);
    
    double denominator = dx_dt * dx_dt + dy_dt * dy_dt;
    double angular_velocity = 0.0;
    
    if (denominator > 1e-6) {
      angular_velocity = angular_scale_ * (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / denominator;
    }
    
    // Set the message fields
    message.linear.x = linear_velocity;
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = angular_velocity;
    
    // Publish the message
    publisher_->publish(message);
    
    // Increment time using cached value
    time_ += time_increment_;
    
    // Log periodically (every 2 seconds)
    if (static_cast<int>(time_ * 10) % 20 == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Publishing - linear.x: %.3f, angular.z: %.3f", 
                   message.linear.x, message.angular.z);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double time_;
  double time_increment_;
  double linear_scale_;
  double angular_scale_;
  double frequency_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GhostPublisher>());
  rclcpp::shutdown();
  return 0;
}
